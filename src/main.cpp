#include <Arduino.h>
#include <time.h>
#include <ArduinoJson.h>

#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/signalk/signalk_ws_client.h"

using namespace sensesp;

// ---------- Helpers ----------
static String isoTimestamp() {
  time_t now; time(&now);
  struct tm* tm_info = gmtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tm_info);
  return String(buf);
}

// ---------- AnchorController ----------
class AnchorController : public FileSystemSaveable {
 public:
  AnchorController() : FileSystemSaveable("/sensors/akat/anchor") {}
  
  // Configuration (visible in UI)
  int  relay_up_pin        = 26;
  int  relay_down_pin      = 27;
  bool relays_active_high  = true;
  bool enabled             = true;
  float default_chain_seconds = 5.0f;
  float max_run_seconds       = 120.0f;
  int   neutral_ms            = 400;

  // Runtime state
  enum RunState { IDLE, RUNNING_UP, RUNNING_DOWN, FAULT };
  RunState state = IDLE;

  // SK Listeners
  StringSKListener* sk_state_listener = nullptr;

  // Timers
  unsigned long op_end_ms = 0;
  bool          neutral_waiting = false;
  unsigned long neutral_until_ms = 0;
  RunState      queued_dir_ = IDLE;
  float         queued_dur_s_ = 0.0f;

  // Periodic tasks & LED state
  unsigned long last_sk_update_ms_ = 0;
  unsigned long last_led_toggle_ms_ = 0;
  bool          led_state_ = false;
  bool          relays_on_ = false;

  // FIX: Command debouncing to prevent listener flooding
  unsigned long last_command_ms_ = 0;
  const unsigned long command_debounce_ms_ = 250;
  String last_command_state_ = "";

  // FIX: Track if we're in the middle of processing commands
  bool processing_command_ = false;

  // ---- Pin IO ----
  void setupPins() {
    pinMode(relay_up_pin, OUTPUT);
    pinMode(relay_down_pin, OUTPUT);
    relaysOff_();
  }
  
  inline void relaysOff_() {
    digitalWrite(relay_up_pin,   relays_active_high ? LOW : HIGH);
    digitalWrite(relay_down_pin, relays_active_high ? LOW : HIGH);
    relays_on_ = false;
  }
  
  inline void relayUpOn_() {
    digitalWrite(relay_down_pin, relays_active_high ? LOW : HIGH);
    digitalWrite(relay_up_pin,   relays_active_high ? HIGH : LOW);
    relays_on_ = true;
  }
  
  inline void relayDownOn_() {
    digitalWrite(relay_up_pin,   relays_active_high ? LOW : HIGH);
    digitalWrite(relay_down_pin, relays_active_high ? HIGH : LOW);
    relays_on_ = true;
  }

  // ---- Publish helpers ----
  String stateToString_() const {
    switch (state) {
      case IDLE: return "idle";
      case RUNNING_UP: return "running_up";
      case RUNNING_DOWN: return "running_down";
      case FAULT: return "fault";
    }
    return "idle";
  }

  // ---- Signal K delta helpers ----
  void sendSkDeltaBool_(const char* path, bool value) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      return; // Silently skip if not connected
    }
    
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }

  void sendSkDeltaString_(const char* path, const String& value) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      return; // Silently skip if not connected
    }
    
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }

  void sendHeartbeat(bool include_enabled = false) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      return; // Skip if not connected
    }
    
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonObject src = upd["source"].to<JsonObject>();
    src["label"] = "signalk-anchoralarm-akat";
    JsonArray values = upd["values"].to<JsonArray>();
    
    if (include_enabled) {
      JsonObject v1 = values.add<JsonObject>();
      v1["path"] = "sensors.akat.anchor.enabled";
      v1["value"] = enabled;
    }
    
    JsonObject v2 = values.add<JsonObject>();
    v2["path"] = "sensors.akat.anchor.lastUpdate";
    v2["value"] = isoTimestamp();
    
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }

  // FIX: Remove redundant publishState to reduce WS traffic
  void publishState_(const char* last_cmd = nullptr) {
    // State publishing happens via periodic heartbeat in tick()
    // No immediate WS send to avoid flooding during reconnection
  }

  // ---- Core operations ----
  void stopNow_(const char* reason = "stop") {
    relaysOff_();
    state = IDLE;
    op_end_ms = 0;
    neutral_waiting = false;
  }

  void startRun_(RunState dir, float seconds) {
    const unsigned long now_ms = millis();
    op_end_ms = now_ms + (unsigned long)(seconds * 1000.0f);
    if (dir == RUNNING_UP) {
      relayUpOn_();
      state = RUNNING_UP;
    } else {
      relayDownOn_();
      state = RUNNING_DOWN;
    }
  }

  void runDirection_(RunState dir, float seconds) {
    if (!enabled) return;

    // FIX: Prevent re-entrant calls and rapid command flooding
    if (processing_command_) {
      ESP_LOGD(TAG, "Ignoring command - already processing");
      return;
    }
    processing_command_ = true;

    // FIX: Debounce identical commands
    const unsigned long now_ms = millis();
    String current_cmd = (dir == RUNNING_UP) ? "up" : (dir == RUNNING_DOWN) ? "down" : "idle";
    if (current_cmd == last_command_state_ && 
        (now_ms - last_command_ms_ < command_debounce_ms_)) {
      processing_command_ = false;
      return;
    }
    last_command_ms_ = now_ms;
    last_command_state_ = current_cmd;

    float dur = seconds;
    if (dur <= 0.0f) dur = default_chain_seconds;
    if (dur > max_run_seconds) dur = max_run_seconds;

    // Change direction → neutral pause
    if ((dir == RUNNING_UP && state == RUNNING_DOWN) ||
        (dir == RUNNING_DOWN && state == RUNNING_UP)) {
      relaysOff_();
      neutral_waiting = true;
      neutral_until_ms = now_ms + (unsigned long)neutral_ms;
      queued_dir_ = dir;
      queued_dur_s_ = dur;
      processing_command_ = false;
      return;
    }

    // Same direction → extend time
    if ((dir == RUNNING_UP && state == RUNNING_UP) ||
        (dir == RUNNING_DOWN && state == RUNNING_DOWN)) {
      unsigned long remaining = (op_end_ms > now_ms) ? (op_end_ms - now_ms) : 0;
      unsigned long add_ms = (unsigned long)(dur * 1000.0f);
      unsigned long new_total = remaining + add_ms;
      unsigned long max_ms = (unsigned long)(max_run_seconds * 1000.0f);
      if (new_total > max_ms) new_total = max_ms;
      op_end_ms = now_ms + new_total;
      processing_command_ = false;
      return;
    }

    // Queued due to neutral wait
    if (neutral_waiting && now_ms < neutral_until_ms) {
      queued_dir_ = dir;
      queued_dur_s_ = dur;
      processing_command_ = false;
      return;
    }

    startRun_(dir, dur);
    processing_command_ = false;
  }

  void tick() {
    const unsigned long now_ms = millis();

    // Process neutral wait queue
    if (neutral_waiting && now_ms >= neutral_until_ms) {
      neutral_waiting = false;
      if (queued_dir_ != IDLE) {
        auto qdir = queued_dir_;
        float qdur = queued_dur_s_;
        queued_dir_ = IDLE; 
        queued_dur_s_ = 0.0f;
        startRun_(qdir, qdur);
        return;
      }
    }

    // Check for operation timeout
    if ((state == RUNNING_UP || state == RUNNING_DOWN) && now_ms >= op_end_ms) {
      stopNow_(state == RUNNING_UP ? "up:done" : "down:done");
    }

    // FIX: Only send heartbeat if connected, and reduce frequency during active operations
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state == SKWSConnectionState::kSKWSConnected) {
      if (now_ms - last_sk_update_ms_ >= 2000) {
        sendHeartbeat(false);
        
        // Send current command state while running
        if (state == RUNNING_UP) {
          sendSkDeltaString_("sensors.akat.anchor.lastCommand", String("up:run"));
        } else if (state == RUNNING_DOWN) {
          sendSkDeltaString_("sensors.akat.anchor.lastCommand", String("down:run"));
        }
        
        last_sk_update_ms_ = now_ms;
      }
    }

    // Blink LED when relays are active
    if (relays_on_) {
      if (now_ms - last_led_toggle_ms_ >= 1000) {
        led_state_ = !led_state_;
        digitalWrite(LED_BUILTIN, led_state_ ? HIGH : LOW);
        last_led_toggle_ms_ = now_ms;
      }
    } else {
      if (led_state_) {
        led_state_ = false;
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }

  // ---------- Serialization for UI config ----------
  bool to_json(JsonObject& root) override {
    root["relay_up_pin"] = relay_up_pin;
    root["relay_down_pin"] = relay_down_pin;
    root["relays_active_high"] = relays_active_high;
    root["enabled"] = enabled;
    root["default_chain_seconds"] = default_chain_seconds;
    root["max_run_seconds"] = max_run_seconds;
    root["neutral_ms"] = neutral_ms;
    return true;
  }

  bool from_json(const JsonObject& c) override {
    if (c["relay_up_pin"].is<int>()) relay_up_pin = c["relay_up_pin"].as<int>();
    if (c["relay_down_pin"].is<int>()) relay_down_pin = c["relay_down_pin"].as<int>();
    if (c["relays_active_high"].is<bool>()) relays_active_high = c["relays_active_high"].as<bool>();
    if (c["enabled"].is<bool>()) enabled = c["enabled"].as<bool>();
    if (c["default_chain_seconds"].is<float>()) default_chain_seconds = c["default_chain_seconds"].as<float>();
    if (c["max_run_seconds"].is<float>()) max_run_seconds = c["max_run_seconds"].as<float>();
    if (c["neutral_ms"].is<int>()) neutral_ms = c["neutral_ms"].as<int>();
    setupPins();
    return true;
  }

  String get_config_schema() const {
    return FPSTR(R"###({
      "type":"object",
      "properties":{
        "relay_up_pin":{"title":"Relay UP GPIO","type":"integer"},
        "relay_down_pin":{"title":"Relay DOWN GPIO","type":"integer"},
        "relays_active_high":{"title":"Relays Active HIGH","type":"boolean"},
        "enabled":{"title":"Enabled","type":"boolean"},
        "default_chain_seconds":{"title":"Default Seconds","type":"number","minimum":0},
        "max_run_seconds":{"title":"Max Run Seconds","type":"number","minimum":1},
        "neutral_ms":{"title":"Neutral Delay (ms)","type":"integer","minimum":0}
      }
    })###");
  }

  // ---------- Signal K integration ----------
  void attachSignalK() {
    // FIX: Add connection state check in listener callback
    sk_state_listener = new StringSKListener("sensors.akat.anchor.state", 300);
    sk_state_listener->connect_to(new LambdaConsumer<String>([this](const String& cmd_state) {
      extern SKWSConnectionState g_ws_state;
      extern unsigned long g_connection_time;
      
      // FIX: Ignore listener updates during initial connection/reconnection
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
        ESP_LOGD(TAG, "Ignoring listener update - not connected");
        return;
      }

      // FIX: Add small delay on first message after connection to let things settle
      if (g_connection_time > 0 && (millis() - g_connection_time < 2000)) {
        ESP_LOGD(TAG, "Ignoring listener update - connection settling period");
        return;
      }

      // Process commands
      if (cmd_state == "running_up") {
        if (state != RUNNING_UP) {
          runDirection_(RUNNING_UP, max_run_seconds);
        }
      } else if (cmd_state == "running_down") {
        if (state != RUNNING_DOWN) {
          runDirection_(RUNNING_DOWN, max_run_seconds);
        }
      } else if (cmd_state == "freefall") {
        runDirection_(RUNNING_DOWN, 0.0f);
      } else if (cmd_state == "idle") {
        if (state != IDLE) {
          stopNow_("idle:remote");
        }
      }
    }));

    // FIX: Add connection check to config listener too
    auto default_chain_listener = new FloatSKListener("sensors.akat.anchor.defaultChainSeconds", 500);
    default_chain_listener->connect_to(new LambdaConsumer<float>([this](float secs) {
      extern SKWSConnectionState g_ws_state;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
        return;
      }
      float v = secs < 0 ? 0.0f : secs;
      default_chain_seconds = v;
    }));
  }
};

// ConfigSchema overload for ConfigItem
namespace sensesp {
inline const String ConfigSchema(const AnchorController& obj) {
  return obj.get_config_schema();
}
}

// --------- Globals ----------
std::shared_ptr<AnchorController> anchor;
SKWSConnectionState g_ws_state = SKWSConnectionState::kSKWSDisconnected;
unsigned long g_connection_time = 0; // Track when connection was established

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  builder.set_hostname("sensesp-anchor");
  builder.set_wifi_access_point("SensESP-anchor", "948171");
  ::sensesp::sensesp_app = builder.get_app();

  configTime(0, 0, "pool.ntp.org");

  anchor = std::make_shared<AnchorController>();
  anchor->load();

  ConfigItem(anchor)
    ->set_title("Anchor Controller")
    ->set_description("Relay control & timings for anchor windlass")
    ->set_sort_order(100)
    ->set_config_schema(anchor->get_config_schema());

  anchor->setupPins();
  anchor->attachSignalK();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // FIX: Enhanced connection state tracking with settling period
  if (auto app = ::sensesp::SensESPApp::get()) {
    auto ws = app->get_ws_client();
    if (ws) {
      ws->connect_to(new LambdaConsumer<SKWSConnectionState>([ws](SKWSConnectionState state) {
        SKWSConnectionState prev_state = g_ws_state;
        g_ws_state = state;
        
        switch (state) {
          case SKWSConnectionState::kSKWSDisconnected:
            ESP_LOGW(TAG, "SK WS: Disconnected");
            g_connection_time = 0;
            break;
            
          case SKWSConnectionState::kSKWSAuthorizing:
            ESP_LOGI(TAG, "SK WS: Authorizing");
            break;
            
          case SKWSConnectionState::kSKWSConnecting:
            ESP_LOGI(TAG, "SK WS: Connecting");
            break;
            
          case SKWSConnectionState::kSKWSConnected:
            ESP_LOGI(TAG, "SK WS: Connected");
            g_connection_time = millis();
            // FIX: Send enabled=true immediately on connection
            if (anchor) {
              // Use a task to send after a small delay to ensure connection is stable
              static unsigned long send_enabled_at = 0;
              send_enabled_at = millis() + 500; // Send after 500ms
            }
            break;
            
          default:
            ESP_LOGD(TAG, "SK WS: state=%d", (int)state);
            break;
        }
      }));
    }
  }
}

void loop() {
  event_loop()->tick();
  if (anchor) anchor->tick();

  // FIX: Send enabled=true shortly after connection is established
  static bool enabled_sent = false;
  if (g_ws_state == SKWSConnectionState::kSKWSConnected && 
      g_connection_time > 0 && 
      !enabled_sent &&
      (millis() - g_connection_time > 500) && 
      (millis() - g_connection_time < 1000)) {
    if (anchor) {
      ESP_LOGI(TAG, "Sending initial enabled=true to SignalK");
      anchor->sendHeartbeat(true);
      enabled_sent = true;
    }
  }
  // Reset flag when disconnected
  if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
    enabled_sent = false;
  }

  // Periodic lightweight heartbeat every 30s (without enabled flag)
  static unsigned long last_hb = 0;
  auto app = ::sensesp::SensESPApp::get();
  if (app) {
    auto ws = app->get_ws_client();
    if (ws && g_ws_state == SKWSConnectionState::kSKWSConnected) {
      unsigned long now = millis();
      if (now - last_hb > 30000UL) {
        last_hb = now;
        if (anchor) anchor->sendHeartbeat(false);
      }
    }
  }

  // Periodic WiFi diagnostics
  static unsigned long last_wifi_log = 0;
  unsigned long now_ms = millis();
  if (now_ms - last_wifi_log > 60000UL) {
    last_wifi_log = now_ms;
    if (WiFi.isConnected()) {
      ESP_LOGI(TAG, "WiFi ok: IP=%s RSSI=%d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
      ESP_LOGW(TAG, "WiFi disconnected");
    }
  }

  // FIX: Enhanced reconnect watchdog with better timing
  static unsigned long not_connected_since = 0;
  static uint8_t reconnect_attempts = 0;
  if (app) {
    auto ws = app->get_ws_client();
    if (ws) {
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
        if (not_connected_since == 0) not_connected_since = now_ms;
        
        // FIX: Wait longer before forcing reconnect (60s instead of 45s)
        if (now_ms - not_connected_since > 60000UL) {
          ESP_LOGW(TAG, "WS watchdog: forcing reconnect (attempt %d)", reconnect_attempts + 1);
          ws->connect();
          not_connected_since = now_ms;
          reconnect_attempts++;
          
          // FIX: Allow more attempts before restart (8 instead of 5)
          if (reconnect_attempts >= 8) {
            ESP_LOGE(TAG, "WS watchdog: exceeded attempts, restarting ESP32");
            delay(100);
            ESP.restart();
          }
        }
      } else {
        not_connected_since = 0;
        reconnect_attempts = 0;
      }
    }
  }
}