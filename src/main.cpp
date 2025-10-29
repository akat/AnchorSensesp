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

// ---------- AnchorController with Chain Counter ----------
class AnchorController : public FileSystemSaveable {
 public:
  AnchorController() : FileSystemSaveable("/sensors/akat/anchor") {}
  
  // Configuration (visible in UI)
  int  relay_up_pin        = 26;
  int  relay_down_pin      = 27;
  bool relays_active_high  = true;
  bool enabled             = true;
  float default_chain_seconds = 5.0f;
  int   neutral_ms            = 400;
  
  // Chain counter configuration
  int  chain_sensor_pin    = 25;       // GPIO για την μαγνητική επαφή
  bool chain_sensor_pullup = true;     // Ενεργοποίηση εσωτερικού pull-up
  float chain_calibration  = 1.0f;     // Μέτρα ανά παλμό (συνήθως 1 μέτρο/κρίκος)
  
  // Chain counter state
  float chain_out_meters   = 0.0f;     // Πόσα μέτρα αλυσίδα έχουν βγει
  int   chain_pulse_count  = 0;        // Μετρητής παλμών
  bool  last_sensor_state  = HIGH;     // Προηγούμενη κατάσταση αισθητήρα
  unsigned long last_pulse_ms = 0;     // Τελευταίος παλμός (για debounce)
  const unsigned long pulse_debounce_ms = 50; // Debounce 50ms

  // Runtime state
  enum RunState { IDLE, RUNNING_UP, RUNNING_DOWN, FAULT };
  RunState state = IDLE;

  // SK Listeners
  StringSKListener* sk_state_listener = nullptr;

  // Timers
  unsigned long op_end_ms = 0;
  unsigned long op_start_ms = 0;
  bool          neutral_waiting = false;
  unsigned long neutral_until_ms = 0;
  RunState      queued_dir_ = IDLE;
  float         queued_dur_s_ = 0.0f;

  // Periodic tasks & LED state
  unsigned long last_sk_update_ms_ = 0;
  unsigned long last_led_toggle_ms_ = 0;
  bool          led_state_ = false;
  bool          relays_on_ = false;

  // Command debouncing
  unsigned long last_command_ms_ = 0;
  const unsigned long command_debounce_ms_ = 250;
  String last_command_state_ = "";
  bool processing_command_ = false;

  // ---- Pin IO ----
  void setupPins() {
    // Relay pins
    pinMode(relay_up_pin, OUTPUT);
    pinMode(relay_down_pin, OUTPUT);
    relaysOff_();
    
    // Chain counter sensor pin
    if (chain_sensor_pullup) {
      pinMode(chain_sensor_pin, INPUT_PULLUP);
    } else {
      pinMode(chain_sensor_pin, INPUT);
    }
    last_sensor_state = digitalRead(chain_sensor_pin);
    
    ESP_LOGI(TAG, "Chain counter initialized: pin=%d, pullup=%d, cal=%.2fm/pulse", 
             chain_sensor_pin, chain_sensor_pullup, chain_calibration);
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

  // ---- Chain Counter Logic ----
  void updateChainCounter() {
    // Διάβασε την κατάσταση του αισθητήρα
    bool current_state = digitalRead(chain_sensor_pin);
    unsigned long now_ms = millis();
    
    // Ανίχνευση μετάβασης LOW→HIGH (όταν η μαγνητική επαφή ανοίξει)
    if (current_state == HIGH && last_sensor_state == LOW) {
      // Debounce: αγνόησε παλμούς που είναι πολύ κοντά
      if (now_ms - last_pulse_ms > pulse_debounce_ms) {
        last_pulse_ms = now_ms;
        
        // Μέτρησε ανάλογα με την κατεύθυνση
        if (state == RUNNING_DOWN) {
          // Κατέβασμα αγκύρας → αύξηση μέτρων
          chain_out_meters += chain_calibration;
          chain_pulse_count++;
          ESP_LOGI(TAG, "Chain OUT: %.1fm (pulse #%d)", chain_out_meters, chain_pulse_count);
        } else if (state == RUNNING_UP) {
          // Ανέβασμα αγκύρας → μείωση μέτρων
          chain_out_meters -= chain_calibration;
          if (chain_out_meters < 0.0f) chain_out_meters = 0.0f; // Δεν πάει αρνητικό
          chain_pulse_count--;
          if (chain_pulse_count < 0) chain_pulse_count = 0;
          ESP_LOGI(TAG, "Chain IN: %.1fm (pulse #%d)", chain_out_meters, chain_pulse_count);
        }
        
        // Στείλε ενημέρωση στο Signal K
        sendChainUpdate_();
      }
    }
    
    last_sensor_state = current_state;
  }
  
  void resetChainCounter() {
    chain_out_meters = 0.0f;
    chain_pulse_count = 0;
    ESP_LOGI(TAG, "Chain counter reset to 0");
    sendChainUpdate_();
  }
  
  void sendChainUpdate_() {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonObject src = upd["source"].to<JsonObject>();
    src["label"] = "signalk-anchoralarm-akat";
    JsonArray values = upd["values"].to<JsonArray>();
    
    // Μέτρα αλυσίδας
    JsonObject v1 = values.add<JsonObject>();
    v1["path"] = "sensors.akat.anchor.chainOut";
    v1["value"] = chain_out_meters;
    
    // Αριθμός παλμών
    JsonObject v2 = values.add<JsonObject>();
    v2["path"] = "sensors.akat.anchor.chainPulses";
    v2["value"] = chain_pulse_count;
    
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
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
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    
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
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    
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
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    
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
    
    // Συμπεριέλαβε την μέτρηση αλυσίδας στο heartbeat
    JsonObject v3 = values.add<JsonObject>();
    v3["path"] = "sensors.akat.anchor.chainOut";
    v3["value"] = chain_out_meters;
    
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }

  void publishState_(const char* last_cmd = nullptr) {
    // State publishing happens via periodic heartbeat
  }

  // ---- Core operations ----
  void stopNow_(const char* reason = "stop") {
    relaysOff_();
    state = IDLE;
    op_end_ms = 0;
    op_start_ms = 0;
    neutral_waiting = false;
  }

  void startRun_(RunState dir, float seconds) {
    const unsigned long now_ms = millis();
    op_start_ms = now_ms;
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

    if (processing_command_) {
      ESP_LOGD(TAG, "Ignoring command - already processing");
      return;
    }
    processing_command_ = true;

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
      
      op_end_ms = now_ms + new_total;
      
      ESP_LOGI(TAG, "Extended runtime: new end in %.1fs", new_total / 1000.0f);
      
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

    // SAFETY: Check connection state and stop if disconnected
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      if (state == RUNNING_UP || state == RUNNING_DOWN) {
        ESP_LOGW(TAG, "SAFETY: Motor running while disconnected - stopping");
        stopNow_("safety:not_connected");
        return;
      }
    }

    // Update chain counter (ανεξάρτητα από την κατάσταση σύνδεσης)
    updateChainCounter();

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

    // Send heartbeat if connected
    if (g_ws_state == SKWSConnectionState::kSKWSConnected) {
      if (now_ms - last_sk_update_ms_ >= 2000) {
        sendHeartbeat(false);
        
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
    root["neutral_ms"] = neutral_ms;
    root["chain_sensor_pin"] = chain_sensor_pin;
    root["chain_sensor_pullup"] = chain_sensor_pullup;
    root["chain_calibration"] = chain_calibration;
    root["chain_out_meters"] = chain_out_meters;  // Αποθήκευση της τρέχουσας μέτρησης
    return true;
  }

  bool from_json(const JsonObject& c) override {
    if (c["relay_up_pin"].is<int>()) relay_up_pin = c["relay_up_pin"].as<int>();
    if (c["relay_down_pin"].is<int>()) relay_down_pin = c["relay_down_pin"].as<int>();
    if (c["relays_active_high"].is<bool>()) relays_active_high = c["relays_active_high"].as<bool>();
    if (c["enabled"].is<bool>()) enabled = c["enabled"].as<bool>();
    if (c["default_chain_seconds"].is<float>()) default_chain_seconds = c["default_chain_seconds"].as<float>();
    if (c["neutral_ms"].is<int>()) neutral_ms = c["neutral_ms"].as<int>();
    if (c["chain_sensor_pin"].is<int>()) chain_sensor_pin = c["chain_sensor_pin"].as<int>();
    if (c["chain_sensor_pullup"].is<bool>()) chain_sensor_pullup = c["chain_sensor_pullup"].as<bool>();
    if (c["chain_calibration"].is<float>()) chain_calibration = c["chain_calibration"].as<float>();
    if (c["chain_out_meters"].is<float>()) chain_out_meters = c["chain_out_meters"].as<float>();
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
        "neutral_ms":{"title":"Neutral Delay (ms)","type":"integer","minimum":0},
        "chain_sensor_pin":{"title":"Chain Sensor GPIO","type":"integer"},
        "chain_sensor_pullup":{"title":"Enable Internal Pull-up","type":"boolean"},
        "chain_calibration":{"title":"Meters per Pulse","type":"number","minimum":0.1}
      }
    })###");
  }

  // ---------- Signal K integration ----------
  void attachSignalK() {
    // State listener
    sk_state_listener = new StringSKListener("sensors.akat.anchor.state", 300);
    sk_state_listener->connect_to(new LambdaConsumer<String>([this](const String& cmd_state) {
      extern SKWSConnectionState g_ws_state;
      extern unsigned long g_connection_time;
      
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
        ESP_LOGD(TAG, "Ignoring listener update - not connected");
        return;
      }

      if (g_connection_time > 0 && (millis() - g_connection_time < 2000)) {
        ESP_LOGD(TAG, "Ignoring listener update - connection settling period");
        return;
      }

      if (cmd_state == "running_up") {
        if (state != RUNNING_UP) {
          runDirection_(RUNNING_UP, 3600.0f);
        }
      } else if (cmd_state == "running_down") {
        if (state != RUNNING_DOWN) {
          runDirection_(RUNNING_DOWN, 3600.0f);
        }
      } else if (cmd_state == "freefall") {
        runDirection_(RUNNING_DOWN, 0.0f);
      } else if (cmd_state == "idle") {
        if (state != IDLE) {
          stopNow_("idle:remote");
        }
      } else if (cmd_state == "reset_counter") {
        // Εντολή για reset του μετρητή αλυσίδας
        resetChainCounter();
      }
    }));

    // Default chain seconds listener
    auto default_chain_listener = new FloatSKListener("sensors.akat.anchor.defaultChainSeconds", 500);
    default_chain_listener->connect_to(new LambdaConsumer<float>([this](float secs) {
      extern SKWSConnectionState g_ws_state;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
      float v = secs < 0 ? 0.0f : secs;
      default_chain_seconds = v;
    }));
    
    // Chain counter reset listener
    auto chain_reset_listener = new BoolSKListener("sensors.akat.anchor.resetChainCounter", 500);
    chain_reset_listener->connect_to(new LambdaConsumer<bool>([this](bool reset) {
      extern SKWSConnectionState g_ws_state;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
      if (reset) {
        resetChainCounter();
      }
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
unsigned long g_connection_time = 0;

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
    ->set_description("Relay control & timings for anchor windlass with chain counter")
    ->set_sort_order(100)
    ->set_config_schema(anchor->get_config_schema());

  anchor->setupPins();
  anchor->attachSignalK();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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
            if (anchor && (anchor->state == AnchorController::RUNNING_UP || 
                          anchor->state == AnchorController::RUNNING_DOWN)) {
              ESP_LOGW(TAG, "SAFETY: Stopping motor due to SignalK disconnection");
              anchor->stopNow_("safety:disconnected");
            }
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
            break;
            
          default:
            ESP_LOGD(TAG, "SK WS: state=%d", (int)state);
            break;
        }
      }));
    }
  }
  
  ESP_LOGI(TAG, "Anchor Windlass Controller with Chain Counter initialized");
}

void loop() {
  event_loop()->tick();
  if (anchor) anchor->tick();

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
  if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
    enabled_sent = false;
  }

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

  static unsigned long not_connected_since = 0;
  static uint8_t reconnect_attempts = 0;
  if (app) {
    auto ws = app->get_ws_client();
    if (ws) {
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
        if (not_connected_since == 0) not_connected_since = now_ms;
        
        if (now_ms - not_connected_since > 60000UL) {
          ESP_LOGW(TAG, "WS watchdog: forcing reconnect (attempt %d)", reconnect_attempts + 1);
          ws->connect();
          not_connected_since = now_ms;
          reconnect_attempts++;
          
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