#include <Arduino.h>
#include <time.h>
#include <ArduinoJson.h>


#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/serializable.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/net/http_server.h"
#include "sensesp/signalk/signalk_ws_client.h"
#include <ESPmDNS.h>

using namespace sensesp;

// ---------- Helpers ----------
static String isoTimestamp() {
  time_t now; time(&now);
  struct tm* tm_info = gmtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tm_info);
  return String(buf);
}

// ---------- AnchorController (χωρίς κληρονομιά) ----------
class AnchorController : public FileSystemSaveable {
 public:
  AnchorController() : FileSystemSaveable("/sensors/akat/anchor") {}
  // Ρυθμίσεις που θα φαίνονται στο UI
  int  relay_up_pin        = 26;
  int  relay_down_pin      = 27;
  bool relays_active_high  = true;
  bool enabled             = true;

  float default_chain_seconds = 5.0f; // default duration (sec)
  float max_run_seconds       = 120.0f;
  int   neutral_ms            = 400;

  // Runtime
  enum RunState { IDLE, RUNNING_UP, RUNNING_DOWN, FAULT };
  RunState state = IDLE;

  // Custom heartbeat publisher via WS (no SKOutput)

  // SK Listeners (incoming commands via SK state paths)
  StringSKListener* sk_state_listener   = nullptr;  // listens to sensors.akat.anchor.state

  // No PUT listeners: control is via SK value listeners

  // timers
  unsigned long op_end_ms = 0;
  bool          neutral_waiting = false;
  unsigned long neutral_until_ms = 0;
  RunState      queued_dir_ = IDLE;
  float         queued_dur_s_ = 0.0f;

  // periodic tasks & LED state
  unsigned long last_sk_update_ms_ = 0;
  unsigned long last_led_toggle_ms_ = 0;
  bool          led_state_ = false;
  bool          relays_on_ = false;

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
  void sendHeartbeat(bool include_enabled = false) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
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
  // Minimal wrapper to touch heartbeat when needed
  void publishState_(const char* last_cmd = nullptr) {
    sendHeartbeat(false);
  }

  // ---- Core ops ----
  void stopNow_(const char* reason = "stop") {
    relaysOff_();
    state = IDLE;
    op_end_ms = 0;
    neutral_waiting = false;
    // no-op for state publishing; heartbeat continues separately
  }
  void startRun_(RunState dir, float seconds) {
    const unsigned long now_ms = millis();
    op_end_ms = now_ms + (unsigned long)(seconds * 1000.0f);
    if (dir == RUNNING_UP) {
      relayUpOn_();
      state = RUNNING_UP;
      // heartbeat touches handled on timer
    } else {
      relayDownOn_();
      state = RUNNING_DOWN;
      // heartbeat touches handled on timer
    }
  }
  void runDirection_(RunState dir, float seconds) {
    if (!enabled) { publishState_("ignored:disabled"); return; }

    float dur = seconds;
    if (dur <= 0.0f) dur = default_chain_seconds;
    if (dur > max_run_seconds) dur = max_run_seconds;

    const unsigned long now_ms = millis();

    // αν αλλάζουμε κατεύθυνση → neutral
    if ((dir == RUNNING_UP && state == RUNNING_DOWN) ||
        (dir == RUNNING_DOWN && state == RUNNING_UP)) {
      relaysOff_();
      neutral_waiting = true;
      neutral_until_ms = now_ms + (unsigned long)neutral_ms;
      queued_dir_ = dir;
      queued_dur_s_ = dur;
      // queued; heartbeat timer continues
      return;
    }

    // ίδια κατεύθυνση → επέκταση
    if ((dir == RUNNING_UP && state == RUNNING_UP) ||
        (dir == RUNNING_DOWN && state == RUNNING_DOWN)) {
      unsigned long remaining = (op_end_ms > now_ms) ? (op_end_ms - now_ms) : 0;
      unsigned long add_ms = (unsigned long)(dur * 1000.0f);
      unsigned long new_total = remaining + add_ms;
      unsigned long max_ms = (unsigned long)(max_run_seconds * 1000.0f);
      if (new_total > max_ms) new_total = max_ms;
      op_end_ms = now_ms + new_total;
      // extend; heartbeat timer continues
      return;
    }

    // queued λόγω neutral;
    if (neutral_waiting && now_ms < neutral_until_ms) {
      queued_dir_ = dir;
      queued_dur_s_ = dur;
      // queued after neutral; heartbeat timer continues
      return;
    }

    startRun_(dir, dur);
  }
  void tick() {
    const unsigned long now_ms = millis();

    if (neutral_waiting && now_ms >= neutral_until_ms) {
      neutral_waiting = false;
      if (queued_dir_ != IDLE) {
        auto qdir = queued_dir_;
        float qdur = queued_dur_s_;
        queued_dir_ = IDLE; queued_dur_s_ = 0.0f;
        startRun_(qdir, qdur);
        return;
      }
    }
    if ((state == RUNNING_UP || state == RUNNING_DOWN) && now_ms >= op_end_ms) {
      stopNow_(state == RUNNING_UP ? "up:done" : "down:done");
    }

    // Heartbeat to Signal K every 2 seconds (custom source label)
    if (now_ms - last_sk_update_ms_ >= 2000) {
      sendHeartbeat(false);
      last_sk_update_ms_ = now_ms;
    }

    // Blink onboard LED every 1s when relays are active
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

  // ---------- ConfigItem hooks (v3 UI) ----------
  // Το ConfigItem θα καλέσει αυτά για να δείχνει / αποθηκεύει config.
  // Serializable implementation
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
    if (c.containsKey("relay_up_pin")) relay_up_pin = c["relay_up_pin"].as<int>();
    if (c.containsKey("relay_down_pin")) relay_down_pin = c["relay_down_pin"].as<int>();
    if (c.containsKey("relays_active_high")) relays_active_high = c["relays_active_high"].as<bool>();
    if (c.containsKey("enabled")) enabled = c["enabled"].as<bool>();
    if (c.containsKey("default_chain_seconds")) default_chain_seconds = c["default_chain_seconds"].as<float>();
    if (c.containsKey("max_run_seconds")) max_run_seconds = c["max_run_seconds"].as<float>();
    if (c.containsKey("neutral_ms")) neutral_ms = c["neutral_ms"].as<int>();
    setupPins();
    // Touch heartbeat after config update
    sendHeartbeat(false);
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

  // Use FileSystemSaveable's load/save/clear.

  // ---------- Signal K glue ----------
  void attachSignalK() {
    // No SKOutput - use custom WS deltas with explicit source label

    // Listeners for remote control via Signal K value changes
    sk_state_listener = new StringSKListener("sensors.akat.anchor.state", 300);
    sk_state_listener->connect_to(new LambdaConsumer<String>([this](const String& cmd_state) {
      // React to remote state commands: running_up / running_down / idle
      if (cmd_state == "running_up") {
        if (state != RUNNING_UP) {
          // Run until told to stop (capped by max_run_seconds)
          runDirection_(RUNNING_UP, max_run_seconds);
        }
      } else if (cmd_state == "running_down") {
        if (state != RUNNING_DOWN) {
          runDirection_(RUNNING_DOWN, max_run_seconds);
        }
      } else if (cmd_state == "freefall") {
        // Drop for defaultChainSeconds
        runDirection_(RUNNING_DOWN, 0.0f);
      } else if (cmd_state == "idle") {
        if (state != IDLE) {
          stopNow_("idle:remote");
        }
      }
    }));

    // Listen for defaultChainSeconds updates from Signal K
    auto default_chain_listener = new FloatSKListener("sensors.akat.anchor.defaultChainSeconds", 500);
    default_chain_listener->connect_to(new LambdaConsumer<float>([this](float secs) {
      float v = secs < 0 ? 0.0f : secs;
      default_chain_seconds = v;
      // Touch heartbeat on config change
      sendHeartbeat(false);
    }));
  }
};

// Provide ConfigSchema overload so ConfigItemT<AnchorController> compiles
namespace sensesp {
inline const String ConfigSchema(const AnchorController& obj) {
  return obj.get_config_schema();
}
} // namespace sensesp

// --------- Globals ----------
std::shared_ptr<AnchorController> anchor;
// No separate server; we'll add handlers to the default HTTP server (port 80)

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  builder.set_hostname("sensesp-anchor");
  builder.set_wifi_access_point("SensESP-anchor", "948171");
  ::sensesp::sensesp_app = builder.get_app();

  configTime(0, 0, "pool.ntp.org");

  anchor = std::make_shared<AnchorController>();
  // Load saved configuration (if present) before initializing IO and SK
  anchor->load();

  // Δείξε το config στο UI (v3: με ConfigItem)
  ConfigItem(anchor)
    ->set_title("Anchor Controller")
    ->set_description("Relay control & timings for anchor windlass")
    ->set_sort_order(100)
    ->set_config_schema(anchor->get_config_schema());

  anchor->setupPins();
  anchor->attachSignalK();
  // Initial heartbeat and enabled=true via WS delta on connect

  // Prepare onboard LED for blink indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // On WS connect, send enabled=true and initial heartbeat
  if (auto app = ::sensesp::SensESPApp::get()) {
    auto ws = app->get_ws_client();
    if (ws) {
      ws->connect_to(new LambdaConsumer<SKWSConnectionState>([](SKWSConnectionState state) {
        if (state == SKWSConnectionState::kSKWSConnected) {
          if (anchor) anchor->sendHeartbeat(true);
        }
      }));
    }
  }

  // Add custom handlers to the default HTTP server (port 80)
  if (auto app80 = ::sensesp::SensESPApp::get()) {
    auto server = app80->get_http_server();
    if (server) {
      // Simple HTML page at /anchor
      auto ui_handler = std::make_shared<HTTPRequestHandler>(
          1 << HTTP_GET, "/anchor", [](httpd_req_t* req) {
            const char* html =
              "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
              "<title>Anchor Control</title>"
              "<style>body{font-family:sans-serif;margin:20px}button{font-size:18px;padding:8px 16px;margin:6px}input{font-size:18px;padding:6px;width:80px}</style>"
              "</head><body>"
              "<h2>Anchor Control</h2>"
              "<div><label>Seconds:</label> <input id='secs' type='number' min='0' step='1' value='5'></div>"
              "<div>"
              "<button onclick=run('down')>Run Down</button>"
              "<button onclick=run('up')>Run Up</button>"
              "<button onclick=stop()>STOP</button>"
              "</div>"
              "<script>function run(dir){var s=document.getElementById('secs').value||0;fetch('/api/anchor/run-'+dir+'?s='+encodeURIComponent(s)).then(()=>{});}"
              "function stop(){fetch('/api/anchor/stop').then(()=>{});}"
              "</script>"
              "</body></html>";
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
          });
      server->add_handler(ui_handler);

      // API: /api/anchor/run-up?s=5
      auto run_up = std::make_shared<HTTPRequestHandler>(
          1 << HTTP_GET, "/api/anchor/run-up", [](httpd_req_t* req) {
            char buf[32]; float secs = 0.0f;
            size_t len = httpd_req_get_url_query_len(req) + 1;
            if (len > 1 && len < sizeof(buf)) {
              char q[32];
              if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
                if (httpd_query_key_value(q, "s", buf, sizeof(buf)) == ESP_OK) secs = atof(buf);
              }
            }
            if (anchor) anchor->runDirection_(AnchorController::RUNNING_UP, secs);
            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
          });
      server->add_handler(run_up);

      // API: /api/anchor/run-down?s=5
      auto run_down = std::make_shared<HTTPRequestHandler>(
          1 << HTTP_GET, "/api/anchor/run-down", [](httpd_req_t* req) {
            char buf[32]; float secs = 0.0f;
            size_t len = httpd_req_get_url_query_len(req) + 1;
            if (len > 1 && len < sizeof(buf)) {
              char q[32];
              if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
                if (httpd_query_key_value(q, "s", buf, sizeof(buf)) == ESP_OK) secs = atof(buf);
              }
            }
            if (anchor) anchor->runDirection_(AnchorController::RUNNING_DOWN, secs);
            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
          });
      server->add_handler(run_down);

      // API: /api/anchor/stop
      auto stop = std::make_shared<HTTPRequestHandler>(
          1 << HTTP_GET, "/api/anchor/stop", [](httpd_req_t* req) {
            if (anchor) anchor->stopNow_("stop:http");
            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
          });
      server->add_handler(stop);
    }
  }
}

void loop() {
  event_loop()->tick();
  if (anchor) anchor->tick();
}
