#include <Arduino.h>
#include <time.h>
#include <ArduinoJson.h>
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/signalk/signalk_ws_client.h"
#include <WiFi.h>

static const char* ANCHOR_TAG = "AnchorController";
using namespace sensesp;

// ----------- Helpers -----------
static String isoTimestamp() {
  time_t now; time(&now);
  struct tm* tm_info = gmtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tm_info);
  return String(buf);
}

class AnchorController : public FileSystemSaveable {
 public:
  AnchorController() : FileSystemSaveable("/sensors/akat/anchor") {}

  // Χαρακτηριστικά relay
  int relay_up_pin = 26;
  int relay_down_pin = 27;
  bool relays_active_high = true;
  bool enabled = true;
  float default_chain_seconds = 5.0f;
  int neutral_ms = 400;

  // Sensor και chain counter
  int chain_sensor_pin = 25;
  bool chain_sensor_pullup = true;
  float chain_calibration = 1.0f;

  float chain_out_meters = 0.0f;
  int chain_pulse_count = 0;
  bool last_sensor_state = HIGH;
  unsigned long last_pulse_ms = 0;
  unsigned long sensor_stable_since = 0;
  bool sensor_stable_state = HIGH;
  const unsigned long pulse_debounce_ms = 150;

  enum RunState { IDLE, RUNNING_UP, RUNNING_DOWN, FAULT };
  RunState state = IDLE;

  unsigned long op_end_ms = 0, op_start_ms = 0;
  bool neutral_waiting = false;
  unsigned long neutral_until_ms = 0;
  RunState queued_dir_ = IDLE;
  float queued_dur_s_ = 0.0f;

  unsigned long last_sk_update_ms_ = 0;
  unsigned long last_led_toggle_ms_ = 0;
  bool led_state_ = false;
  bool relays_on_ = false;
  unsigned long last_command_ms_ = 0;
  const unsigned long command_debounce_ms_ = 250;
  String last_command_state_ = "";
  bool processing_command_ = false;

  // Εξωτερικές είσοδοι
  int ext_up_gpio = -1;
  int ext_down_gpio = -1;
  bool ext_input_active_high = true;
  int ext_input_debounce_ms = 50;
  unsigned long last_ext_in_sample_ms = 0;
  bool ext_up_filtered = false, ext_down_filtered = false;
  unsigned long ext_up_stable_ms = 0, ext_down_stable_ms = 0;
  bool ext_up_state = false, ext_down_state = false;
  String ext_source = "NONE";
  bool external_control_active = false;

  // “Virtual” buzzer: μόνο SignalK event, όχι interrupt, pin ή GPIO
  float base_threshold_m = 20.0f, step_m = 10.0f;
  int base_beeps = 1, beeps_per_step = 1;
  String beep_on_direction = "DOWN";
  float buzzer_hysteresis_m = 0.2f;
  float buzzer_last_alert_threshold = 0.0f;
  int buzzer_last_alert_beeps = 0;
  String buzzer_last_alert_time = "";

  StringSKListener* sk_command_listener = nullptr;

  void setupPins() {
    pinMode(relay_up_pin, OUTPUT);
    pinMode(relay_down_pin, OUTPUT);
    relaysOff_();
    if (chain_sensor_pullup) pinMode(chain_sensor_pin, INPUT_PULLUP);
    else pinMode(chain_sensor_pin, INPUT);
    last_sensor_state = digitalRead(chain_sensor_pin);
    sensor_stable_since = millis();
    sensor_stable_state = last_sensor_state;
    if (ext_up_gpio >= 0) pinMode(ext_up_gpio, ext_input_active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
    if (ext_down_gpio >= 0) pinMode(ext_down_gpio, ext_input_active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
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

  void updateChainCounter() {
    float prevChainOut = chain_out_meters;
    bool current_state = digitalRead(chain_sensor_pin);
    unsigned long now_ms = millis();
    if (current_state != last_sensor_state) {
      last_sensor_state = current_state;
      sensor_stable_since = now_ms; return;
    }
    if (now_ms - sensor_stable_since < pulse_debounce_ms) return;
    if (current_state != sensor_stable_state) {
      bool old_stable = sensor_stable_state;
      sensor_stable_state = current_state;
      if (old_stable == LOW && sensor_stable_state == HIGH) {
        if (now_ms - last_pulse_ms < pulse_debounce_ms * 2) return;
        last_pulse_ms = now_ms;
        if (state == RUNNING_DOWN) {
          chain_out_meters += chain_calibration; chain_pulse_count++;
        } else if (state == RUNNING_UP) {
          chain_out_meters -= chain_calibration;
          if (chain_out_meters < 0.0f) chain_out_meters = 0.0f;
          chain_pulse_count--;
          if (chain_pulse_count < 0) chain_pulse_count = 0;
        }
        sendChainUpdate_();
        checkBuzzerThresholds(prevChainOut, chain_out_meters, state == RUNNING_DOWN);
      }
    }
  }
  void resetChainCounter() {
    chain_out_meters = 0.0f;
    chain_pulse_count = 0;
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
    JsonObject v1 = values.add<JsonObject>();
    v1["path"] = "sensors.akat.anchor.chainOut";
    v1["value"] = chain_out_meters;
    JsonObject v2 = values.add<JsonObject>();
    v2["path"] = "sensors.akat.anchor.chainPulses";
    v2["value"] = chain_pulse_count;
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }
  void publishState_() {
    sendSkDeltaString_("sensors.akat.anchor.state", stateToString_());
  }
  String stateToString_() {
    switch (state) {
      case IDLE: return "idle";
      case RUNNING_UP: return "running_up";
      case RUNNING_DOWN: return "running_down";
      case FAULT: return "fault";
    }
    return "unknown";
  }
  void handleExternalInputs_() {
    unsigned long now = millis();
    if (now - last_ext_in_sample_ms < 10) return;
    last_ext_in_sample_ms = now;
    bool ext_up_raw = ext_up_gpio >= 0 ? (digitalRead(ext_up_gpio) == (ext_input_active_high ? HIGH : LOW)) : false;
    if (ext_up_raw != ext_up_state) { ext_up_stable_ms = now; ext_up_state = ext_up_raw; }
    else if (now - ext_up_stable_ms >= (unsigned long)ext_input_debounce_ms) ext_up_filtered = ext_up_raw;
    bool ext_down_raw = ext_down_gpio >= 0 ? (digitalRead(ext_down_gpio) == (ext_input_active_high ? HIGH : LOW)) : false;
    if (ext_down_raw != ext_down_state) { ext_down_stable_ms = now; ext_down_state = ext_down_raw; }
    else if (now - ext_down_stable_ms >= (unsigned long)ext_input_debounce_ms) ext_down_filtered = ext_down_raw;
    if (ext_up_filtered && ext_down_filtered) {
      if (external_control_active && ext_source != "NONE") ESP_LOGW(ANCHOR_TAG, "External input conflict: BOTH!");
      ext_source = "NONE"; external_control_active = true;
      runDirection_(IDLE, 0); publishSkExternalControl_(); return;
    }
    RunState nextInputState = IDLE; String newSource = "NONE";
    if (ext_up_filtered)      { nextInputState = RUNNING_UP;   newSource = "UP"; }
    else if (ext_down_filtered){ nextInputState = RUNNING_DOWN; newSource = "DOWN"; }
    if (external_control_active != (nextInputState != IDLE) || ext_source != newSource) {
      ESP_LOGI(ANCHOR_TAG, "Ext ctrl: %s", newSource.c_str()); ext_source = newSource;
      external_control_active = (nextInputState != IDLE);
      publishSkExternalControl_(); runDirection_(nextInputState, 3600.0f);
    }
  }
  void publishSkExternalControl_() {
    sendSkDeltaBool_("sensors.akat.anchor.externalControl.active", external_control_active);
    sendSkDeltaString_("sensors.akat.anchor.externalControl.source", ext_source);
  }

  // --- Virtual Buzzer: Publish μόνο SignalK Alert Events ---
  void checkBuzzerThresholds(float prevChainOut, float currChainOut, bool directionDown) {
    bool beepDirMatch = (beep_on_direction == "DOWN" && directionDown) ||
                        (beep_on_direction == "UP" && !directionDown) ||
                        (beep_on_direction == "BOTH");
    if (!beepDirMatch) return;
    int lastStep = (int)((prevChainOut - base_threshold_m + buzzer_hysteresis_m) / step_m);
    int currStep = (int)((currChainOut - base_threshold_m) / step_m);
    if (prevChainOut < base_threshold_m && currChainOut >= base_threshold_m) fireBuzzer_(base_beeps, currChainOut);
    else if (currStep > lastStep && currStep >= 0) {
      int nBeeps = base_beeps + currStep * beeps_per_step;
      float thresh = base_threshold_m + currStep * step_m;
      fireBuzzer_(nBeeps, thresh);
    }
  }
  void fireBuzzer_(int beeps, float thresh) {
    buzzer_last_alert_threshold = thresh;
    buzzer_last_alert_beeps = beeps;
    buzzer_last_alert_time = isoTimestamp();

    JsonDocument doc;
    JsonObject ev = doc.to<JsonObject>();
    ev["beeps"] = beeps;
    ev["threshold"] = thresh;
    ev["time"] = buzzer_last_alert_time;
    String payload;
    serializeJson(ev, payload);
    sendSkDeltaString_("sensors.akat.anchor.alert.buzzerEvent", payload);

    sendSkDeltaFloat_("sensors.akat.anchor.alert.lastThreshold", thresh);
    sendSkDeltaInt_("sensors.akat.anchor.alert.lastBeeps", beeps);
    sendSkDeltaString_("sensors.akat.anchor.alert.firedAt", buzzer_last_alert_time);
  }

  void stopNow_(const char* reason = "stop") {
    relaysOff_(); state = IDLE;
    op_end_ms = 0; op_start_ms = 0; neutral_waiting = false;
    publishState_();
  }
  void startRun_(RunState dir, float seconds) {
    const unsigned long now_ms = millis();
    op_start_ms = now_ms;
    op_end_ms = now_ms + (unsigned long)(seconds * 1000.0f);
    if (dir == RUNNING_UP)    { relayUpOn_();   state = RUNNING_UP; }
    else if (dir == RUNNING_DOWN){ relayDownOn_(); state = RUNNING_DOWN; }
    publishState_();
  }
  void runDirection_(RunState dir, float seconds) {
    if (!enabled) return; if (processing_command_) return;
    processing_command_ = true;
    const unsigned long now_ms = millis();
    String current_cmd = (dir == RUNNING_UP) ? "up" : (dir == RUNNING_DOWN) ? "down" : "idle";
    if (current_cmd == last_command_state_ && (now_ms - last_command_ms_ < command_debounce_ms_)) {
      processing_command_ = false; return;
    }
    last_command_ms_      = now_ms; last_command_state_   = current_cmd;
    float dur = seconds; if (dur <= 0.0f) dur = default_chain_seconds;
    if ((dir == RUNNING_UP && state == RUNNING_DOWN) ||
        (dir == RUNNING_DOWN && state == RUNNING_UP)) {
      relaysOff_(); neutral_waiting = true;
      neutral_until_ms = now_ms + (unsigned long)neutral_ms;
      queued_dir_ = dir; queued_dur_s_ = dur;
      processing_command_ = false; return;
    }
    if ((dir == RUNNING_UP && state == RUNNING_UP) ||
        (dir == RUNNING_DOWN && state == RUNNING_DOWN)) {
      unsigned long remaining = (op_end_ms > now_ms) ? (op_end_ms - now_ms) : 0;
      unsigned long add_ms = (unsigned long)(dur * 1000.0f);
      unsigned long new_total = remaining + add_ms;
      op_end_ms = now_ms + new_total; processing_command_ = false; return;
    }
    if (neutral_waiting && now_ms < neutral_until_ms) {
      queued_dir_ = dir; queued_dur_s_ = dur; processing_command_ = false; return;
    }
    startRun_(dir, dur); processing_command_ = false;
  }

  void sendSkDeltaBool_(const char* path, bool value) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    JsonDocument doc;  JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>(); v["path"] = path; v["value"] = value;
    String payload; serializeJson(doc, payload); ws->sendTXT(payload);
  }
  void sendSkDeltaString_(const char* path, const String& value) {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return;
    auto ws = app->get_ws_client();
    if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    JsonDocument doc;  JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>(); v["path"] = path; v["value"] = value;
    String payload; serializeJson(doc, payload); ws->sendTXT(payload);
  }
  void sendSkDeltaFloat_(const char* path, float value) { 
    auto app = ::sensesp::SensESPApp::get(); if (!app) return;
    auto ws = app->get_ws_client(); if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    JsonDocument doc;  JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
    String payload; serializeJson(doc, payload); ws->sendTXT(payload);
  }
  void sendSkDeltaInt_(const char* path, int value) { 
    auto app = ::sensesp::SensESPApp::get(); if (!app) return;
    auto ws = app->get_ws_client(); if (!ws) return;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
    JsonDocument doc;  JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
    String payload; serializeJson(doc, payload); ws->sendTXT(payload);
  }
  void sendHeartbeat(bool include_enabled = false) {
    auto app = ::sensesp::SensESPApp::get(); if (!app) return;
    auto ws = app->get_ws_client(); if (!ws) return;
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
    JsonObject v3 = values.add<JsonObject>();
    v3["path"] = "sensors.akat.anchor.chainOut";
    v3["value"] = chain_out_meters;
    JsonObject v4 = values.add<JsonObject>();
    v4["path"] = "sensors.akat.anchor.state";
    v4["value"] = stateToString_();
    String payload;
    serializeJson(doc, payload); ws->sendTXT(payload);
  }
  void attachSignalK() {
    sk_command_listener = new StringSKListener("sensors.akat.anchor.command", 300);
    sk_command_listener->connect_to(new LambdaConsumer<String>([this](const String& cmd_state) {
      extern SKWSConnectionState g_ws_state;
      extern unsigned long g_connection_time;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
      if (g_connection_time > 0 && (millis() - g_connection_time < 2000)) return;
      if (cmd_state == "running_up") {
        if (state != RUNNING_UP) runDirection_(RUNNING_UP, 3600.0f);
      } else if (cmd_state == "running_down") {
        if (state != RUNNING_DOWN) runDirection_(RUNNING_DOWN, 3600.0f);
      } else if (cmd_state == "freefall") {
        runDirection_(RUNNING_DOWN, 0.0f);
      } else if (cmd_state == "idle") {
        if (state != IDLE) stopNow_("idle:remote");
      } else if (cmd_state == "reset_counter") {
        resetChainCounter();
      }
    }));
    auto chain_set_listener = new FloatSKListener("sensors.akat.anchor.chainOutSet", 500);
    chain_set_listener->connect_to(new LambdaConsumer<float>([this](float meters) {
      extern SKWSConnectionState g_ws_state;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
      chain_out_meters = meters;
      if (chain_out_meters < 0.0f) chain_out_meters = 0.0f;
      chain_pulse_count = (int)(chain_out_meters / chain_calibration);
      sendChainUpdate_();
    }));
    auto chain_reset_listener = new BoolSKListener("sensors.akat.anchor.resetChainCounter", 500);
    chain_reset_listener->connect_to(new LambdaConsumer<bool>([this](bool reset) {
      extern SKWSConnectionState g_ws_state;
      if (g_ws_state != SKWSConnectionState::kSKWSConnected) return;
      if (reset) resetChainCounter();
    }));
  }
  void tick() {
    const unsigned long now_ms = millis();
    handleExternalInputs_();
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      if (state == RUNNING_UP || state == RUNNING_DOWN) { stopNow_("safety:not_connected"); return; }
    }
    updateChainCounter();
    if (neutral_waiting && now_ms >= neutral_until_ms) {
      neutral_waiting = false;
      if (queued_dir_ != IDLE) {
        auto qdir = queued_dir_;
        float qdur = queued_dur_s_;
        queued_dir_ = IDLE; queued_dur_s_ = 0.0f;
        startRun_(qdir, qdur); return;
      }
    }
    if ((state == RUNNING_UP || state == RUNNING_DOWN) && now_ms >= op_end_ms)
      stopNow_(state == RUNNING_UP ? "up:done" : "down:done");
    if (g_ws_state == SKWSConnectionState::kSKWSConnected) {
      if (now_ms - last_sk_update_ms_ >= 2000) {
        sendHeartbeat(false);
        last_sk_update_ms_ = now_ms;
      }
    }
    if (relays_on_) {
      if (now_ms - last_led_toggle_ms_ >= 1000) {
        led_state_ = !led_state_; digitalWrite(LED_BUILTIN, led_state_ ? HIGH : LOW); last_led_toggle_ms_ = now_ms;
      }
    } else if (led_state_) { led_state_ = false; digitalWrite(LED_BUILTIN, LOW); }
  }

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
    root["chain_out_meters"] = chain_out_meters;
    root["ext_up_gpio"] = ext_up_gpio;
    root["ext_down_gpio"] = ext_down_gpio;
    root["ext_input_active_high"] = ext_input_active_high;
    root["ext_input_debounce_ms"] = ext_input_debounce_ms;
    root["base_threshold_m"] = base_threshold_m;
    root["step_m"] = step_m;
    root["base_beeps"] = base_beeps;
    root["beeps_per_step"] = beeps_per_step;
    root["beep_on_direction"] = beep_on_direction;
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
    if (c["ext_up_gpio"].is<int>()) ext_up_gpio = c["ext_up_gpio"].as<int>();
    if (c["ext_down_gpio"].is<int>()) ext_down_gpio = c["ext_down_gpio"].as<int>();
    if (c["ext_input_active_high"].is<bool>()) ext_input_active_high = c["ext_input_active_high"].as<bool>();
    if (c["ext_input_debounce_ms"].is<int>()) ext_input_debounce_ms = c["ext_input_debounce_ms"].as<int>();
    if (c["base_threshold_m"].is<float>()) base_threshold_m = c["base_threshold_m"].as<float>();
    if (c["step_m"].is<float>()) step_m = c["step_m"].as<float>();
    if (c["base_beeps"].is<int>()) base_beeps = c["base_beeps"].as<int>();
    if (c["beeps_per_step"].is<int>()) beeps_per_step = c["beeps_per_step"].as<int>();
    if (c["beep_on_direction"].is<const char*>()) beep_on_direction = c["beep_on_direction"].as<const char*>();
    setupPins(); return true;
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
        "chain_calibration":{"title":"Meters per Pulse","type":"number","minimum":0.1},
        "ext_up_gpio":{"title":"EXT UP GPIO","type":"integer"},
        "ext_down_gpio":{"title":"EXT DOWN GPIO","type":"integer"},
        "ext_input_active_high":{"title":"EXT Input Active High","type":"boolean"},
        "ext_input_debounce_ms":{"title":"EXT Input Debounce (ms)","type":"integer","minimum":10,"maximum":250},
        "base_threshold_m":{"title":"Base Threshold (m)","type":"number","minimum":0},
        "step_m":{"title":"Step (m)","type":"number","minimum":1},
        "base_beeps":{"title":"Base Beeps","type":"integer","minimum":1,"maximum":10},
        "beeps_per_step":{"title":"Beeps/Step","type":"integer","minimum":1,"maximum":5},
        "beep_on_direction":{"title":"Beep On Direction","type":"string","enum":["DOWN","UP","BOTH"]}
      }
    })###");
  }
};

namespace sensesp {
inline const String ConfigSchema(const AnchorController& obj) { return obj.get_config_schema(); }
}
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
  anchor = std::make_shared<AnchorController>(); anchor->load();
  ConfigItem(anchor)
    ->set_title("Anchor Controller")
    ->set_description("Relay control & timings for anchor windlass with chain counter")
    ->set_sort_order(100)
    ->set_config_schema(anchor->get_config_schema());
  anchor->setupPins(); anchor->attachSignalK();
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
  ESP_LOGI(ANCHOR_TAG, "Anchor Windlass Controller with Chain Counter initialized");
}
void loop() { event_loop()->tick(); if(anchor) anchor->tick(); }