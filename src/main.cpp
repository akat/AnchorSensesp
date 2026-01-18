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
#include <esp_attr.h>

static const char* ANCHOR_TAG = "AnchorController";
using namespace sensesp;

// ----------- Helpers -----------
static String isoTimestamp() {
  time_t now;
  time(&now);

  // Always return a timestamp - use epoch if NTP not synced yet
  // (NTP synced check: epoch > Jan 1, 2001)
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
  int pulse_debounce_ms = 150;

  float chain_out_meters = 0.0f;
  int chain_pulse_count = 0;
  bool last_sensor_state = HIGH;
  unsigned long last_pulse_ms = 0;
  unsigned long sensor_stable_since = 0;
  bool sensor_stable_state = HIGH;
  float last_saved_chain_meters = 0.0f;
  unsigned long last_chain_save_ms = 0;

  enum RunState { IDLE, RUNNING_UP, RUNNING_DOWN, FAULT };
  RunState state = IDLE;

  unsigned long op_end_ms = 0, op_start_ms = 0;
  bool neutral_waiting = false;
  unsigned long neutral_until_ms = 0;
  RunState queued_dir_ = IDLE;
  float queued_dur_s_ = 0.0f;

  unsigned long last_led_toggle_ms_ = 0;
  bool led_state_ = false;
  bool relays_on_ = false;
  unsigned long last_command_ms_ = 0;
  const unsigned long command_debounce_ms_ = 250;
  String last_command_state_ = "";
  bool processing_command_ = false;
  bool state_changed_ = false;  // Flag για deferred publish από tick()

  // Εξωτερικές είσοδοι
  int ext_up_gpio = 32;
  int ext_down_gpio = 33;
  bool ext_input_active_high = false;  // Active LOW με pull-up (relay κλειστό = LOW)
  int ext_input_debounce_ms = 50;
  unsigned long last_ext_in_sample_ms = 0;
  String ext_source = "NONE";
  bool external_control_active = false;

  // Debounce state για external inputs
  struct DebounceState {
    bool raw = false;
    bool filtered = false;
    bool last_state = false;
    unsigned long stable_ms = 0;

    void update(bool input, unsigned long now, unsigned long debounce_ms) {
      raw = input;
      if (raw != last_state) {
        stable_ms = now;
        last_state = raw;
      } else if (now - stable_ms >= debounce_ms) {
        filtered = raw;
      }
    }
  };
  DebounceState ext_up_db_, ext_down_db_;

  // "Virtual" buzzer: μόνο SignalK event
  float base_threshold_m = 20.0f, step_m = 10.0f;
  int base_beeps = 1, beeps_per_step = 1;
  String beep_on_direction = "DOWN";
  float buzzer_hysteresis_m = 0.2f;
  float buzzer_last_alert_threshold = 0.0f;
  int buzzer_last_alert_beeps = 0;
  String buzzer_last_alert_time = "";

  StringSKListener* sk_command_listener = nullptr;
  FloatSKListener* sk_chain_set_listener = nullptr;
  BoolSKListener* sk_chain_reset_listener = nullptr;

  void setupPins() {
    pinMode(relay_up_pin, OUTPUT);
    pinMode(relay_down_pin, OUTPUT);
    relaysOff_();
    
    if (chain_sensor_pullup) pinMode(chain_sensor_pin, INPUT_PULLUP);
    else pinMode(chain_sensor_pin, INPUT);
    last_sensor_state = digitalRead(chain_sensor_pin);
    sensor_stable_since = millis();
    sensor_stable_state = last_sensor_state;
    
    // External inputs με pull-up (active LOW)
    if (ext_up_gpio >= 0) {
      if (ext_input_active_high) pinMode(ext_up_gpio, INPUT_PULLDOWN);
      else pinMode(ext_up_gpio, INPUT_PULLUP);
    }
    if (ext_down_gpio >= 0) {
      if (ext_input_active_high) pinMode(ext_down_gpio, INPUT_PULLDOWN);
      else pinMode(ext_down_gpio, INPUT_PULLUP);
    }
    
    ESP_LOGI(ANCHOR_TAG, "Pins configured: relay_up=%d, relay_down=%d, chain_sensor=%d", 
             relay_up_pin, relay_down_pin, chain_sensor_pin);
    ESP_LOGI(ANCHOR_TAG, "External inputs: up=%d, down=%d, active_high=%d", 
             ext_up_gpio, ext_down_gpio, ext_input_active_high);
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

  // Signal K helpers - unified template to avoid code duplication
  std::shared_ptr<SKWSClient> getConnectedWs_() {
    auto app = ::sensesp::SensESPApp::get();
    if (!app) return nullptr;
    auto ws = app->get_ws_client();
    if (!ws) return nullptr;
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return nullptr;
    return ws;
  }

  template<typename T>
  void sendSkDelta_(const char* path, T value, bool with_priority = true) {
    auto ws = this->getConnectedWs_();
    if (!ws) return;

    StaticJsonDocument<512> doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    if (with_priority) root["priority"] = "instant";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    JsonObject src = upd["source"].to<JsonObject>();
    src["label"] = "anchorSensor";
    JsonArray values = upd["values"].to<JsonArray>();
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
    String payload;
    serializeJson(doc, payload);
    ws->sendTXT(payload);
  }

  // Convenience wrappers
  void sendSkDeltaBool_(const char* path, bool value) { sendSkDelta_(path, value); }
  void sendSkDeltaString_(const char* path, const String& value) { sendSkDelta_(path, value); }
  void sendSkDeltaFloat_(const char* path, float value) { sendSkDelta_(path, value); }
  void sendSkDeltaInt_(const char* path, int value) { sendSkDelta_(path, value); }

  // Helper to add a value to a JsonArray (for batch sends)
  template<typename T>
  void addSkValue_(JsonArray& values, const char* path, T value) {
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
  }

  void updateChainCounter() {
    float prevChainOut = chain_out_meters;
    bool current_state = digitalRead(chain_sensor_pin);
    unsigned long now_ms = millis();
    
    if (current_state != last_sensor_state) {
      last_sensor_state = current_state;
      sensor_stable_since = now_ms; 
      return;
    }
    
    if (now_ms - sensor_stable_since < (unsigned long)pulse_debounce_ms) return;
    
    if (current_state != sensor_stable_state) {
      bool old_stable = sensor_stable_state;
      sensor_stable_state = current_state;
      
      if (old_stable == LOW && sensor_stable_state == HIGH) {
        if (now_ms - last_pulse_ms < (unsigned long)pulse_debounce_ms * 2) {
          ESP_LOGD(ANCHOR_TAG, "Pulse ignored (too soon: %lums)", now_ms - last_pulse_ms);
          return;
        }
        
        last_pulse_ms = now_ms;
        
        if (state == RUNNING_DOWN) {
          chain_out_meters += chain_calibration; 
          chain_pulse_count++;
          ESP_LOGI(ANCHOR_TAG, "Chain OUT: %.1fm (pulse #%d)%s", 
                   chain_out_meters, chain_pulse_count,
                   external_control_active ? " [EXT]" : "");
        } else if (state == RUNNING_UP) {
          chain_out_meters -= chain_calibration;
          if (chain_out_meters < 0.0f) chain_out_meters = 0.0f;
          chain_pulse_count--;
          if (chain_pulse_count < 0) chain_pulse_count = 0;
          ESP_LOGI(ANCHOR_TAG, "Chain IN: %.1fm (pulse #%d)%s", 
                   chain_out_meters, chain_pulse_count,
                   external_control_active ? " [EXT]" : "");
        }
        
        sendChainUpdate_();
        checkBuzzerThresholds(prevChainOut, chain_out_meters, state == RUNNING_DOWN);

        // Auto-save on significant change (5m threshold)
        float change = abs(chain_out_meters - last_saved_chain_meters);
        if (change >= 5.0f) {
          save();
          last_saved_chain_meters = chain_out_meters;
          last_chain_save_ms = now_ms;
          ESP_LOGI(ANCHOR_TAG, "Chain counter auto-saved (5m threshold): %.1fm", chain_out_meters);
        }
      }
    }
  }
  
  void resetChainCounter() {
    chain_out_meters = 0.0f;
    chain_pulse_count = 0;
    last_saved_chain_meters = 0.0f;
    last_chain_save_ms = millis();

    // Reset buzzer alert data
    buzzer_last_alert_threshold = 0.0f;
    buzzer_last_alert_beeps = 0;
    buzzer_last_alert_time = "";

    ESP_LOGI(ANCHOR_TAG, "Chain counter RESET to 0");
    sendChainUpdate_();

    // Send reset values for buzzer alerts
    sendSkDeltaFloat_("sensors.akat.anchor.alert.lastThreshold", buzzer_last_alert_threshold);
    sendSkDeltaInt_("sensors.akat.anchor.alert.lastBeeps", buzzer_last_alert_beeps);
    sendSkDeltaString_("sensors.akat.anchor.alert.firedAt", buzzer_last_alert_time);

    // Send empty buzzer event to clear any previous alerts
    StaticJsonDocument<128> resetDoc;
    JsonObject resetEv = resetDoc.to<JsonObject>();
    resetEv["beeps"] = 0;
    resetEv["threshold"] = 0.0f;
    resetEv["time"] = "";
    String resetPayload;
    serializeJson(resetEv, resetPayload);
    sendSkDeltaString_("sensors.akat.anchor.alert.buzzerEvent", resetPayload);
  }
  
  void sendChainUpdate_() {
    sendSkDelta_("sensors.akat.anchor.chainOut", chain_out_meters, false);
    sendSkDelta_("sensors.akat.anchor.chainPulses", chain_pulse_count, false);
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
  
  bool readExtInput_(int gpio) {
    if (gpio < 0) return false;
    return digitalRead(gpio) == (ext_input_active_high ? HIGH : LOW);
  }

  void handleExternalInputs_() {
    unsigned long now = millis();
    if (now - last_ext_in_sample_ms < 10) return;
    last_ext_in_sample_ms = now;

    // Read and debounce inputs
    ext_up_db_.update(readExtInput_(ext_up_gpio), now, ext_input_debounce_ms);
    ext_down_db_.update(readExtInput_(ext_down_gpio), now, ext_input_debounce_ms);

    // Conflict detection
    if (ext_up_db_.filtered && ext_down_db_.filtered) {
      if (!external_control_active || ext_source != "CONFLICT") {
        ESP_LOGW(ANCHOR_TAG, "External input CONFLICT: both UP and DOWN active!");
        ext_source = "CONFLICT"; 
        external_control_active = true;
        stopNow_("conflict");
        publishSkExternalControl_();
      }
      return;
    }
    
    // Determine state
    RunState nextInputState = IDLE;
    String newSource = "NONE";

    if (ext_up_db_.filtered) {
      nextInputState = RUNNING_UP;
      newSource = "UP";
    } else if (ext_down_db_.filtered) {
      nextInputState = RUNNING_DOWN;
      newSource = "DOWN";
    }
    
    // State changed?
    bool state_changed = (external_control_active != (nextInputState != IDLE)) || 
                         (ext_source != newSource);
    
    if (state_changed) {
      ESP_LOGI(ANCHOR_TAG, "External control: %s → %s", 
               ext_source.c_str(), newSource.c_str());
      
      ext_source = newSource;
      external_control_active = (nextInputState != IDLE);
      
      publishSkExternalControl_();
      
      if (nextInputState != IDLE) {
        runDirection_(nextInputState, 3600.0f);
      } else {
        stopNow_("external_stop");
      }
    }
  }
  
  void publishSkExternalControl_() {
    // Στείλε μόνο το external control status, όχι το state
    sendSkDeltaBool_("sensors.akat.anchor.externalControl.active", external_control_active);
    sendSkDeltaString_("sensors.akat.anchor.externalControl.source", ext_source);
  }

  // --- Virtual Buzzer ---
  void checkBuzzerThresholds(float prevChainOut, float currChainOut, bool directionDown) {
    bool beepDirMatch = (beep_on_direction == "DOWN" && directionDown) ||
                        (beep_on_direction == "UP" && !directionDown) ||
                        (beep_on_direction == "BOTH");
    if (!beepDirMatch) return;
    
    int lastStep = (int)((prevChainOut - base_threshold_m + buzzer_hysteresis_m) / step_m);
    int currStep = (int)((currChainOut - base_threshold_m) / step_m);
    
    if (prevChainOut < base_threshold_m && currChainOut >= base_threshold_m) {
      fireBuzzer_(base_beeps, currChainOut);
    } else if (currStep > lastStep && currStep >= 0) {
      int nBeeps = base_beeps + currStep * beeps_per_step;
      float thresh = base_threshold_m + currStep * step_m;
      fireBuzzer_(nBeeps, thresh);
    }
  }
  
  void fireBuzzer_(int beeps, float thresh) {
    buzzer_last_alert_threshold = thresh;
    buzzer_last_alert_beeps = beeps;
    buzzer_last_alert_time = isoTimestamp();

    StaticJsonDocument<256> doc;
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
    
    ESP_LOGI(ANCHOR_TAG, "BUZZER: %d beeps at %.1fm", beeps, thresh);
  }

  void stopNow_(const char* reason = "stop") {
    relaysOff_();
    state = IDLE;
    op_end_ms = 0;
    op_start_ms = 0;
    neutral_waiting = false;
    state_changed_ = true;  // Deferred publish από tick() - αποφυγή deadlock σε WS callback
    ESP_LOGI(ANCHOR_TAG, "Motor STOPPED: %s", reason);
  }
  
  void startRun_(RunState dir, float seconds) {
    const unsigned long now_ms = millis();
    op_start_ms = now_ms;
    op_end_ms = now_ms + (unsigned long)(seconds * 1000.0f);

    if (dir == RUNNING_UP) {
      relayUpOn_();
      state = RUNNING_UP;
      ESP_LOGI(ANCHOR_TAG, "Motor START: UP for %.1fs", seconds);
    } else if (dir == RUNNING_DOWN) {
      relayDownOn_();
      state = RUNNING_DOWN;
      ESP_LOGI(ANCHOR_TAG, "Motor START: DOWN for %.1fs", seconds);
    }

    state_changed_ = true;  // Deferred publish από tick()
  }
  
  void runDirection_(RunState dir, float seconds) {
    if (!enabled) return; 
    if (processing_command_) return;
    
    processing_command_ = true;
    const unsigned long now_ms = millis();
    
    String current_cmd = (dir == RUNNING_UP) ? "up" : (dir == RUNNING_DOWN) ? "down" : "idle";
    if (current_cmd == last_command_state_ && (now_ms - last_command_ms_ < command_debounce_ms_)) {
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
      ESP_LOGI(ANCHOR_TAG, "Direction change: entering neutral delay (%dms)", neutral_ms);
      return;
    }
    
    // Same direction → extend
    if ((dir == RUNNING_UP && state == RUNNING_UP) ||
        (dir == RUNNING_DOWN && state == RUNNING_DOWN)) {
      unsigned long remaining = (op_end_ms > now_ms) ? (op_end_ms - now_ms) : 0;
      unsigned long add_ms = (unsigned long)(dur * 1000.0f);
      unsigned long new_total = remaining + add_ms;
      op_end_ms = now_ms + new_total; 
      processing_command_ = false; 
      ESP_LOGD(ANCHOR_TAG, "Runtime extended by %.1fs", dur);
      return;
    }
    
    // Queued
    if (neutral_waiting && now_ms < neutral_until_ms) {
      queued_dir_ = dir; 
      queued_dur_s_ = dur; 
      processing_command_ = false; 
      return;
    }
    
    // Idle → start
    if (dir == IDLE) {
      stopNow_("command:idle");
    } else {
      startRun_(dir, dur);
    }
    
    processing_command_ = false;
  }

  void sendHeartbeat() {
    auto ws = this->getConnectedWs_();
    if (!ws) return;

    ESP_LOGD(ANCHOR_TAG, "DEBUG: Preparing heartbeat, free heap: %u", ESP.getFreeHeap());

    StaticJsonDocument<768> doc;
    JsonObject root = doc.to<JsonObject>();
    root["context"] = "vessels.self";
    JsonArray updates = root["updates"].to<JsonArray>();
    JsonObject upd = updates.add<JsonObject>();
    upd["source"]["label"] = "anchorSensor";
    JsonArray values = upd["values"].to<JsonArray>();

    this->addSkValue_(values, "sensors.akat.anchor.enabled", enabled);
    this->addSkValue_(values, "sensors.akat.anchor.lastUpdate", isoTimestamp());
    this->addSkValue_(values, "sensors.akat.anchor.chainOut", chain_out_meters);
    this->addSkValue_(values, "sensors.akat.anchor.state", stateToString_());

    String payload;
    serializeJson(doc, payload);
    ESP_LOGD(ANCHOR_TAG, "DEBUG: Sending heartbeat payload: %s", payload.c_str());
    ws->sendTXT(payload);
    ESP_LOGD(ANCHOR_TAG, "DEBUG: Heartbeat sent, free heap: %u", ESP.getFreeHeap());
  }
  
  // Check if SK listener callback should be processed (connected + settling done)
  bool shouldProcessSkCallback_() {
    extern SKWSConnectionState g_ws_state;
    extern unsigned long g_connection_time;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) return false;
    if (g_connection_time > 0 && (millis() - g_connection_time < 2000)) return false;
    return true;
  }

  void attachSignalK() {
    sk_command_listener = new StringSKListener("sensors.akat.anchor.command", 300);
    sk_command_listener->connect_to(new LambdaConsumer<String>([this](const String& cmd_state) {
      ESP_LOGI(ANCHOR_TAG, "DEBUG: Command listener triggered, free heap: %u", ESP.getFreeHeap());
      if (!shouldProcessSkCallback_()) {
        ESP_LOGD(ANCHOR_TAG, "Command ignored - not ready");
        return;
      }

      ESP_LOGI(ANCHOR_TAG, "Command received: %s", cmd_state.c_str());

      if (cmd_state == "running_up") {
        ESP_LOGI(ANCHOR_TAG, "DEBUG: Processing running_up command");
        if (state != RUNNING_UP) runDirection_(RUNNING_UP, 3600.0f);
      } else if (cmd_state == "running_down") {
        ESP_LOGI(ANCHOR_TAG, "DEBUG: Processing running_down command");
        if (state != RUNNING_DOWN) runDirection_(RUNNING_DOWN, 3600.0f);
      } else if (cmd_state == "freefall") {
        ESP_LOGI(ANCHOR_TAG, "DEBUG: Processing freefall command");
        runDirection_(RUNNING_DOWN, 0.0f);
      } else if (cmd_state == "idle") {
        ESP_LOGI(ANCHOR_TAG, "DEBUG: Processing idle command");
        if (state != IDLE) stopNow_("command:idle");
      } else if (cmd_state == "reset_counter") {
        ESP_LOGI(ANCHOR_TAG, "DEBUG: Processing reset_counter command");
        resetChainCounter();
      }

      ESP_LOGI(ANCHOR_TAG, "DEBUG: Command processing complete, free heap: %u", ESP.getFreeHeap());
    }));

    sk_chain_set_listener = new FloatSKListener("sensors.akat.anchor.chainOutSet", 500);
    sk_chain_set_listener->connect_to(new LambdaConsumer<float>([this](float meters) {
      if (!shouldProcessSkCallback_()) return;

      chain_out_meters = meters;
      if (chain_out_meters < 0.0f) chain_out_meters = 0.0f;
      chain_pulse_count = (int)(chain_out_meters / chain_calibration);

      ESP_LOGI(ANCHOR_TAG, "Chain counter SET to %.1fm via SignalK", chain_out_meters);
      save();
      last_saved_chain_meters = chain_out_meters;
      last_chain_save_ms = millis();
      sendChainUpdate_();
    }));

    sk_chain_reset_listener = new BoolSKListener("sensors.akat.anchor.resetChainCounter", 500);
    sk_chain_reset_listener->connect_to(new LambdaConsumer<bool>([this](bool reset) {
      if (!shouldProcessSkCallback_()) return;
      if (reset) resetChainCounter();
    }));
  }
  
  void tick() {
    const unsigned long now_ms = millis();

    // Deferred state publish (αποφυγή deadlock σε WS callback)
    if (state_changed_) {
      state_changed_ = false;
      publishState_();
    }

    // SAFETY CHECK FIRST: Stop if disconnected while running
    extern SKWSConnectionState g_ws_state;
    if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
      // Only stop if WE are controlling the motor (not external control)
      if ((state == RUNNING_UP || state == RUNNING_DOWN) && !external_control_active) {
        ESP_LOGW(ANCHOR_TAG, "SAFETY: Connection lost while motor running - STOPPING");
        stopNow_("safety:disconnected");
        return;
      }
    }

    // Handle external inputs
    handleExternalInputs_();

    // Update chain counter
    updateChainCounter();

    // Handle neutral wait queue
    if (neutral_waiting && now_ms >= neutral_until_ms) {
      neutral_waiting = false;
      if (queued_dir_ != IDLE) {
        auto qdir = queued_dir_;
        float qdur = queued_dur_s_;
        queued_dir_ = IDLE;
        queued_dur_s_ = 0.0f;
        ESP_LOGI(ANCHOR_TAG, "Neutral delay complete, starting queued direction");
        startRun_(qdir, qdur);
        return;
      }
    }

    // Check timeout
    if ((state == RUNNING_UP || state == RUNNING_DOWN) && now_ms >= op_end_ms) {
      stopNow_(state == RUNNING_UP ? "timeout:up" : "timeout:down");
    }

    // Periodic auto-save (every 30s if there's unsaved data)
    // Only check every 5s to reduce CPU load
    static unsigned long last_save_check_ms = 0;
    if (now_ms - last_save_check_ms >= 5000) {
      last_save_check_ms = now_ms;

      if (chain_out_meters != last_saved_chain_meters) {
        if (last_chain_save_ms == 0 || (now_ms - last_chain_save_ms >= 30000)) {
          ESP_LOGI(ANCHOR_TAG, "Chain counter auto-saving: current=%.1fm, last_saved=%.1fm, time_since_save=%lums",
                   chain_out_meters, last_saved_chain_meters, now_ms - last_chain_save_ms);
          save();
          last_saved_chain_meters = chain_out_meters;
          last_chain_save_ms = now_ms;
          ESP_LOGI(ANCHOR_TAG, "Chain counter auto-saved (30s periodic): %.1fm", chain_out_meters);
        }
      }
    }

    // LED blink
    if (relays_on_) {
      if (now_ms - last_led_toggle_ms_ >= 1000) {
        led_state_ = !led_state_;
        digitalWrite(LED_BUILTIN, led_state_ ? HIGH : LOW);
        last_led_toggle_ms_ = now_ms;
      }
    } else if (led_state_) {
      led_state_ = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
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
    root["pulse_debounce_ms"] = pulse_debounce_ms;
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
    if (c["pulse_debounce_ms"].is<int>()) pulse_debounce_ms = c["pulse_debounce_ms"].as<int>();
    if (c["chain_out_meters"].is<float>()) {
      chain_out_meters = c["chain_out_meters"].as<float>();
      last_saved_chain_meters = chain_out_meters;  // Sync on load
      last_chain_save_ms = millis();
    }
    if (c["ext_up_gpio"].is<int>()) ext_up_gpio = c["ext_up_gpio"].as<int>();
    if (c["ext_down_gpio"].is<int>()) ext_down_gpio = c["ext_down_gpio"].as<int>();
    if (c["ext_input_active_high"].is<bool>()) ext_input_active_high = c["ext_input_active_high"].as<bool>();
    if (c["ext_input_debounce_ms"].is<int>()) ext_input_debounce_ms = c["ext_input_debounce_ms"].as<int>();
    if (c["base_threshold_m"].is<float>()) base_threshold_m = c["base_threshold_m"].as<float>();
    if (c["step_m"].is<float>()) step_m = c["step_m"].as<float>();
    if (c["base_beeps"].is<int>()) base_beeps = c["base_beeps"].as<int>();
    if (c["beeps_per_step"].is<int>()) beeps_per_step = c["beeps_per_step"].as<int>();
    if (c["beep_on_direction"].is<const char*>()) beep_on_direction = c["beep_on_direction"].as<const char*>();
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
        "chain_calibration":{"title":"Meters per Pulse","type":"number","minimum":0.1},
        "pulse_debounce_ms":{"title":"Pulse Debounce (ms)","type":"integer","minimum":50,"maximum":500},
        "ext_up_gpio":{"title":"External UP GPIO","type":"integer","description":"Input from windlass UP relay (use -1 to disable)"},
        "ext_down_gpio":{"title":"External DOWN GPIO","type":"integer","description":"Input from windlass DOWN relay (use -1 to disable)"},
        "ext_input_active_high":{"title":"External Input Active High","type":"boolean","description":"false = Active LOW with pull-up (relay closed = LOW)"},
        "ext_input_debounce_ms":{"title":"External Input Debounce (ms)","type":"integer","minimum":10,"maximum":250},
        "base_threshold_m":{"title":"Base Threshold (m)","type":"number","minimum":0},
        "step_m":{"title":"Step (m)","type":"number","minimum":1},
        "base_beeps":{"title":"Base Beeps","type":"integer","minimum":1,"maximum":10},
        "beeps_per_step":{"title":"Beeps per Step","type":"integer","minimum":1,"maximum":5},
        "beep_on_direction":{"title":"Beep On Direction","type":"string","enum":["DOWN","UP","BOTH"]}
      }
    })###");
  }
};

namespace sensesp {
inline const String ConfigSchema(const AnchorController& obj) { 
  return obj.get_config_schema(); 
}
}

std::shared_ptr<AnchorController> anchor;
SKWSConnectionState g_ws_state = SKWSConnectionState::kSKWSDisconnected;
unsigned long g_connection_time = 0;
RTC_DATA_ATTR bool g_ws_watchdog_restart = false;
RTC_DATA_ATTR uint32_t g_ws_restart_attempts = 0;
RTC_DATA_ATTR uint32_t g_ws_last_uptime_ms = 0;
static unsigned long g_ws_restart_blocked_until_ms = 0;

static void init_ws_restart_guard() {
  const unsigned long rapid_restart_threshold_ms = 60000UL;
  const unsigned long base_backoff_ms = 5000UL;
  const unsigned long max_backoff_ms = 300000UL;
  const uint32_t backoff_start_attempt = 3;

  if (g_ws_watchdog_restart) {
    if (g_ws_last_uptime_ms > 0 &&
        g_ws_last_uptime_ms < rapid_restart_threshold_ms) {
      g_ws_restart_attempts++;
    } else {
      g_ws_restart_attempts = 1;
    }

    if (g_ws_restart_attempts >= backoff_start_attempt) {
      unsigned long backoff = base_backoff_ms;
      uint32_t exp = g_ws_restart_attempts - backoff_start_attempt;
      for (uint32_t i = 0; i < exp; i++) {
        if (backoff >= max_backoff_ms / 2) {
          backoff = max_backoff_ms;
          break;
        }
        backoff *= 2;
      }
      g_ws_restart_blocked_until_ms = millis() + backoff;
      ESP_LOGW(ANCHOR_TAG,
               "WS restart backoff active: %lu ms (attempt %lu)",
               backoff, (unsigned long)g_ws_restart_attempts);
    } else {
      g_ws_restart_blocked_until_ms = 0;
    }
  } else {
    g_ws_restart_attempts = 0;
    g_ws_restart_blocked_until_ms = 0;
  }

  g_ws_watchdog_restart = false;
  g_ws_last_uptime_ms = 0;
}

void setup() {
  SetupLogging();
  init_ws_restart_guard();
  
  SensESPAppBuilder builder;
  builder.set_hostname("anchor-guard");
  builder.set_wifi_access_point("anchor-guard", "pass12345");
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
  
  // Connection state handler
  if (auto app = ::sensesp::SensESPApp::get()) {
    auto ws = app->get_ws_client();
    if (ws) {
      ws->connect_to(new LambdaConsumer<SKWSConnectionState>([ws](SKWSConnectionState state) {
        SKWSConnectionState prev_state = g_ws_state;
        g_ws_state = state;

        ESP_LOGI(ANCHOR_TAG, "DEBUG: WS state change: %d -> %d, free heap: %u", (int)prev_state, (int)state, ESP.getFreeHeap());

        switch (state) {
          case SKWSConnectionState::kSKWSDisconnected:
            ESP_LOGW(ANCHOR_TAG, "SignalK WebSocket: Disconnected");
            g_connection_time = 0;
            if (anchor && (anchor->state == AnchorController::RUNNING_UP ||
                          anchor->state == AnchorController::RUNNING_DOWN)) {
              ESP_LOGW(ANCHOR_TAG, "SAFETY: Stopping motor due to disconnection");
              anchor->stopNow_("safety:disconnected");
            }
            break;

          case SKWSConnectionState::kSKWSAuthorizing:
            ESP_LOGI(ANCHOR_TAG, "SignalK WebSocket: Authorizing");
            break;

          case SKWSConnectionState::kSKWSConnecting:
            ESP_LOGI(ANCHOR_TAG, "SignalK WebSocket: Connecting");
            break;

          case SKWSConnectionState::kSKWSConnected:
            ESP_LOGI(ANCHOR_TAG, "SignalK WebSocket: Connected");
            g_connection_time = millis();
            ESP_LOGI(ANCHOR_TAG, "Connection settling period: 2 seconds");
            break;

          default:
            ESP_LOGD(ANCHOR_TAG, "SignalK WebSocket: state=%d", (int)state);
            break;
        }
      }));
    }
  }
  
  ESP_LOGI(ANCHOR_TAG, "Anchor Windlass Controller initialized");
  ESP_LOGI(ANCHOR_TAG, "Chain counter: %.1fm loaded from memory", anchor->chain_out_meters);
}

void loop() {
  event_loop()->tick();

  if (anchor) anchor->tick();

  unsigned long now_ms = millis();

  // Send custom subscription with clientName shortly after connection
  static bool custom_subscription_sent = false;
  if (g_ws_state == SKWSConnectionState::kSKWSConnected &&
      g_connection_time > 0 &&
      !custom_subscription_sent &&
      (now_ms - g_connection_time > 300) &&
      (now_ms - g_connection_time < 600)) {
    if (auto app = ::sensesp::SensESPApp::get()) {
      auto ws = app->get_ws_client();
      if (ws) {
        StaticJsonDocument<256> subDoc;
        subDoc["clientName"] = "Anchor Guard";
        subDoc["context"] = "vessels.self";
        JsonArray subscribe = subDoc["subscribe"].to<JsonArray>();
        JsonObject sub1 = subscribe.add<JsonObject>();
        sub1["path"] = "sensors.akat.anchor.command";
        JsonObject sub2 = subscribe.add<JsonObject>();
        sub2["path"] = "sensors.akat.anchor.chainOutSet";
        JsonObject sub3 = subscribe.add<JsonObject>();
        sub3["path"] = "sensors.akat.anchor.resetChainCounter";

        String subPayload;
        serializeJson(subDoc, subPayload);
        ESP_LOGI(ANCHOR_TAG, "Sending custom subscription with clientName: %s", subPayload.c_str());
        ws->sendTXT(subPayload);
        custom_subscription_sent = true;
      }
    }
  }

  if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
    custom_subscription_sent = false;
  }

  // Send initial heartbeat shortly after connection
  static bool initial_heartbeat_sent = false;
  if (g_ws_state == SKWSConnectionState::kSKWSConnected &&
      g_connection_time > 0 &&
      !initial_heartbeat_sent &&
      (now_ms - g_connection_time > 500) &&
      (now_ms - g_connection_time < 1000)) {
    if (anchor) {
      ESP_LOGI(ANCHOR_TAG, "Sending initial heartbeat to SignalK");
      anchor->sendHeartbeat();
      initial_heartbeat_sent = true;
    }
  }

  if (g_ws_state != SKWSConnectionState::kSKWSConnected) {
    initial_heartbeat_sent = false;
  }

  // Send heartbeat every 15 seconds (reduced from 5s to prevent server backpressure)
  static unsigned long last_heartbeat_ms = 0;
  if (g_ws_state == SKWSConnectionState::kSKWSConnected && anchor) {
    if (now_ms - last_heartbeat_ms >= 15000) {
      anchor->sendHeartbeat();
      last_heartbeat_ms = now_ms;
    }
  }

  // =====================================================
  // WATCHDOG: WiFi & WebSocket connection monitoring
  // =====================================================
  // Fail-safe approach: restart ESP32 on prolonged connection loss
  // This follows ESP32 firmware best practices for reliability
  static unsigned long wifi_disconnect_since = 0;
  static unsigned long ws_disconnect_since = 0;
  static bool ws_was_connected = false;
  static bool wifi_was_connected = false;

  // --- WiFi Watchdog ---
  if (WiFi.isConnected()) {
    wifi_was_connected = true;
    wifi_disconnect_since = 0;
  } else {
    wifi_mode_t wifi_mode = WiFi.getMode();
    if (!wifi_was_connected ||
        wifi_mode == WIFI_MODE_AP ||
        wifi_mode == WIFI_MODE_APSTA) {
      // Skip watchdog before first WiFi connection or while in AP setup mode.
      wifi_disconnect_since = 0;
    } else {
      if (wifi_disconnect_since == 0) {
        wifi_disconnect_since = now_ms;
        ESP_LOGW(ANCHOR_TAG, "WiFi: disconnected, will restart in 30 seconds");
      }

      // After 30 seconds without WiFi, restart ESP32
      if (now_ms - wifi_disconnect_since > 30000UL) {
        ESP_LOGE(ANCHOR_TAG, "WATCHDOG: WiFi lost - RESTARTING ESP32");
        delay(100);
        ESP.restart();
      }
    }
  }

  // --- WebSocket Watchdog ---
  // Immediate restart on WS disconnect to recover from server-side backpressure drops.
  // Only triggers if WiFi is up and the WS was connected before.
  const unsigned long ws_watchdog_timeout_ms = 0;  // 0 = immediate

  if (g_ws_state == SKWSConnectionState::kSKWSConnected) {
    ws_was_connected = true;
    ws_disconnect_since = 0;
  } else if (WiFi.isConnected() && ws_was_connected) {
    if (ws_disconnect_since == 0) {
      ws_disconnect_since = now_ms;
      if (g_ws_restart_blocked_until_ms > now_ms) {
        ESP_LOGW(ANCHOR_TAG,
                 "WebSocket: disconnected (was connected), restart suppressed for %lu ms",
                 g_ws_restart_blocked_until_ms - now_ms);
      } else if (ws_watchdog_timeout_ms == 0) {
        ESP_LOGW(ANCHOR_TAG,
                 "WebSocket: disconnected (was connected), restarting immediately");
      } else {
        ESP_LOGW(ANCHOR_TAG,
                 "WebSocket: disconnected (was connected), will restart in %lu seconds if not reconnected",
                 ws_watchdog_timeout_ms / 1000);
      }
    }

    if (g_ws_restart_blocked_until_ms > now_ms) {
      // Suppressed due to restart backoff.
      return;
    }

    if (ws_watchdog_timeout_ms == 0 ||
        now_ms - ws_disconnect_since >= ws_watchdog_timeout_ms) {
      if (ws_watchdog_timeout_ms == 0) {
        ESP_LOGE(ANCHOR_TAG, "WATCHDOG: WebSocket disconnected - RESTARTING ESP32");
      } else {
        ESP_LOGE(ANCHOR_TAG,
                 "WATCHDOG: WebSocket disconnected for %lu seconds - RESTARTING ESP32",
                 ws_watchdog_timeout_ms / 1000);
      }
      g_ws_watchdog_restart = true;
      g_ws_last_uptime_ms = now_ms;
      delay(100);
      ESP.restart();
    }
  } else {
    ws_disconnect_since = 0;
  }

  // Clear restart backoff after a stable connection period.
  if (g_ws_state == SKWSConnectionState::kSKWSConnected &&
      g_connection_time > 0 &&
      now_ms - g_connection_time > 120000UL) {
    if (g_ws_restart_attempts != 0 || g_ws_restart_blocked_until_ms != 0) {
      g_ws_restart_attempts = 0;
      g_ws_restart_blocked_until_ms = 0;
      ESP_LOGI(ANCHOR_TAG, "WS restart backoff cleared after stable connection");
    }
  }

  // --- Health diagnostics (every 10 minutes) ---
  static unsigned long last_health_log = 0;
  if (now_ms - last_health_log > 600000UL) {
    last_health_log = now_ms;
    ESP_LOGI(ANCHOR_TAG, "Health: Free heap=%u bytes, uptime=%lu min",
             ESP.getFreeHeap(), now_ms / 60000UL);
  }

  // --- Memory health check ---
  const uint32_t min_free_heap = 50000;  // 50KB minimum
  static bool memory_warning_logged = false;
  uint32_t free_heap = ESP.getFreeHeap();
  if (free_heap < min_free_heap) {
    if (!memory_warning_logged) {
      ESP_LOGW(ANCHOR_TAG, "MEMORY WARNING: Free heap %u < %u, restarting in 5 seconds", free_heap, min_free_heap);
      memory_warning_logged = true;
      // Delay restart to allow logging
      delay(5000);
      ESP.restart();
    }
  } else {
    memory_warning_logged = false;
  }
}
