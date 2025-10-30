# SensESP Anchor Windlass Controller with Chain Counter

A Signal K-based anchor windlass controller for ESP32 that provides safe, remote operation of electric anchor windlass motors through relay control with integrated chain length measurement.

## Overview

This controller integrates with your boat's Signal K network to provide remote control of your anchor windlass with automatic chain length tracking. It uses two relays to control motor direction (UP/DOWN), a magnetic reed switch to count chain links, and includes multiple safety features to protect the motor and prevent damage.

## Features

### Core Functionality
- **Dual-relay motor control** - Independent UP and DOWN relay channels
- **Chain length counter** - Automatic measurement using magnetic reed switch
- **Signal K integration** - Full integration with marine data network
- **Remote operation** - Control from any Signal K-compatible app
- **Web configuration** - Easy setup through SensESP web interface
- **Persistent settings** - Configuration and chain count saved to ESP32 flash memory

### Chain Counter Features
- **Automatic measurement** - Counts chain links in real-time
- **Bidirectional counting** - Tracks chain deployment (DOWN) and retrieval (UP)
- **Advanced debouncing** - 150ms debounce with state validation to prevent double-counting
- **Manual adjustment** - Set counter to any value via Signal K
- **Calibration support** - Adjustable meters per pulse for different chain types
- **Persistent storage** - Remembers chain length after power loss
- **Multiple reset options** - Reset to zero or set to specific value

### Safety Features
- **Neutral delay** - Mandatory pause when reversing direction to prevent mechanical damage
- **Connection monitoring** - Automatic motor stop if Signal K connection is lost
- **Command debouncing** - Prevents command flooding during network reconnections
- **Re-entrancy protection** - Prevents conflicting simultaneous commands
- **Visual feedback** - Onboard LED blinks when motor is running

### External Control
- **GPIO inputs** - Optional external buttons/switches for UP/DOWN control
- **Debounced inputs** - Configurable debounce time for external controls
- **Conflict detection** - Prevents simultaneous UP/DOWN activation

### Virtual Buzzer Alerts
- **Threshold-based alerts** - Configurable chain length thresholds
- **Directional alerts** - Alerts on deployment, retrieval, or both
- **Signal K events** - Publishes alert events with beep counts and timestamps

### Network Protection
- **Connection settling period** - Ignores cached values for 2 seconds after reconnection
- **Reconnection handling** - Automatic reconnection with watchdog timer
- **WiFi monitoring** - Periodic diagnostics and auto-recovery

## Hardware Requirements

### ESP32 Board
- Any ESP32 development board (tested on ESP32-WROOM-32)
- Minimum 4MB flash memory recommended

### Relay Module
- 2-channel relay module (5V or 3.3V compatible)
- Active HIGH or Active LOW configurable
- Recommended: Optocoupler-isolated relays
- Minimum rating: Match your windlass motor current

### Chain Counter Sensor
- **Magnetic reed switch** (normally open)
- Mount near chain gypsy/windlass
- Attach small magnet to chain or install on moving parts
- Typical configuration: Reed switch triggers once per chain link (≈1 meter)
- **Optional**: Add 100nF capacitor across switch terminals for hardware debouncing

### Windlass Motor
- DC motor with UP and DOWN wiring
- Common ground configuration
- Motor current within relay specifications

### Power Supply
- 5V power for ESP32 (USB or dedicated supply)
- Separate power supply for windlass motor (12V/24V typical)
- **Important**: ESP32 and motor should share common ground

### Optional External Controls
- Push buttons or switches for UP/DOWN
- Configurable GPIO pins and active level

## Wiring Diagram

```
┌─────────────┐
│   ESP32     │
│             │
│  GPIO 26 ───┼──→ Relay 1 (UP) ──→ Motor UP terminal
│  GPIO 27 ───┼──→ Relay 2 (DOWN) ─→ Motor DOWN terminal
│             │
│  GPIO 25 ───┼──→ Reed Switch ─────→ GND
│             │     (with internal pull-up)
│             │     Optional: 100nF capacitor parallel
│             │
│  GPIO 12 ───┼──→ EXT UP Button (optional)
│  GPIO 13 ───┼──→ EXT DOWN Button (optional)
│             │
│  GND ───────┼──→ Common Ground ←── Motor Ground
│             │
│  5V ────────┼──→ ESP32 Power
└─────────────┘

Motor Power Supply (12V/24V)
      │
      └──→ Relay COM terminals

Chain Counter Setup:
┌────────────────────────────┐
│  Windlass/Chain Gypsy      │
│                            │
│  [Magnet] ←─ on chain      │
│     ↓  ↑                   │
│  [Reed Switch] ←─ mounted  │
│     │                      │
│     └──→ GPIO 25           │
└────────────────────────────┘

Optional Hardware Debounce:
Reed Switch
    ├──→ GPIO 25
    ├──→ 100nF capacitor
    └──→ GND
```

### Default GPIO Pins
- **Relay UP**: GPIO 26
- **Relay DOWN**: GPIO 27
- **Chain Sensor**: GPIO 25 (with internal pull-up)
- **EXT UP**: GPIO 12 (optional)
- **EXT DOWN**: GPIO 13 (optional)
- **LED indicator**: Built-in LED (GPIO 2 on most boards)

*Note: All pins are configurable through the web interface*

## Installation

### 1. Hardware Setup

#### Motor Control
1. Connect ESP32 to relay module
2. Wire relays to windlass motor
3. Ensure proper power supply and grounding
4. **Test relays manually before connecting motor**

#### Chain Counter Sensor
1. **Mount reed switch** near chain gypsy where chain passes
2. **Attach magnet** to chain or rotating part
   - Option A: Small neodymium magnet on chain link
   - Option B: Magnet on rotating gear/sprocket
3. **Position sensor** so magnet passes within 5-10mm
4. **Wire sensor**:
   - One wire to GPIO 25
   - Other wire to GND
   - Internal pull-up is enabled in software
   - Optional: Add 100nF capacitor for hardware debouncing
5. **Test sensor**: Watch serial output while manually turning windlass

#### Optional External Controls
1. Connect push buttons to configured GPIO pins
2. Configure active level (HIGH/LOW) in web interface
3. Set debounce time appropriately

### 2. Software Installation

#### Prerequisites
- PlatformIO IDE or Arduino IDE
- SensESP library (v3.1.1 or later)
- Signal K server running on your network

#### Compilation
```bash
# Using PlatformIO
pio run -t upload

# Using Arduino IDE
# 1. Install ESP32 board support
# 2. Install SensESP library
# 3. Compile and upload
```

### 3. Initial Configuration

#### First Boot
1. ESP32 creates WiFi access point: **SensESP-anchor** (Password: `948171`)
2. Connect to this network
3. Navigate to `http://192.168.4.1`
4. Configure WiFi credentials and Signal K server details

#### Web Configuration
Access the configuration interface at `http://sensesp-anchor.local` (or device IP)

**Configuration Options:**

| Setting | Description | Default | Notes |
|---------|-------------|---------|-------|
| **Motor Control** | | | |
| Relay UP GPIO | GPIO pin for UP relay | 26 | Configurable |
| Relay DOWN GPIO | GPIO pin for DOWN relay | 27 | Configurable |
| Relays Active HIGH | Relay trigger logic | true | Set false for active-low relays |
| Enabled | Enable/disable controller | true | Safety disable |
| Default Seconds | Duration for "freefall" command | 5.0 | In seconds |
| Neutral Delay | Pause when reversing direction | 400 | In milliseconds |
| **Chain Counter** | | | |
| Chain Sensor GPIO | GPIO pin for reed switch | 25 | Configurable |
| Enable Internal Pull-up | Use ESP32 internal pull-up | true | Usually keep enabled |
| Meters per Pulse | Chain length per sensor pulse | 1.0 | Calibration value |
| Pulse Debounce (ms) | Debounce time for chain sensor | 150 | 50-500ms, adjust for windlass speed |
| **External Controls** | | | |
| EXT UP GPIO | GPIO pin for external UP input | -1 | -1 to disable |
| EXT DOWN GPIO | GPIO pin for external DOWN input | -1 | -1 to disable |
| EXT Input Active High | External input trigger logic | true | Set false for active-low |
| EXT Input Debounce (ms) | Debounce time for external inputs | 50 | 10-250ms |
| **Virtual Buzzer** | | | |
| Base Threshold (m) | Starting chain length for alerts | 20.0 | In meters |
| Step (m) | Chain length increment for additional beeps | 10.0 | In meters |
| Base Beeps | Number of beeps at base threshold | 1 | 1-10 |
| Beeps/Step | Additional beeps per step | 1 | 1-5 |
| Beep On Direction | When to trigger alerts | "DOWN" | "UP", "DOWN", or "BOTH" |

### 4. Chain Counter Calibration

To ensure accurate measurements:

1. **Reset the counter** when anchor is fully retrieved
2. **Deploy a known length** of chain (e.g., measure 10 meters manually)
3. **Check the reading** in Signal K: `sensors.akat.anchor.chainOut`
4. **Calculate calibration**:
   ```
   Actual meters deployed / Reading = Calibration factor
   Example: 10.0m / 11.0m = 0.909
   ```
5. **Update config** with new calibration factor
6. **Reset and test** again to verify accuracy

**Typical calibration values:**
- Standard chain with 1m links: `1.0`
- Metric chain with 1m markers: `1.0`
- Imperial chain (3.28 ft/link): `1.0` for 1m approximation
- Custom setup: Measure and calculate

## Signal K Integration

### Published Paths

The controller publishes to Signal K:

| Path | Type | Description | Update Rate |
|------|------|-------------|-------------|
| `sensors.akat.anchor.enabled` | boolean | Controller enabled status | On connect |
| `sensors.akat.anchor.lastUpdate` | timestamp | Last heartbeat timestamp | Every 2 seconds |
| `sensors.akat.anchor.lastCommand` | string | Current operation | While running |
| `sensors.akat.anchor.chainOut` | number | Meters of chain deployed | Real-time + heartbeat |
| `sensors.akat.anchor.chainPulses` | number | Raw pulse count (debug) | On change |
| `sensors.akat.anchor.state` | string | Current state ("idle", "running_up", "running_down", "fault") | Real-time + heartbeat |
| `sensors.akat.anchor.externalControl.active` | boolean | External control active status | On change |
| `sensors.akat.anchor.externalControl.source` | string | External control source ("NONE", "UP", "DOWN") | On change |
| `sensors.akat.anchor.alert.buzzerEvent` | string | JSON alert event with beeps and threshold | On alert |
| `sensors.akat.anchor.alert.lastThreshold` | number | Last alert threshold (meters) | On alert |
| `sensors.akat.anchor.alert.lastBeeps` | number | Last alert beep count | On alert |
| `sensors.akat.anchor.alert.firedAt` | string | Last alert timestamp | On alert |

### Subscribed Paths (Commands)

Send commands by setting these Signal K values:

| Path | Value | Action |
|------|-------|--------|
| **Motor Control** | | |
| `sensors.akat.anchor.command` | `"running_up"` | Raise anchor (runs until stopped) |
| `sensors.akat.anchor.command` | `"running_down"` | Lower anchor (runs until stopped) |
| `sensors.akat.anchor.command` | `"freefall"` | Quick drop for configured seconds |
| `sensors.akat.anchor.command` | `"idle"` | Stop motor immediately |
| `sensors.akat.anchor.command` | `"reset_counter"` | Reset chain counter to zero |
| **Chain Counter Control** | | |
| `sensors.akat.anchor.chainOutSet` | number | **Set counter to specific value (meters)** |
| `sensors.akat.anchor.resetChainCounter` | boolean | Reset counter to zero (send `true`) |

### Example: Signal K Deltas

**Raise anchor:**
```json
{
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.command",
      "value": "running_up"
    }]
  }]
}
```

**Set chain counter to 25 meters:**
```json
{
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.chainOutSet",
      "value": 25.0
    }]
  }]
}
```

**Reset chain counter to zero:**
```json
{
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.resetChainCounter",
      "value": true
    }]
  }]
}
```

**Read chain length:**
```json
// Subscribe to:
"sensors.akat.anchor.chainOut"

// Receive updates like:
{
  "path": "sensors.akat.anchor.chainOut",
  "value": 15.5  // 15.5 meters of chain deployed
}
```

## Operation

### Starting Motor
1. Send `"running_up"` or `"running_down"` command
2. Motor runs for 1 hour (or until stopped)
3. LED blinks once per second while running
4. Chain counter updates in real-time
5. Heartbeat updates sent every 2 seconds

### Stopping Motor
1. Send `"idle"` command, OR
2. Connection to Signal K is lost (automatic safety stop)

### Monitoring Chain Length
- **Real-time updates** while motor is running
- **Periodic updates** in heartbeat (every 2 seconds)
- **View in Signal K apps** - data shown like any other sensor
- **Example display**: "Anchor Chain: 23.5m deployed"

### Managing Chain Counter

#### Resetting to Zero
**When to reset:**
- When anchor is fully retrieved (all chain on windlass)
- After manual chain adjustment
- When starting fresh measurement

**How to reset (2 methods):**

1. **Via State Command**:
   ```json
   {"path": "sensors.akat.anchor.command", "value": "reset_counter"}
   ```

2. **Via Boolean**:
   ```json
   {"path": "sensors.akat.anchor.resetChainCounter", "value": true}
   ```

#### Setting to Specific Value
**When to use:**
- You know exactly how much chain is deployed
- Correcting drift/measurement errors
- Synchronizing with manual measurement
- Starting with chain already deployed

**How to set:**
```json
{
  "path": "sensors.akat.anchor.chainOutSet",
  "value": 30.0  // Set to 30 meters
}
```

**What happens:**
1. Counter immediately updates to specified value
2. Value is saved to flash memory
3. Confirmation sent back to Signal K
4. Counter continues from new value

### Extending Runtime
- Send the same direction command again while running
- Time is extended without stopping motor
- No maximum limit (runs until explicitly stopped)

### Reversing Direction
1. Send opposite direction command
2. Motor stops immediately
3. **Neutral delay** (400ms default) pause
4. Motor starts in new direction
5. Chain counter automatically switches to counting opposite direction

### Freefall Mode
- Send `"freefall"` command
- Motor lowers anchor for `default_chain_seconds`
- Stops automatically after configured time
- Chain counter tracks deployment
- Useful for quick chain deployment

### External Control
- Connect buttons to configured GPIO pins
- Buttons override Signal K commands when pressed
- Automatic conflict detection prevents simultaneous UP/DOWN

### Virtual Buzzer Alerts
- Alerts trigger when chain crosses configured thresholds
- Beep count increases with chain length
- Events published to Signal K for external handling
- Configurable for deployment, retrieval, or both directions

## Chain Counter Details

### How It Works

1. **Sensor triggers** when magnet passes (LOW signal)
2. **State validation** - waits 150ms for stable state
3. **Double-check** - ignores pulses within 300ms
4. **LOW→HIGH transition** counted as one pulse
5. **Direction matters**:
   - `RUNNING_DOWN`: Adds to chain_out_meters
   - `RUNNING_UP`: Subtracts from chain_out_meters
6. **Advanced debouncing** filters false triggers
7. **Updates sent** to Signal K immediately

### Advanced Debouncing Algorithm

The controller uses a sophisticated debouncing system:

```cpp
// State machine tracking:
- last_sensor_state: Current reading
- sensor_stable_state: Confirmed stable state
- sensor_stable_since: When stability began

// Debounce process:
1. Detect state change
2. Wait 150ms for stability
3. Confirm state hasn't changed during wait
4. Check 300ms minimum between pulses
5. Count only if all checks pass
```

**Why this matters:**
- ❌ Simple debounce (50ms): Counts 2-3 times per pulse
- ✅ Advanced debounce (150ms + validation): Accurate single count

### Sensor Placement Tips

**Good placement:**
- ✅ Magnet passes within 5-10mm of reed switch
- ✅ Switch mounted securely, won't vibrate loose
- ✅ Protected from water spray
- ✅ One trigger per chain link/meter
- ✅ Magnet attached firmly (use epoxy or cable ties)

**Avoid:**
- ❌ Magnet too far (>15mm) - unreliable triggering
- ❌ Multiple magnets in range - double counting
- ❌ Loose mounting - missed counts
- ❌ Direct water exposure - use waterproof reed switch
- ❌ Fast magnet passes - can cause bounce

### Troubleshooting Chain Counter

**Counter incrementing multiple times (+2, +3):**
1. **Bouncing issue** - most common cause
   - Check logs for "pulse ignored (too soon)" messages
   - If you see many, hardware is bouncing
   - Solution: Add 100nF capacitor across switch
2. **Magnet too strong/close**
   - Switch stays triggered too long
   - Solution: Move magnet slightly farther away
3. **Vibration triggering**
   - Boat vibration causing false triggers
   - Solution: Mount switch more securely
4. **Increase debounce time** (if needed):
   ```cpp
   const unsigned long pulse_debounce_ms = 200; // or 250ms
   ```

**Counter not incrementing:**
1. Check wiring: GPIO 25 to one terminal, GND to other
2. Verify magnet proximity: Should be <10mm at closest point
3. Check logs: Should see chain updates
4. Test sensor: Use multimeter in continuity mode

**Counter incrementing in wrong direction:**
1. Verify motor direction matches state (UP should raise, DOWN should lower)
2. Check relay wiring
3. Swap relay connections if motor runs backwards

**Counter drifts over time:**
1. Use `chainOutSet` to correct periodically
2. Recalibrate: Measure known length and adjust calibration factor
3. Check for loose magnet (might miss some triggers)
4. Verify sensor isn't triggering from vibration

**Counter resets unexpectedly:**
1. Check power supply stability
2. Verify flash storage is working (logs will show save errors)
3. Don't send reset commands accidentally
4. Check Signal K for unexpected `chainOutSet` values

## Safety Features Explained

### 1. Neutral Delay (400ms)
**Why it exists**: Prevents mechanical and electrical damage when reversing motor direction.

**How it works**:
- When you change from UP to DOWN (or vice versa)
- Motor stops completely
- Waits 400ms (configurable)
- Then starts in new direction

**Without this**: Motor would try to reverse while still spinning, causing:
- High current spikes
- Motor winding damage
- Gearbox stress
- Shortened equipment lifespan

### 2. Connection Loss Protection
**Why it exists**: Prevents runaway motor if network fails.

**How it works**:
- Monitors Signal K WebSocket connection continuously
- If connection lost while motor running → immediate stop
- Checked in two places:
  - Connection state change handler (immediate)
  - tick() function (watchdog, checks every cycle)

**Scenarios protected**:
- WiFi disconnection
- Signal K server crash
- Network cable unplugged
- Router reboot

**Note**: Chain counter continues to work even if disconnected (counts locally)

### 3. Command Debouncing (250ms)
**Why it exists**: Prevents command flooding during Signal K reconnection.

**How it works**:
- Ignores duplicate commands within 250ms
- Prevents re-entrant command processing
- Connection settling period (2 seconds after reconnect)

**Without this**: Signal K restart would flood ESP32 with cached commands, causing freeze.

### 4. Chain Counter Debouncing (150ms + validation)
**Why it exists**: Prevents double/triple counting from switch bounce.

**How it works**:
- Waits 150ms for sensor state to stabilize
- Validates state hasn't changed during wait
- Requires minimum 300ms between pulses
- Logs "too soon" warnings for debugging

**Without this**: One magnet pass could count 2-3 times.

### 5. External Input Debouncing (50ms configurable)
**Why it exists**: Prevents false triggers from button bounce.

**How it works**:
- Debounces external GPIO inputs
- Configurable time (10-250ms)
- Filters out noise and contact bounce

### 6. Reconnection Handling
**Why it exists**: Automatic recovery from network issues.

**How it works**:
- Watchdog monitors connection every cycle
- If disconnected > 60 seconds → attempts reconnect
- After 8 failed attempts → ESP32 reboot (full reset)

## Technical Specifications

### Timing Parameters
- **Command debounce**: 250ms
- **Neutral delay**: 400ms (configurable)
- **Chain sensor debounce**: 150ms (state validation)
- **Minimum pulse interval**: 300ms (double-check)
- **Connection settling**: 2000ms after reconnect
- **Heartbeat interval**: 2000ms
- **Reconnect timeout**: 60000ms
- **Default runtime**: 3600s (1 hour)
- **External input debounce**: 50ms (configurable)

### Memory Usage
- **Flash**: ~1.1MB (program + libraries)
- **RAM**: ~55KB typical usage
- **Persistent storage**: <2KB (configuration + chain count)

### Network Performance
- **WebSocket messages**: ~250-350 bytes per heartbeat
- **Bandwidth**: <200 bytes/sec average
- **Latency**: <50ms command response (local network)

### GPIO Requirements
- **2 output pins** for relays (configurable)
- **1 input pin** for chain sensor (configurable, with pull-up)
- **2 optional input pins** for external controls (configurable)
- **1 output pin** for LED (built-in)
- All standard ESP32 GPIO pins supported

### Chain Counter Specifications
- **Sensor type**: Reed switch (normally open)
- **Trigger distance**: 5-15mm typical
- **Debounce time**: 150ms + 300ms double-check
- **Resolution**: Configurable (typically 1.0m per pulse)
- **Range**: 0-9999.9 meters (practical limit)
- **Accuracy**: ±1 pulse (depends on calibration and setup)
- **Set range**: 0-9999.9 meters via `chainOutSet`

## Integration Examples

### Node-RED Flow
```javascript
// Read chain length
msg.payload = {
  "context": "vessels.self",
  "subscribe": [{
    "path": "sensors.akat.anchor.chainOut"
  }]
};
return msg;

// Set chain counter to specific value
msg.payload = {
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.chainOutSet",
      "value": 25.0
    }]
  }]
};
return msg;

// Reset chain counter
msg.payload = {
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.resetChainCounter",
      "value": true
    }]
  }]
};
return msg;

// Display in dashboard
if (msg.payload.updates) {
  var chainOut = msg.payload.updates[0].values[0].value;
  msg.payload = "Chain deployed: " + chainOut.toFixed(1) + "m";
}
return msg;
```

### WilhelmSK (iOS/Android)
1. Add instrument: "Anchor Chain"
2. Configure path: `sensors.akat.anchor.chainOut`
3. Set units: meters
4. Display shows real-time chain length
5. Add buttons for `chainOutSet` and `resetChainCounter`

### Grafana Dashboard
```sql
-- Query for chain deployment over time
SELECT
  time,
  value as "Chain Out (m)"
FROM signalk
WHERE path = 'sensors.akat.anchor.chainOut'
AND time > now() - 24h
```

## Development

### Code Structure
```
main.cpp
├── AnchorController class
│   ├── Pin control (setupPins, relay control)
│   ├── Chain counter (updateChainCounter, resetChainCounter)
│   ├── State machine (runDirection, startRun, stopNow)
│   ├── Signal K integration (sendHeartbeat, attachSignalK)
│   ├── External inputs (handleExternalInputs_)
│   ├── Virtual buzzer (checkBuzzerThresholds, fireBuzzer_)
│   ├── Configuration (to_json, from_json, get_config_schema)
│   └── Main loop (tick)
├── setup() - Initialization
└── loop() - Main loop + watchdogs
```

### Adding Features
The code is designed to be extended:
- Add more safety checks in `tick()`
- Add additional Signal K paths in `attachSignalK()`
- Add configuration options in `get_config_schema()`
- Add logging/diagnostics in appropriate methods
- Modify chain counter logic in `updateChainCounter()`

### Debugging Chain Counter
Enable verbose logging:
```cpp
SetupLogging(ESP_LOG_DEBUG);  // In setup()
```

Watch for these log messages:
```
Chain counter initialized: pin=25, pullup=1, cal=1.00m/pulse
Chain OUT: 1.0m (pulse #1)
Chain pulse ignored (too soon: 45ms)  ← Bouncing detected!
Chain OUT: 2.0m (pulse #2)
Chain IN: 1.0m (pulse #1)
Chain counter SET to 25.0m (25 pulses) via SignalK
Chain counter reset to 0
```

### Custom Calibration Logic
If you need complex calibration (e.g., different chain sections):
```cpp
// In updateChainCounter(), modify:
if (state == RUNNING_DOWN) {
  // Custom calibration based on current depth
  float cal = (chain_out_meters < 10.0f) ? 1.0f : 0.95f;
  chain_out_meters += cal;
}
```

## License

This project is provided as-is for marine use. Use at your own risk. Always ensure proper safety measures when operating anchor windlass equipment.

## Credits

Built with:
- **SensESP** - Signal K sensor development framework
- **Signal K** - Marine data standard
- **ESP32** - Espressif microcontroller platform

## Support

For issues, questions, or contributions, please refer to the project repository.

## Changelog

### Version 2.1 (Current)
- ✨ Added `chainOutSet` path - set counter to any value
- ✨ Added external GPIO control inputs
- ✨ Added virtual buzzer alerts with configurable thresholds
- 🔧 Improved debouncing algorithm (150ms + validation)
- 🔧 Added double-check for pulses (300ms minimum)
- 📝 Enhanced logging with "too soon" warnings
- 🐛 Fixed double/triple counting issue
- 📊 Counter value now saved after manual set

### Version 2.0
- ✨ Added chain counter with magnetic reed switch
- ✨ Real-time chain length measurement
- ✨ Persistent chain count storage
- ✨ Calibration support for different chain types
- ✨ Chain counter reset via Signal K
- 🔧 Improved configuration UI
- 📊 New Signal K paths for chain data

### Version 1.0
- ⚡ Initial release
- ⚡ Dual-relay motor control
- ⚡ Signal K integration
- ⚡ Safety features (neutral delay, connection monitoring)
- ⚡ Web configuration interface

---

**⚠️ Safety Warning**: This controller operates powerful marine equipment. Always:
- Ensure proper electrical installation by qualified personnel
- Test thoroughly before relying on remote operation
- Maintain manual override capability
- Follow manufacturer's guidelines for your windlass
- Never leave boat unattended while windlass is operating
- Keep clear of moving chain and anchor while operating
- Verify chain counter accuracy regularly
- Use chain counter as reference only, not primary safety measurement
- Manually verify chain length periodically
- Account for chain stretch and measurement drift over time

**⚠️ Chain Counter Disclaimer**: The chain counter is a convenience feature and should not be relied upon as the sole means of determining anchor rode length. Always:
- Visually verify chain deployment when possible
- Use depth sounder to confirm water depth
- Account for scope requirements (chain length to depth ratio)
- Mark your chain physically with paint or markers
- Periodically verify counter accuracy against physical markers
- Recalibrate after maintenance or chain replacement

**Last Updated**: 2025-10-30