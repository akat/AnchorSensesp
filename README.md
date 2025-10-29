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
- **Calibration support** - Adjustable meters per pulse for different chain types
- **Debounce protection** - Filters false triggers from sensor bounce
- **Persistent storage** - Remembers chain length after power loss
- **Reset capability** - Manual reset when anchor is fully retrieved

### Safety Features
- **Neutral delay** - Mandatory pause when reversing direction to prevent mechanical damage
- **Connection monitoring** - Automatic motor stop if Signal K connection is lost
- **Command debouncing** - Prevents command flooding during network reconnections
- **Re-entrancy protection** - Prevents conflicting simultaneous commands
- **Visual feedback** - Onboard LED blinks when motor is running

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

### Windlass Motor
- DC motor with UP and DOWN wiring
- Common ground configuration
- Motor current within relay specifications

### Power Supply
- 5V power for ESP32 (USB or dedicated supply)
- Separate power supply for windlass motor (12V/24V typical)
- **Important**: ESP32 and motor should share common ground

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
```

### Default GPIO Pins
- **Relay UP**: GPIO 26
- **Relay DOWN**: GPIO 27
- **Chain Sensor**: GPIO 25 (with internal pull-up)
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
5. **Test sensor**: Watch serial output while manually turning windlass

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

### Subscribed Paths (Commands)

Send commands by setting these Signal K values:

| Path | Value | Action |
|------|-------|--------|
| `sensors.akat.anchor.state` | `"running_up"` | Raise anchor (runs until stopped) |
| `sensors.akat.anchor.state` | `"running_down"` | Lower anchor (runs until stopped) |
| `sensors.akat.anchor.state` | `"freefall"` | Quick drop for configured seconds |
| `sensors.akat.anchor.state` | `"idle"` | Stop motor immediately |
| `sensors.akat.anchor.state` | `"reset_counter"` | Reset chain counter to zero |
| `sensors.akat.anchor.defaultChainSeconds` | number | Update default freefall duration |
| `sensors.akat.anchor.resetChainCounter` | boolean | Reset counter (send `true`) |

### Example: Signal K Deltas

**Raise anchor:**
```json
{
  "context": "vessels.self",
  "updates": [{
    "values": [{
      "path": "sensors.akat.anchor.state",
      "value": "running_up"
    }]
  }]
}
```

**Reset chain counter:**
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

### Resetting Chain Counter
**When to reset:**
- When anchor is fully retrieved (all chain on windlass)
- After manual chain adjustment
- When starting fresh measurement

**How to reset:**
1. Ensure anchor is fully retrieved
2. Send reset command via Signal K
3. Counter resets to 0.0 meters
4. New measurements start from zero

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

## Chain Counter Details

### How It Works

1. **Sensor triggers** when magnet passes (LOW signal)
2. **Sensor releases** when magnet moves away (HIGH signal)
3. **LOW→HIGH transition** counted as one pulse
4. **Direction matters**:
   - `RUNNING_DOWN`: Adds to chain_out_meters
   - `RUNNING_UP`: Subtracts from chain_out_meters
5. **Debouncing** filters pulses closer than 50ms
6. **Updates sent** to Signal K immediately

### Sensor Placement Tips

**Good placement:**
- ✅ Magnet passes within 5-10mm of reed switch
- ✅ Switch mounted securely, won't vibrate loose
- ✅ Protected from water spray
- ✅ One trigger per chain link/meter

**Avoid:**
- ❌ Magnet too far (>15mm) - unreliable triggering
- ❌ Multiple magnets in range - double counting
- ❌ Loose mounting - missed counts
- ❌ Direct water exposure - use waterproof reed switch

### Troubleshooting Chain Counter

**Counter not incrementing:**
1. Check wiring: GPIO 25 to one terminal, GND to other
2. Verify magnet proximity: Should be <10mm at closest point
3. Check logs: Should see "Chain OUT" or "Chain IN" messages
4. Test sensor: Use multimeter in continuity mode

**Counter incrementing too fast:**
1. Check for multiple magnets in range
2. Verify debounce is working (50ms minimum between pulses)
3. Reduce sensor sensitivity if adjustable

**Counter incrementing in wrong direction:**
1. Verify motor direction matches state (UP should raise, DOWN should lower)
2. Check relay wiring
3. Swap relay connections if motor runs backwards

**Counter drifts over time:**
1. Recalibrate: Measure known length and adjust calibration factor
2. Check for loose magnet (might miss some triggers)
3. Verify sensor isn't triggering from vibration

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

### 4. Reconnection Handling
**Why it exists**: Automatic recovery from network issues.

**How it works**:
- Watchdog monitors connection every cycle
- If disconnected > 60 seconds → attempts reconnect
- After 8 failed attempts → ESP32 reboot (full reset)

## Troubleshooting

### Motor Won't Start
1. **Check Signal K connection**
   - LED should not be blinking if idle
   - Check logs: `sensors.akat.anchor.lastUpdate` should be recent
   
2. **Check enabled status**
   - Verify `enabled = true` in configuration
   - Send command with enabled check: see `sensors.akat.anchor.enabled`

3. **Check relay wiring**
   - Verify GPIO pins in configuration match physical wiring
   - Test relay trigger logic (active HIGH vs LOW)
   - Use multimeter to verify relay switching

4. **Check command debouncing**
   - Wait 2 seconds after connection before sending commands
   - Avoid rapid repeated commands

### Motor Won't Stop
1. **Send idle command**
   ```json
   {"path": "sensors.akat.anchor.state", "value": "idle"}
   ```

2. **Disconnect Signal K** - Motor will stop automatically (safety feature)

3. **Power cycle ESP32** - Last resort

### Chain Counter Issues

**Not counting:**
1. Check sensor wiring (GPIO 25 and GND)
2. Verify magnet passes close enough (<10mm)
3. Enable debug logging to see sensor state changes
4. Test sensor with multimeter (should show open/closed)

**Counting backwards:**
1. Verify motor direction (check relay connections)
2. Ensure UP command raises anchor, DOWN lowers it

**Inaccurate count:**
1. Recalibrate using known chain length
2. Check for missed triggers (magnet too far)
3. Verify debounce isn't filtering valid pulses

**Counter resets unexpectedly:**
1. Check power supply stability
2. Verify flash storage is working (logs will show save errors)
3. Don't send reset commands accidentally

### Connection Issues
1. **Check WiFi signal strength**
   - Logs show RSSI every 60 seconds
   - Relocate ESP32 if signal weak

2. **Check Signal K server**
   - Verify server running and accessible
   - Check server logs for connection attempts

3. **Check logs**
   ```
   SK WS: Connected          ← Good
   SK WS: Disconnected       ← Check network
   WiFi disconnected         ← WiFi problem
   Chain OUT: 15.5m          ← Counter working
   ```

### ESP32 Keeps Rebooting
1. **Check power supply** - Needs stable 5V, 500mA minimum
2. **Check relay current draw** - May need separate relay power
3. **Check logs before reboot** - Look for watchdog triggers

### Motor Stops Unexpectedly
1. **Connection loss** - Check for WiFi/network stability
2. **Timeout reached** - Motor runs for 1 hour default, then stops
3. **Check logs** for reason:
   ```
   up:done              ← Completed normally
   safety:disconnected  ← Connection lost
   idle:remote          ← Stopped by command
   ```

## Technical Specifications

### Timing Parameters
- **Command debounce**: 250ms
- **Neutral delay**: 400ms (configurable)
- **Chain sensor debounce**: 50ms
- **Connection settling**: 2000ms after reconnect
- **Heartbeat interval**: 2000ms
- **Reconnect timeout**: 60000ms
- **Default runtime**: 3600s (1 hour)

### Memory Usage
- **Flash**: ~1MB (program + libraries)
- **RAM**: ~50KB typical usage
- **Persistent storage**: <2KB (configuration + chain count)

### Network Performance
- **WebSocket messages**: ~200-300 bytes per heartbeat
- **Bandwidth**: <150 bytes/sec average
- **Latency**: <50ms command response (local network)

### GPIO Requirements
- **2 output pins** for relays (configurable)
- **1 input pin** for chain sensor (configurable, with pull-up)
- **1 output pin** for LED (built-in)
- All standard ESP32 GPIO pins supported

### Chain Counter Specifications
- **Sensor type**: Reed switch (normally open)
- **Trigger distance**: 5-15mm typical
- **Debounce time**: 50ms
- **Resolution**: Configurable (typically 1.0m per pulse)
- **Range**: 0-9999.9 meters (practical limit)
- **Accuracy**: ±1 pulse (depends on calibration)

## Use Cases & Examples

### Scenario 1: Anchoring in 20m Depth
```
1. Position boat over anchorage
2. Send: "running_down" command
3. Watch Signal K display: chainOut increases
4. At 20m deployed: Send "idle" command
5. Set anchor, verify holding
6. Note: "23m deployed" (20m depth + 3m scope)
```

### Scenario 2: Retrieving Anchor
```
1. Motor up slowly: Send "running_up"
2. Watch chain counter decrease: 23m → 20m → 15m...
3. At 0m: Anchor at bow roller
4. Send "idle" command
5. Chain counter shows 0.0m (or close to it)
6. Send "reset_counter" to zero any drift
```

### Scenario 3: Let Out More Scope
```
Current: 25m deployed, need 35m
1. Send "running_down"
2. Watch: 25m → 30m → 35m
3. Send "idle"
Result: 35m deployed, 10m added
```

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
Chain OUT: 2.0m (pulse #2)
Chain IN: 1.0m (pulse #1)
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

### Version 2.0 (Current)
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

**Last Updated**: 2025-10-29