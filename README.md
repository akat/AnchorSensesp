# SensESP Anchor Windlass Controller

A Signal K-based anchor windlass controller for ESP32 that provides safe, remote operation of electric anchor windlass motors through relay control.

## Overview

This controller integrates with your boat's Signal K network to provide remote control of your anchor windlass. It uses two relays to control motor direction (UP/DOWN) and includes multiple safety features to protect the motor and prevent damage.

## Features

### Core Functionality
- **Dual-relay motor control** - Independent UP and DOWN relay channels
- **Signal K integration** - Full integration with marine data network
- **Remote operation** - Control from any Signal K-compatible app
- **Web configuration** - Easy setup through SensESP web interface
- **Persistent settings** - Configuration saved to ESP32 flash memory

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
│  GND ───────┼──→ Common Ground ←── Motor Ground
│             │
│  5V ────────┼──→ ESP32 Power
└─────────────┘

Motor Power Supply (12V/24V)
     │
     └──→ Relay COM terminals
```

### Default GPIO Pins
- **Relay UP**: GPIO 26
- **Relay DOWN**: GPIO 27
- **LED indicator**: Built-in LED (GPIO 2 on most boards)

*Note: All pins are configurable through the web interface*

## Installation

### 1. Hardware Setup
1. Connect ESP32 to relay module
2. Wire relays to windlass motor
3. Ensure proper power supply and grounding
4. **Test relays manually before connecting motor**

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
| Relay UP GPIO | GPIO pin for UP relay | 26 | Configurable |
| Relay DOWN GPIO | GPIO pin for DOWN relay | 27 | Configurable |
| Relays Active HIGH | Relay trigger logic | true | Set false for active-low relays |
| Enabled | Enable/disable controller | true | Safety disable |
| Default Seconds | Duration for "freefall" command | 5.0 | In seconds |
| Neutral Delay | Pause when reversing direction | 400 | In milliseconds |

## Signal K Integration

### Published Paths

The controller publishes to Signal K:

| Path | Type | Description | Update Rate |
|------|------|-------------|-------------|
| `sensors.akat.anchor.enabled` | boolean | Controller enabled status | On connect |
| `sensors.akat.anchor.lastUpdate` | timestamp | Last heartbeat timestamp | Every 2 seconds |
| `sensors.akat.anchor.lastCommand` | string | Current operation | While running |

### Subscribed Paths (Commands)

Send commands by setting these Signal K values:

| Path | Value | Action |
|------|-------|--------|
| `sensors.akat.anchor.state` | `"running_up"` | Raise anchor (runs until stopped) |
| `sensors.akat.anchor.state` | `"running_down"` | Lower anchor (runs until stopped) |
| `sensors.akat.anchor.state` | `"freefall"` | Quick drop for configured seconds |
| `sensors.akat.anchor.state` | `"idle"` | Stop motor immediately |
| `sensors.akat.anchor.defaultChainSeconds` | number | Update default freefall duration |

### Example: Signal K Delta

To raise the anchor, send this delta to Signal K:

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

## Operation

### Starting Motor
1. Send `"running_up"` or `"running_down"` command
2. Motor runs for 1 hour (or until stopped)
3. LED blinks once per second while running
4. Heartbeat updates sent every 2 seconds

### Stopping Motor
1. Send `"idle"` command, OR
2. Connection to Signal K is lost (automatic safety stop)

### Extending Runtime
- Send the same direction command again while running
- Time is extended without stopping motor
- No maximum limit (runs until explicitly stopped)

### Reversing Direction
1. Send opposite direction command
2. Motor stops immediately
3. **Neutral delay** (400ms default) pause
4. Motor starts in new direction

### Freefall Mode
- Send `"freefall"` command
- Motor lowers anchor for `default_chain_seconds`
- Stops automatically after configured time
- Useful for quick chain deployment

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
- **Connection settling**: 2000ms after reconnect
- **Heartbeat interval**: 2000ms
- **Reconnect timeout**: 60000ms
- **Default runtime**: 3600s (1 hour)

### Memory Usage
- **Flash**: ~1MB (program + libraries)
- **RAM**: ~50KB typical usage
- **Persistent storage**: <1KB (configuration file)

### Network Performance
- **WebSocket messages**: ~200 bytes per heartbeat
- **Bandwidth**: <100 bytes/sec average
- **Latency**: <50ms command response (local network)

### GPIO Requirements
- **2 output pins** for relays (configurable)
- **1 output pin** for LED (built-in)
- All standard ESP32 GPIO pins supported

## Development

### Code Structure
```
main.cpp
├── AnchorController class
│   ├── Pin control (setupPins, relayUpOn, relayDownOn, relaysOff)
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

### Debugging
Enable verbose logging:
```cpp
SetupLogging(ESP_LOG_DEBUG);  // In setup()
```

Monitor serial output at 115200 baud.

## License

This project is provided as-is for marine use. Use at your own risk. Always ensure proper safety measures when operating anchor windlass equipment.

## Credits

Built with:
- **SensESP** - Signal K sensor development framework
- **Signal K** - Marine data standard
- **ESP32** - Espressif microcontroller platform

## Support

For issues, questions, or contributions, please refer to the project repository.

---

**⚠️ Safety Warning**: This controller operates powerful marine equipment. Always:
- Ensure proper electrical installation by qualified personnel
- Test thoroughly before relying on remote operation
- Maintain manual override capability
- Follow manufacturer's guidelines for your windlass
- Never leave boat unattended while windlass is operating
- Keep clear of moving chain and anchor while operating

**Last Updated**: 2025-10-29