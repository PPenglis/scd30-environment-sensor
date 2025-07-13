# SCD30 Environmental Monitor

A comprehensive IoT environmental monitoring device built with ESP32 and SCD30 sensor for real-time CO2, temperature, and humidity monitoring with Home Assistant integration.

## Features

- **Real-time Environmental Monitoring**: CO2, temperature, and humidity sensing using SCD30 sensor
- **Visual Display**: Live readings on integrated TFT display with color-coded alerts
- **WiFi Connectivity**: Automatic connection with fallback captive portal configuration
- **MQTT Integration**: Publishes data to MQTT broker with Home Assistant auto-discovery
- **Battery Monitoring**: Smart battery level monitoring with low-power sleep mode
- **Web Configuration**: Easy setup via captive portal web interface
- **Visual Alerts**: Color-coded warnings for CO2 levels (blue warning, red critical)

## Hardware Requirements

- **ESP32**: TTGO T-Display (with integrated TFT display)
- **SCD30 Sensor**: CO2, temperature, and humidity sensor
- **Battery**: For portable operation (optional)
- **Connections**: I2C interface for sensor communication

### Pin Configuration

- **Battery Monitor**: Pin 34 (ADC)
- **Display**: Integrated on TTGO T-Display
- **SCD30 Sensor**: I2C (SDA/SCL default pins)

## Software Dependencies

### Libraries Required

Install these libraries through Arduino IDE Library Manager:

```
TFT_eSPI [2.5.43]
SensirionI2cScd30 [0.1.0]
PubSubClient [2.8]
ESPAsyncWebServer [3.6.2]
DNSServer 
LittleFS 
ArduinoJson [0.2.0]
AsyncTCP [3.3.2]
```

### ESP32 Board Package

**Important**: Use ESP32 Arduino Core **version 3.0** (not 3.1) to avoid compatibility issues.

## Configuration

### Initial Setup

1. Flash the code to your ESP32 TTGO T-Display
2. On first boot, the device will create a WiFi access point named "SCD30-Setup"
3. Connect to this network and navigate to any website (captive portal will redirect)
4. Configure WiFi and MQTT settings through the web interface

### Web Configuration Interface

The captive portal provides fields for:

#### WiFi Settings
- **SSID**: Your WiFi network name
- **Password**: WiFi password

#### MQTT Settings
- **Server**: MQTT broker address (IP or hostname)
- **Port**: MQTT port (default: 1883)
- **Username**: MQTT username (Not working)
- **Password**: MQTT password (Not working)
- **Topic Prefix**: Base topic for sensor data (e.g., "home/sensors/scd30")

## Home Assistant Integration

The device automatically publishes Home Assistant discovery messages for:

- **CO2 Level** (ppm)
- **Temperature** (°C)
- **Humidity** (%)
- **Battery Voltage** (V)

Topics published:
- `home/sensor/co2`
- `home/sensor/temperature`
- `home/sensor/humidity`
- `home/sensor/batteryVoltage`

## Display Features

### Real-time Display Shows:
- CO2 concentration (ppm)
- Temperature (°C)
- Humidity (%)
- Battery voltage

### Visual Alerts:
- **Normal**: Green text
- **Warning** (CO2 > 1000 ppm): Blue Display Border
- **Critical** (CO2 > 1500 ppm): Red/Blue Display Border

## Power Management

### Battery Monitoring
- Continuous voltage monitoring with smoothing algorithm
- Automatic deep sleep when battery drops below 3.5V
- Low-power WiFi transmission settings

### Power Optimization
- Reduced WiFi transmission power
- Smart sleep modes for battery conservation
- Efficient sensor reading intervals

## Technical Specifications

### Sensor Thresholds
- **CO2 Warning**: 1000 ppm
- **CO2 Critical**: 1500 ppm
- **Low Battery**: 3.5V

### Timing Configuration
- **Sensor Warmup**: readings skipped on startup
- **MQTT Publish**: Every 5 seconds
- **Display Update**: 1 second intervals
- **WiFi Timeout**: 20 seconds

### Memory Management
- Uses LittleFS for configuration storage
- JSON-based configuration files

## Installation

1. **Hardware Setup**:
   - Connect SCD30 sensor to ESP32 via I2C
   - Ensure proper power connections
   - Optional: Connect battery for portable operation

2. **Software Installation**:
   - Install Arduino IDE
   - Add ESP32 board package (version 3.0)
   - Install required libraries
   - Upload the code to your device

3. **Configuration**:
   - Connect to "SCD30-Setup" WiFi network
   - Browse to 192.168.4.1
   - Configure WiFi and MQTT settings
   - Device will restart and begin normal operation

## Troubleshooting

### Common Issues

**WiFi Connection Problems**:
- Check SSID and password in configuration
- Ensure WiFi network is 2.4GHz
- Device will restart and enter setup mode if connection fails

**MQTT Connection Issues**:
- Verify MQTT broker address and port
- Check username/password if authentication is required
- Monitor serial output for connection status

**Sensor Reading Errors**:
- Ensure I2C connections are secure
- Check sensor power supply
- Monitor serial output for error messages
- Recalibrate SCD30

## Serial Monitor Output

The device provides detailed logging via Serial (115200 baud):
- System initialization status
- WiFi connection attempts
- MQTT publishing confirmation
- Sensor readings and errors
- Battery voltage monitoring

## Author

**Peter Penglis**  
Date: July 13, 2025