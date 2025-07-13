/* SCD30 Environmental Monitor
 * IoT device for monitoring CO2, temperature, and humidity
 * Features: WiFI, MQTT publishing, captive portal initiation, TFT Display
 * 
 * Author: Peter Penglis
 * Date: 12/07/2025
 * Hardware: ESP32 + SCD30 + Display(TTGO T-Display) 
 */

#include <TFT_eSPI.h>
#include <Free_Fonts.h>
#include <Arduino.h>
#include <SensirionI2cScd30.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_netif.h>
#include <lwip/dns.h>
using namespace fs;


// *********************************************************************
// Configuration
// *********************************************************************


//Hardware
#define SCREEN_WIDTH 135
#define SCREEN_HEIGHT 240
#define PIN_BATTERY 34
#define LOW_BATTERY_THRESHOLD 3.5
#define BATTERY_VOLTAGE_DIVIDER 2.0
#define BATTERY_CALIBRATION_FACTOR (1100.0 / 1000.0)
#define MAX_ADC 4095.0
#define REF_VOLTAGE 3.3

//Network
#define CONFIG_FILE "/wifi.json"
#define MQTT_PORT 1883
#define SETUP_AP_SSID "SCD30-Setup"

// SCD30 level thresholds
#define CO2_WARNING_THRESHOLD 1000
#define CO2_CRITICAL_THRESHOLD 1500

//Adjustments
#define DNS_INTERVAL 30
#define SENSOR_WARMMING 10
#define WIFI_CONNECT_TIMEOUT 20
#define MQTT_CONNECT_RETRIES 3
#define MQTT_PUBLISH_INTERVAL 5
#define BATTERY_SMOOTH 0.125
#define BATTERY_ALL 1
#define BATTERY_ADJ_LIMIT 0.2



// *********************************************************************
// Globals
// *********************************************************************

// Global variables - hardware
DNSServer dns;
AsyncWebServer server(80);
SensirionI2cScd30 sensor;
TFT_eSPI tft = TFT_eSPI();
WiFiClient espClient;
PubSubClient client(espClient);

// System Status - State Management
struct SystemState {
  bool portalMode = false;
  bool portalDns = false;
  bool initialized = false;
  float rollingBatteryVoltage = 0.0;
  int warmupReadings = SENSOR_WARMMING;
  int publishState = 0;
  int displayState = 0;
  char errorBuffer[128];
  int16_t lastError = 0;
};

// Configuration container
struct DeviceConfiguration {
  String SSID;
  String Password;
  String MQTTServer;
  int MQTTPort;
  String MQTTUser;
  String MQTTPass;
  String MQTTTopic;
  DeviceConfiguration()
    : MQTTPort(MQTT_PORT) {}
};

SystemState sysState;
DeviceConfiguration devConfig;

// *********************************************************************
// Battery Functions
// *********************************************************************

/*
   Using a rolling average for voltage
   Determines if we want to sleep to avoid killing the battery
   Normal ESP should last a year on sleep for 500mAh
   You should alert if its been more than a week.
   Includes smoothing function because when we trasmit we get a sudden drop
   So we drop out sudden lows by using a smaller factor.
 */

/* 
 *  Convert ADC reading to levels
 */
float calcBatteryVoltage(int rawValue) {
  return ((float)rawValue / MAX_ADC) * BATTERY_VOLTAGE_DIVIDER * REF_VOLTAGE * BATTERY_CALIBRATION_FACTOR;
}

// Loop Function
void updateBatteryVoltage() {
  int rawBattery = analogRead(PIN_BATTERY);
  float batteryVoltage = calcBatteryVoltage(rawBattery);
  float lerp_percent = abs(batteryVoltage - sysState.rollingBatteryVoltage) > BATTERY_ADJ_LIMIT ? BATTERY_ALL : BATTERY_SMOOTH;
  sysState.rollingBatteryVoltage = sysState.rollingBatteryVoltage * (1 - lerp_percent) + batteryVoltage * lerp_percent;
}

bool isBatteryTooLow() {
  return sysState.rollingBatteryVoltage < LOW_BATTERY_THRESHOLD;
}
// *********************************************************************
// Configuration
// *********************************************************************

// Load WiFi credentials from LittleFS
bool loadCredentials() {
  if (!LittleFS.exists(CONFIG_FILE)) {
    Serial.println("Config file does not exist");
    return false;
  }

  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) {
    Serial.println("Failed to open config file");
    return false;
  }

  StaticJsonDocument<512> config;
  DeserializationError err = deserializeJson(config, file);
  file.close();

  if (err) {
    Serial.println("Failed to parse config file");
    return false;
  }

  devConfig.SSID = config["wifi"]["ssid"].as<String>();
  devConfig.Password = config["wifi"]["password"].as<String>();
  devConfig.MQTTServer = config["mqtt"]["server"].as<String>();
  devConfig.MQTTPort = config["mqtt"]["port"] | MQTT_PORT;
  devConfig.MQTTUser = config["mqtt"]["user"].as<String>();
  devConfig.MQTTPass = config["mqtt"]["pass"].as<String>();
  devConfig.MQTTTopic = config["mqtt"]["topic"].as<String>();
  Serial.println("WiFi SSID: " + devConfig.SSID);
  Serial.println("WiFi Password: [HIDDEN]");
  Serial.println("MQTT Server: " + devConfig.MQTTServer);
  Serial.println("MQTT Port: " + String(devConfig.MQTTPort));
  Serial.println("MQTT User: " + devConfig.MQTTUser);
  Serial.println("MQTT Pass: [HIDDEN]");
  Serial.println("MQTT Topic: " + devConfig.MQTTTopic);

  return !devConfig.SSID.isEmpty();
}

// Save WiFi credentials to LittleFS
bool saveDeviceConfig() {
  StaticJsonDocument<512> config;
  config["wifi"]["ssid"] = devConfig.SSID;
  config["wifi"]["password"] = devConfig.Password;
  config["mqtt"]["server"] = devConfig.MQTTServer;
  config["mqtt"]["port"] = devConfig.MQTTPort;
  config["mqtt"]["user"] = devConfig.MQTTUser;
  config["mqtt"]["pass"] = devConfig.MQTTPass;
  config["mqtt"]["topic"] = devConfig.MQTTTopic;

  File configFile = LittleFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    Serial.println("Failed to create configuration file");
    return false;
  }

  serializeJson(config, configFile);
  configFile.close();

  Serial.println("Device configuration saved successfully");
  return true;
}
// *********************************************************************
// Networking
// *********************************************************************

// Try to connect to WiFi with saved credentials
bool tryConnectWiFi() {
  if (!loadCredentials()) {
    Serial.print("no credentials found");
    return false;
  }

  Serial.println("Connecting to WiFi: " + devConfig.SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(devConfig.SSID.c_str(), devConfig.Password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected successfully\nIP address: %s\n", WiFi.localIP().toString().c_str());
    return true;
  } else {
    Serial.println("WiFi connection failed");
    return false;
  }
}


void dnsTask(void *param) {
  delay(100);      // Optional: wait for netif to be ready
  setupDNS_sub();  // Safe here
  vTaskDelete(NULL);
}

void setupDNS() {
  xTaskCreatePinnedToCore(dnsTask, "dnsTask", 4096, NULL, 1, NULL, 0);  // core 0
}

// Instead of dns.start(), use:
void setupDNS_sub() {
  esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (ap_netif == nullptr) {
    Serial.println("Failed to get AP interface");
    return;
  }
  // Set up DNS server manually
  esp_netif_dns_info_t dns_info;
  IP_ADDR4(&dns_info.ip, 192, 168, 4, 1);  // Use IP_ADDR4 not IP4_ADDR
  esp_netif_dhcps_stop(ap_netif);          // Correct usage with esp_netif_t*
  esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info);
  esp_netif_dhcps_start(ap_netif);
  vTaskDelete(NULL);
}

// Function prototypes
bool loadCredentials();
bool tryConnectWiFi();
void startCaptivePortal();
void connectMQTT();
void publishSensorConfig(String type, String stateTopic, String unit, String idSuffix, String deviceName = "SCD30 Sensor");


void startWebServerTask(void *param) {
  delay(300);  // Wait for AP + TCPIP to be fully ready
  Serial.println("Captive portal starting");
  esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (ap_netif) {
    Serial.println("AP netif acquired.");
  }
  server.begin();
  Serial.println("Captive portal started");
  vTaskDelete(NULL);
}





// Start captive portal for WiFi configuration
void startCaptivePortal() {
  Serial.println("Starting captive portal...");

  // Stop any existing WiFi connection
  WiFi.disconnect();
  delay(100);

  // Set up Access Point
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP("SCD30-Setup");
  delay(2000);

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(apIP);
  delay(1000);

  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  if (ESP.getFreeHeap() < 10000) {  // Less than 10KB free
    Serial.println("Low memory, may cause TCP allocation issues");
  }

  // Simple web interface
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>SCD30 WiFi Setup</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; margin: 20px; }";
    html += "form { max-width: 400px; }";
    html += "input[type='text'], input[type='password'], input[type='number'] { width: 100%; padding: 8px; margin: 5px 0 15px 0; border: 1px solid #ddd; border-radius: 4px; }";
    html += "input[type='submit'] { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; }";
    html += "input[type='submit']:hover { background-color: #45a049; }";
    html += "label { display: block; margin-top: 10px; }";
    html += "h2 { color: #333; }";
    html += ".section { margin-bottom: 20px; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }";
    html += ".section h3 { margin-top: 0; color: #555; }";
    html += "</style>";
    html += "</head><body>";
    html += "<h2>SCD30 Configuration</h2>";
    html += "<form method='POST' action='/save'>";

    // WiFi Section
    html += "<div class='section'>";
    html += "<h3>WiFi Settings</h3>";
    html += "<label for='ssid'>SSID:</label>";
    html += "<input type='text' id='ssid' name='ssid' required>";
    html += "<label for='pass'>Password:</label>";
    html += "<input type='password' id='pass' name='pass'>";
    html += "</div>";

    // MQTT Section
    html += "<div class='section'>";
    html += "<h3>MQTT Settings</h3>";
    html += "<label for='mqtt_server'>MQTT Server:</label>";
    html += "<input type='text' id='mqtt_server' name='mqtt_server' placeholder='e.g., 192.168.1.100 or broker.hivemq.com' required>";
    html += "<label for='mqtt_port'>MQTT Port:</label>";
    html += "<input type='number' id='mqtt_port' name='mqtt_port' value='1883' min='1' max='65535' required>";
    html += "<label for='mqtt_user'>MQTT Username:</label>";
    html += "<input type='text' id='mqtt_user' name='mqtt_user' placeholder='Optional'>";
    html += "<label for='mqtt_pass'>MQTT Password:</label>";
    html += "<input type='password' id='mqtt_pass' name='mqtt_pass' placeholder='Optional'>";
    html += "<label for='mqtt_topic'>Topic Prefix:</label>";
    html += "<input type='text' id='mqtt_topic' name='mqtt_topic' placeholder='e.g., home/sensors/scd30' required>";
    html += "</div>";

    html += "<input type='submit' value='Save & Restart'>";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    String ssid = "";
    String pass = "";
    String mqtt_server = "";
    String mqtt_port = "";
    String mqtt_user = "";
    String mqtt_pass = "";
    String mqtt_topic = "";

    // Get WiFi parameters
    if (request->hasParam("ssid", true)) {
      ssid = request->getParam("ssid", true)->value();
    }
    if (request->hasParam("pass", true)) {
      pass = request->getParam("pass", true)->value();
    }

    // Get MQTT parameters
    if (request->hasParam("mqtt_server", true)) {
      mqtt_server = request->getParam("mqtt_server", true)->value();
    }
    if (request->hasParam("mqtt_port", true)) {
      mqtt_port = request->getParam("mqtt_port", true)->value();
    }
    if (request->hasParam("mqtt_user", true)) {
      mqtt_user = request->getParam("mqtt_user", true)->value();
    }
    if (request->hasParam("mqtt_pass", true)) {
      mqtt_pass = request->getParam("mqtt_pass", true)->value();
    }
    if (request->hasParam("mqtt_topic", true)) {
      mqtt_topic = request->getParam("mqtt_topic", true)->value();
    }

    devConfig.MQTTPort = mqtt_port.toInt();
    devConfig.MQTTServer = mqtt_server;
    devConfig.SSID = ssid;
    devConfig.Password = pass;
    devConfig.MQTTUser = mqtt_user;
    devConfig.MQTTPass = mqtt_pass;
    devConfig.MQTTTopic = mqtt_topic;


    if (!ssid.isEmpty()) {
      // Send confirmation page
      String html = "<!DOCTYPE html><html><body>";
      html += "<h2>Configuration Saved!</h2>";
      html += "<p>WiFi SSID: " + ssid + "</p>";
      html += "<p>MQTT Server: " + mqtt_server + ":" + mqtt_port + "</p>";
      html += "<p>MQTT Topic: " + mqtt_topic + "</p>";
      html += "<p>Device will restart in 3 seconds...</p>";
      html += "<script>setTimeout(function(){ window.location.href='/'; }, 3000);</script>";
      html += "</body></html>";
      saveDeviceConfig();
      delay(2000);
      ESP.restart();
    } else {
      request->send(400, "text/html", "<html><body><h2>Error: Please enter SSID</h2></body></html>");
    }
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/");
  });
  Serial.println("Captive portal starting");
  xTaskCreatePinnedToCore(startWebServerTask, "StartWeb", 4096, NULL, 1, NULL, 0);
  Serial.println("Captive portal started");
}

/******************************************************
   Read from sensor
  
  */
bool readSensorData(float &co2concentration, float &temperature, float &humidity) {
  sysState.lastError = sensor.blockingReadMeasurementData(co2concentration, temperature, humidity);
  if (sysState.lastError == NO_ERROR) return true;
  errorToString(sysState.lastError, sysState.errorBuffer, sizeof(sysState.errorBuffer));
  Serial.printf("Error reading sensor data: %s\n", sysState.errorBuffer);
  return false;
}

// Connect to MQTT broker
void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT");
    return;
  }

  client.setServer(devConfig.MQTTServer.c_str(), devConfig.MQTTPort);

  int attempts = 0;
  while (!client.connected() && attempts < MQTT_CONNECT_RETRIES) {
    Serial.print("Connecting to MQTT...");
    String clientId = "SCD30_" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(" Connected!");
      break;
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
      attempts++;
    }
  }
}

// Publish sensor configuration for Home Assistant
void publishSensorConfig(String type, String stateTopic, String unit, String idSuffix, String deviceName) {
  if (!client.connected()) return;

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String baseId = "scd30_" + mac;
  String uniqueId = baseId + "_" + idSuffix;
  String configTopic = "homeassistant/sensor/" + baseId + "/" + idSuffix + "/config";

  StaticJsonDocument<512> doc;
  doc["name"] = deviceName + " " + type;
  doc["state_topic"] = stateTopic;
  doc["unit_of_measurement"] = unit;
  doc["value_template"] = "{{ value }}";
  doc["unique_id"] = uniqueId;

  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = baseId;
  device["name"] = deviceName + " " + mac;
  device["manufacturer"] = "Peter Penglis";
  device["model"] = "SCD30";
  device["sw_version"] = "1.0";

  String payload;
  serializeJson(doc, payload);

  client.publish(configTopic.c_str(), payload.c_str(), true);
}

// ***************************************************

void showWarning(float co2) {
  // Visual alerts for CO2 levels
  if (co2 > CO2_WARNING_THRESHOLD && co2 < CO2_CRITICAL_THRESHOLD) {
   tft.drawRect(0, 0, SCREEN_HEIGHT, SCREEN_WIDTH, TFT_BLUE);
  }
  if (co2 >= CO2_CRITICAL_THRESHOLD) {
    tft.drawRect(0, 0, SCREEN_HEIGHT, SCREEN_WIDTH, (sysState.displayState == 0 ? TFT_BLUE : TFT_RED));
  }
}

//Loop for animations / Not hammering MQTT
void updateStates() {
  sysState.publishState = sysState.publishState >= MQTT_PUBLISH_INTERVAL ? 0 : sysState.publishState + 1;
  sysState.displayState = sysState.displayState >= 1 ? 0 : sysState.displayState + 1;
}

// Update display
void updateDisplay(float co2, float temperature, float humidity, float battery) {
  tft.fillScreen(TFT_BLACK);

  //Blue or Red highlight
  showWarning(co2);

  // Display sensor data
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setFreeFont(FSSB12);
  tft.setCursor(10, 30);
  tft.print("CO2 Level:");

  tft.setCursor(10, 65);
  tft.setFreeFont(FM12);
  char buf[20];
  dtostrf(co2, 10, 3, buf);
  tft.print(buf);
  tft.print(" ppm");

  tft.setFreeFont(FSSB9);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 100);
  tft.print("Temp: ");
  tft.setFreeFont(FM9);
  tft.print(temperature, 1);
  tft.setFreeFont(FSSB9);
  tft.print(" C");

  tft.setCursor(10, 120);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print("Humidity: ");
  tft.setFreeFont(FM9);
  tft.print(humidity, 1);
  tft.setFreeFont(FSSB9);
  tft.print(" %");

  // Display battery info
  tft.setFreeFont(FSS9);
  tft.setCursor(154, 20);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("Bat:");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(sysState.rollingBatteryVoltage, 1);
}

// ***************************************************

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SCD30 Device Starting ===");

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    if (LittleFS.format()) {
      Serial.println("LittleFS formatted, mounting...");
      if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed after format!");
        return;
      }
    } else {
      Serial.println("LittleFS format failed!");
      return;
    }
  }
  Serial.println("LittleFS mounted successfully");

  // Try to connect to WiFi
  if (!tryConnectWiFi()) {
    sysState.portalMode = true;
    startCaptivePortal();
    return;  // Exit setup, run in portal mode
  }

  // If we get here, WiFi is connected - initialize everything else
  Serial.println("=== Normal Mode - Initializing Systems ===");

  // Initialize I2C and sensor
  Wire.begin();
  sensor.begin(Wire, SCD30_I2C_ADDR_61);
  sensor.stopPeriodicMeasurement();
  sensor.softReset();
  delay(2000);

  // Initialize TFT display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FSSB18);

  // Check sensor firmware
  uint8_t major = 0;
  uint8_t minor = 0;
  sysState.lastError = sensor.readFirmwareVersion(major, minor);
  if (sysState.lastError != NO_ERROR) {
    Serial.print("Error reading firmware version: ");
    errorToString(sysState.lastError, sysState.errorBuffer, sizeof sysState.errorBuffer);
    Serial.println(sysState.errorBuffer);
    return;
  }

  Serial.print("Sensor firmware: ");
  Serial.print(major);
  Serial.print(".");
  Serial.println(minor);

  // Start sensor measurements
  sysState.lastError = sensor.startPeriodicMeasurement(0);
  if (sysState.lastError != NO_ERROR) {
    Serial.print("Error starting measurements: ");
    errorToString(sysState.lastError, sysState.errorBuffer, sizeof sysState.errorBuffer);
    Serial.println(sysState.errorBuffer);
    return;
  }

  // Initialize MQTT
  connectMQTT();

  // Publish Home Assistant configuration
  if (client.connected()) {
    publishSensorConfig("CO2", "home/sensor/co2", "ppm", "co2");
    publishSensorConfig("Temperature", "home/sensor/temperature", "°C", "temp");
    publishSensorConfig("Humidity", "home/sensor/humidity", "%", "humidity");
    publishSensorConfig("Battery", "home/sensor/batteryVoltage", "V", "battery");
  }

  // Initialize battery monitoring
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  int rawBattery = analogRead(PIN_BATTERY);
  sysState.rollingBatteryVoltage = ((float)rawBattery / 4095.0) * 2.0 * 3.3 * (1100 / 1000.0);

  sysState.initialized = true;
  Serial.println("=== System Initialized Successfully ===");
}


// #####################################################################
// #                                 MAIN LOOP                         #
// #####################################################################

void loop() {
  // Handle captive portal mode
  if (sysState.portalMode) {
    delay(100);
    return;
  }
  // Only proceed if system is initialized
  if (!sysState.initialized) {
    delay(1000);
    return;
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost, attempting reconnection...");
    if (!tryConnectWiFi()) {
      Serial.println("WiFi reconnection failed, restarting device...");
      ESP.restart();
    }
  }

  // Handle captive portal mode
  if (sysState.portalMode) {
    if (!sysState.portalDns && WiFi.softAPgetStationNum() >= 0) {
      // Start DNS server for captive portal
      //dns.setErrorReplyCode(DNSReplyCode::NoError);
      //dns.start(53, "*", WiFi.softAPIP());
      Serial.println("Setting up DNS");
      setupDNS();
      sysState.portalDns = true;
      Serial.println("DNS server started");
    }
    if (sysState.portalDns) {
      dns.processNextRequest();
      delay(DNS_INTERVAL);
    }
    delay(10);
    return;
  }

  // Only proceed if system is initialized
  if (!sysState.initialized) {
    delay(1000);
    return;
  }



  // Maintain MQTT connection
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Battery monitoring
  updateBatteryVoltage();
  if (isBatteryTooLow()) {
    Serial.println("Low battery, entering deep sleep...");
    esp_deep_sleep_start();
  }

  // Read sensor data
  float co2Concentration = 0.0;
  float temperature = 0.0;
  float humidity = 0.0;

  if (!readSensorData(co2Concentration, temperature, humidity)) {
    delay(2000);
    return;
  }

  updateDisplay(co2Concentration, temperature, humidity, sysState.rollingBatteryVoltage);
  updateStates();

  // Publish to MQTT every 5 seconds
  if (sysState.displayState == 0 && client.connected()) {
    if (sysState.warmupReadings > 0) {
      sysState.warmupReadings--;
      Serial.println("Skipping first measurement");
    } else {
      client.publish("home/sensor/co2", String(co2Concentration, 2).c_str());
      client.publish("home/sensor/temperature", String(temperature, 2).c_str());
      client.publish("home/sensor/humidity", String(humidity, 2).c_str());
      client.publish("home/sensor/batteryVoltage", String(sysState.rollingBatteryVoltage, 2).c_str());

      Serial.print("Published - CO2: ");
      Serial.print(co2Concentration);
      Serial.print(" ppm, Temp: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    }
  }
}