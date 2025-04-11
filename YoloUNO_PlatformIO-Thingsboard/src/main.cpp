#define LED_PIN 13
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#include <WiFi.h>
#include <WifiUdp.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT.h"  // Include the DHT sensor library
#include "Wire.h"
#include <ArduinoOTA.h>
#include <ArduinoJson.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

constexpr char TOKEN[] = "s5cob74hx2hwze5s63n5";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false; 
volatile uint16_t blinkingInterval = 5000;
volatile int ledmode = 0;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

void onAttributesReceived(const JsonObjectConst& data) {
  if (data.containsKey("ledMode")) {
      ledmode = data["ledMode"];  // Cập nhật giá trị ledmode
      Serial.print("Updated LED mode: ");
      Serial.println(ledmode);
  }
}

void requestSharedAttributes() {
  tb.Shared_Attributes_Request(Attribute_Request_Callback(onAttributesReceived));
}

DHT dht11(27, DHT11);  // Initialize DHT11 sensor on GPIO14 (adjust GPIO as needed)

void InitWiFi() {
  Serial.println("Connecting...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  vTaskDelay(pdMS_TO_TICKS(10000));

}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

RPC_Response setLedModeCallback(const RPC_Data &data) {
  Serial.println("Received RPC call: setLedMode");
  Serial.print("Raw data received: ");
  Serial.println(data.as<String>());   
  if (data.containsKey("params") && data["params"].containsKey("ledMode")) {
     ledMode = data["params"]["ledMode"];
    Serial.printf("LED mode updated: %d\n", ledMode);
    
    digitalWrite(LED_PIN, ledMode ? HIGH : LOW);
    return RPC_Response("LED mode updated", true);
}

  Serial.println(" Error: No ledMode parameter found in RPC call.");
  return RPC_Response("Error: No ledMode parameter", false);
}

void Task_DHT(void *pvParameters){
  Serial.println("Task DHT started");
  while (1){
    float temperature = dht11.readTemperature(); // Read temperature from DHT11
    float humidity = dht11.readHumidity(); // Read humidity from DHT11

    if (isnan(temperature)) {
      Serial.println("Failed to read from DHT11 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C");
      Serial.print(" Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void Task_Wifi(void *pvParameters){
  Serial.println("Task Wifi started");
  while(1){
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      InitWiFi();
    }
  vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void Task_MQTT(void *pvParameters){
  Serial.println("Task MQTT started");
  while(1){
  if (!tb.connected()) {
    Serial.println("Connecting to...");
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
    }
    Serial.println("Connected!");
    tb.RPC_Subscribe(RPC_Callback("setLedMode", setLedModeCallback));
    tb.Shared_Attributes_Subscribe(Shared_Attribute_Callback(onAttributesReceived));
    requestSharedAttributes(); 
}

tb.loop();
vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_LED(void *pvParameters){
  Serial.println("Task LED started");
  while (1) {
    if (ledMode == 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED is OFF"); 
      vTaskDelay(1000);
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED is ON");
      vTaskDelay(1000);
    }
  }
}


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  // InitWiFi();
  pinMode(LED_PIN, OUTPUT);
  // Wire.begin(SDA_PIN, SCL_PIN);
  dht11.begin();  // Initialize DHT11 sensor

  xTaskCreate(Task_Wifi, "Task Wifi", 10000, NULL, 1, NULL);
  xTaskCreate(Task_LED, "Task LED", 1000, NULL, 1, NULL);
  xTaskCreate(Task_MQTT, "Task MQTT", 5000, NULL, 2, NULL);
  xTaskCreate(Task_DHT, "Task DHT", 5000, NULL, 2, NULL);

}

void loop() {
}




