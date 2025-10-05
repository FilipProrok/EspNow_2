#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 ATH;

typedef struct CommandData{ // moja struktura do wysyłania komendy w celu rządania danych
  uint8_t command;
};

typedef struct SensorData{
  float temperature;
  float humidity;
};

CommandData commandData;
SensorData sensorData;

uint8_t brodcastAddress[] = {0x4c, 0x11, 0xae, 0x65, 0x30, 0xe0}; // adres drugiego modułu

void setup_esp_now();
void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data,  int len);
void setup_sensor();

void setup() {
  Serial.begin(115200);
  setup_esp_now();
  setup_sensor();
}

void loop() {
  
}

void setup_esp_now(){
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSend);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;

  // Register peer
  memcpy(peerInfo.peer_addr, brodcastAddress, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
    // Add peer       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status){
  if(status != ESP_OK){
    Serial.println("problem z wysłaniem danych");
    return;
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data,  int len){
  memcpy(&commandData, incoming_data, sizeof(sensorData));
  if(commandData.command == 1){
    sensors_event_t temperature_event, humidity_event;
    ATH.getEvent(&temperature_event, &humidity_event);
    sensorData.temperature = temperature_event.temperature;
    sensorData.humidity = humidity_event.relative_humidity;
    esp_err_t result = esp_now_send(brodcastAddress, (uint8_t*) &sensorData, sizeof(sensorData));
  }
}

void setup_sensor(){
  if (!ATH.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}