#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_AHTX0.h>
#include <esp_wifi.h>

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

  // Pick the channel your other module is using. Change this to match the sender.
  const uint8_t espnow_channel = 6; // <-- set this to the same channel as the other module
  esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSend);
  esp_now_register_recv_cb(OnDataRecv);

  // Print local MAC and channel for debugging
  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());
  uint8_t cur_chan = 0; wifi_second_chan_t sec; // wifi_second_chan_t from esp_wifi.h
  if (esp_wifi_get_channel(&cur_chan, &sec) == ESP_OK) {
    Serial.print("WiFi channel: ");
    Serial.println(cur_chan);
  }

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // clear to avoid garbage in other fields

  // Register peer (the other module)
  memcpy(peerInfo.peer_addr, brodcastAddress, 6);
  peerInfo.channel = espnow_channel; 
  peerInfo.encrypt = false;
  // ensure interface is STA
#if defined(ESP_INTERFACE_WIFI_STA)
  peerInfo.ifidx = ESP_IF_WIFI_STA;
#else
  peerInfo.ifidx = WIFI_IF_STA; // fallback name
#endif

  esp_err_t add_err = esp_now_add_peer(&peerInfo);
  if (add_err != ESP_OK){
    Serial.print("Failed to add peer: ");
    Serial.println(add_err);
    return;
  } else {
    Serial.println("Peer added successfully");
  }
}

void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if(status != ESP_OK){
    Serial.print("Problem sending to "); Serial.print(macStr);
    Serial.print(" status="); Serial.println((int)status);
    return;
  }
  Serial.print("Send OK to "); Serial.println(macStr);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data,  int len){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Received packet from: "); Serial.print(macStr);
  Serial.print("  len="); Serial.println(len);

  if(len != sizeof(commandData)){
    Serial.println("Received unexpected payload size");
    return;
  }

  memcpy(&commandData, incoming_data, sizeof(commandData));
  if(commandData.command == 1){
    sensors_event_t temperature_event, humidity_event;
    ATH.getEvent(&temperature_event, &humidity_event);
    sensorData.temperature = temperature_event.temperature;
    sensorData.humidity = humidity_event.relative_humidity;
    esp_err_t result = esp_now_send(brodcastAddress, (uint8_t*) &sensorData, sizeof(sensorData));
    if(result != ESP_OK){
      Serial.print("Failed to send sensor data, err="); Serial.println(result);
    } else {
      Serial.println("Sensor data sent");
    }
  }
}

void setup_sensor(){
  if (!ATH.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}