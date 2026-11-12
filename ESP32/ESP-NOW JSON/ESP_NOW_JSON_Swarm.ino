#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <ArduinoJson.h>

// ==========================================
// CONFIGURATION
// ==========================================

// Change this identifier on each board
#define DEVICE_NAME "ESP32_A"

// Replace with the OTHER ESP32's Wi-Fi MAC address
// Example: AA:BB:CC:DD:EE:FF
uint8_t peerAddress[] = {
    0x98, 0x3D, 0xAE, 0xED, 0x2A, 0xAC
};

// Both boards must use the same Wi-Fi channel
#define WIFI_CHANNEL 1

#define SEND_INTERVAL 1000

// ==========================================
// GLOBAL VARIABLES
// ==========================================

uint32_t packetSequence = 0;
unsigned long previousSendTime = 0;

// Receive buffer
char rxBuffer[250];
volatile bool packetReceived = false;
volatile int rxLength = 0;

portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// Send status
volatile bool sendComplete = false;
volatile bool sendSuccess = false;

// ==========================================
// ESP-NOW CALLBACKS
// ==========================================

// Called after a transmission completes
// ESP32 Arduino Core 3.x callback signature
void OnDataSent(const wifi_tx_info_t *info,
                esp_now_send_status_t status) {

    sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    sendComplete = true;
}

// Called when a JSON packet is received
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len) {

    if (len > 0 && len < sizeof(rxBuffer)) {

        portENTER_CRITICAL(&rxMux);

        memcpy(rxBuffer, data, len);
        rxLength = len;
        packetReceived = true;

        portEXIT_CRITICAL(&rxMux);
    }
}

// ==========================================
// INITIALIZE ESP-NOW
// ==========================================

void initializeESPNow() {

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    // Set communication channel
    esp_wifi_set_channel(
        WIFI_CHANNEL,
        WIFI_SECOND_CHAN_NONE
    );

    // Retrieve local MAC address
    uint8_t mac[6];

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {

        Serial.printf(
            "Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac[0], mac[1], mac[2],
            mac[3], mac[4], mac[5]
        );
    }

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    // Register callbacks
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Configure peer
    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, peerAddress, 6);

    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer!");
        return;
    }

    Serial.println("ESP-NOW initialized successfully!");
}

// ==========================================
// TRANSMIT JSON PACKET
// ==========================================

void sendJSON() {

    JsonDocument doc;

    doc["device"] = DEVICE_NAME;
    doc["sequence"] = packetSequence++;
    doc["uptime_ms"] = millis();
    doc["message"] = "Hello from ESP32_A!";

    char jsonBuffer[250];

    size_t len = serializeJson(
        doc,
        jsonBuffer,
        sizeof(jsonBuffer)
    );

    if (len == 0) {
        Serial.println("JSON serialization failed!");
        return;
    }

    esp_err_t result = esp_now_send(
        peerAddress,
        (uint8_t *)jsonBuffer,
        len
    );

    if (result == ESP_OK) {
        Serial.print("Sending JSON: ");
        Serial.println(jsonBuffer);
    } else {
        Serial.printf("Send request failed: %d\n", result);
    }
}

// ==========================================
// PROCESS RECEIVED JSON PACKET
// ==========================================

void processReceivedJSON() {

    char localBuffer[250];
    int localLength = 0;
    bool available = false;

    // Safely copy received data
    portENTER_CRITICAL(&rxMux);

    if (packetReceived) {

        localLength = rxLength;
        memcpy(localBuffer, rxBuffer, localLength);

        packetReceived = false;
        available = true;
    }

    portEXIT_CRITICAL(&rxMux);

    if (!available) return;

    localBuffer[localLength] = '\0';

    JsonDocument doc;

    DeserializationError error = deserializeJson(
        doc,
        localBuffer,
        localLength
    );

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    Serial.println("\n--- RECEIVED JSON PACKET ---");

    Serial.print("Device: ");
    Serial.println(doc["device"].as<const char *>());

    Serial.print("Sequence: ");
    Serial.println(doc["sequence"].as<uint32_t>());

    Serial.print("Uptime: ");
    Serial.println(doc["uptime_ms"].as<uint32_t>());

    Serial.print("Message: ");
    Serial.println(doc["message"].as<const char *>());

    Serial.println("----------------------------\n");
}

// ==========================================
// SETUP
// ==========================================

void setup() {

    Serial.begin(115200);
    delay(2000);

    Serial.println("\nESP32-S3 Initializing...");

    initializeESPNow();
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

    // Process incoming packets
    processReceivedJSON();

    // Send packets periodically
    if (millis() - previousSendTime >= SEND_INTERVAL) {

        previousSendTime = millis();

        sendJSON();
    }

    // Report completed transmissions
    if (sendComplete) {

        sendComplete = false;

        if (sendSuccess) {
            Serial.println("Transmission successful!");
        } else {
            Serial.println("Transmission failed!");
        }
    }
}

