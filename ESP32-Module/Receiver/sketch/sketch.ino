/*
Inspired by: 
    1- Paul McWhorter,      Source: https://www.youtube.com/watch?v=2AO_Gmh5K3Q&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&ab_channel=PaulMcWhorter
    2- DroneBot Workshop,   Source: https://www.youtube.com/watch?v=bEKjCDDUPaU&t=1556s&ab_channel=DroneBotWorkshop
*/

// Include required libraries
#include <WiFi.h>
#include <esp_now.h>

#define BaudRate 115200 // 500000

unsigned long counter_rec {0};
unsigned long millis_rec {};

// Define data structure
typedef struct struct_data {
    uint8_t device_id {};       // 0
    unsigned long counter {};   // 1
    unsigned long millis {};    // 2
    uint8_t calb_sys {};        // 3
    uint8_t calb_acc {};        // 4
    uint8_t calb_gyr {};        // 5
    uint8_t calb_mag {};        // 6
    float acc_x {};             // 7
    float acc_y {};             // 8
    float acc_z {};             // 9
    float gyr_x {};             // 10
    float gyr_y {};             // 11
    float gyr_z {};             // 12
    float mag_x {};             // 13
    float mag_y {};             // 14
    float mag_z {};             // 15
    float grav_x {};            // 16
    float grav_y {};            // 17
    float grav_z {};            // 18
    float linearAcc_x {};       // 19
    float linearAcc_y {};       // 20
    float linearAcc_z {};       // 21
    float euler_x {};           // 22
    float euler_y {};           // 23
    float euler_z {};           // 24
    float quat_w {};            // 25
    float quat_x {};            // 26
    float quat_y {};            // 27
    float quat_z {};            // 28
    float roll {};              // 29
    float pitch {};             // 30
    float yaw {};               // 31
    int8_t temperature {};      // 32
    uint16_t FSR_1 {};          // 33
    uint16_t FSR_2 {};          // 34
    uint16_t FSR_3 {};          // 35
    unsigned long counter_rec {0};  // 36
    unsigned long millis_rec {};    // 37
} struct_data;

// Create structured data object
struct_data data;

// Callback function
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // Get incoming data
    memcpy(&data, incomingData, sizeof(data));

    // Print to Serial Monitor
    millis_rec = millis();
    data.counter_rec = counter_rec;
    data.millis_rec = millis_rec;
    
    // Serial.print("[");
    // Serial.print(
    //     String(data.device_id)+","
    //     + String(data.counter)+","+String(data.millis)+","
    //     + String(data.calb_sys)+","+String(data.calb_acc)+","+String(data.calb_gyr)+","+String(data.calb_mag)+","
    //     + String(data.acc_x)+","+String(data.acc_y)+","+String(data.acc_z)+","
    //     + String(data.gyr_x)+","+String(data.gyr_y)+","+String(data.gyr_z)+","
    //     + String(data.mag_x)+","+String(data.mag_y)+","+String(data.mag_z)+","
    //     + String(data.grav_x)+","+String(data.grav_y)+","+String(data.grav_z)+","
    //     + String(data.linearAcc_x)+","+String(data.linearAcc_y)+","+String(data.linearAcc_z)+","
    //     + String(data.euler_x)+","+String(data.euler_y)+","+String(data.euler_z)+","
    //     + String(data.quat_w)+","+String(data.quat_x)+","+String(data.quat_y)+","+String(data.quat_z)+","
    //     + String(data.roll)+","+String(data.pitch)+","+String(data.yaw)+","
    //     + String(data.temperature)+","
    //     + String(data.FSR_1)+","+String(data.FSR_2)+","+String(data.FSR_3)+","
    //     + String(data.counter_rec)+","+String(data.millis_rec)
    // );
    // Serial.println("]");
    
    Serial.write((uint8_t *) &data, sizeof(data));  // Serial.write(buf, len) allows to send any data types
    // Serial.println(sizeof(data));
    counter_rec++;
}
 
void setup() {
    // Set up Serial Monitor
    Serial.begin(BaudRate);

    // Start ESP32 in Station mode
    WiFi.mode(WIFI_STA);

    // Initalize ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callback function
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
}