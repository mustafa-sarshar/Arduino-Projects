/*
Inspired by: 
    1- Paul McWhorter,      Source: https://www.youtube.com/watch?v=2AO_Gmh5K3Q&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&ab_channel=PaulMcWhorter
    2- DroneBot Workshop,   Source: https://www.youtube.com/watch?v=bEKjCDDUPaU&t=1556s&ab_channel=DroneBotWorkshop
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BaudRate 115200 // 500000
#define BNO055_SMAPLERATE_DELAY_MS 50
#define DeviceID 1
#define FSR1_PIN 35
#define FSR2_PIN 33
#define FSR3_PIN 34

const bool debugDataSent = false;
const float GRAVITY {9.80665}; // standard acceleration of gravity
const float Pi {3.141592653589793}; // Pi value

float dt {};
unsigned long counter {0};
unsigned long milisOld {};

int8_t temperature = 0;
uint8_t sys, gyr, acc, mag {};

Adafruit_BNO055 BNO = Adafruit_BNO055(-1, BNO055_ADDRESS_A);

// Define data structure
typedef struct struct_data {
    uint8_t device_id=DeviceID;
    unsigned long counter {};
    unsigned long millis {};
    uint8_t calb_sys {};
    uint8_t calb_acc {};
    uint8_t calb_gyr {};
    uint8_t calb_mag {};
    float acc_x {};
    float acc_y {};
    float acc_z {};
    float gyr_x {};
    float gyr_y {};
    float gyr_z {};
    float mag_x {};
    float mag_y {};
    float mag_z {};
    float grav_x {};
    float grav_y {};
    float grav_z {};
    float linearAcc_x {};
    float linearAcc_y {};
    float linearAcc_z {};
    float euler_x {};
    float euler_y {};
    float euler_z {};
    float quat_w {};
    float quat_x {};
    float quat_y {};
    float quat_z {};
    float roll {};
    float pitch {};
    float yaw {};
    int8_t temperature {};
    uint16_t FSR_1 {};
    uint16_t FSR_2 {};
    uint16_t FSR_3 {};
} struct_data;

// Create structured data object
struct_data data;

// Responder MAC Address (Replace with your responders MAC Address)
uint8_t broadcastAddress[] = {0xxx, 0xxx, 0xxx, 0xxx, 0xxx, 0xxx}; // Change the Address to your server's MAc Address

// Register peer
esp_now_peer_info_t peerInfo;

// Sent data callback function
void OnDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    if (debugDataSent == true) {
        Serial.print("Last Packet Send Status: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
        Serial.println("Size of data: "+String(sizeof(data)));
    }
}

float rad2deg(float radian) {
    return radian * (180.0/Pi);
}

float deg2rad(float degree) {
    return degree * (Pi/180.0);
}

void setup(void) {
    if (debugDataSent == true) Serial.begin(BaudRate);
    BNO.begin();
    BNO.setExtCrystalUse(true);
    WiFi.mode(WIFI_STA); // Set ESP32 WiFi mode to Station temporarly
    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
        if (debugDataSent == true) Serial.println("Error initializing ESP-NOW");
        return;
    }    
    esp_now_register_send_cb(OnDataSent); // Define callback

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        if (debugDataSent == true) Serial.println("Failed to add peer");
        return;
    }

    milisOld=millis();
    if (debugDataSent == true) Serial.println("Setup Finished !!!");
}

void loop(void) {
    dt = ((float)(millis()-milisOld))/1000.0f;
    milisOld = millis();
    
    // *** Get data from the sensor and assign it to data ***
    get_counter(false, false, false);
    get_millis(false, false, false);
    get_calibration_data(BNO, false, false, false, false);
    get_acc(BNO, false, false, true);
    get_gyr(BNO, false, false, false);
    get_mag(BNO, false, false, false);
    get_grav(BNO, false, false, false);
    get_linearAcc(BNO, false, false, false);
    get_euler(BNO, false, false, false);
    get_quat(BNO, false, false, false, false);
    get_temperature_data(BNO, false, false, false);
    get_FSR_data(false, false, false);

    if (debugDataSent == true) Serial.println("");
    esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data)); // Send data
    delay(BNO055_SMAPLERATE_DELAY_MS);
    counter++;
}

void get_counter(bool print_data, bool print_data_start, bool print_data_close) {
    data.counter = counter;
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Counter\':");
        Serial.print(data.counter);
        if (print_data_close == true) Serial.println("}"); 
    }    
}

void get_millis(bool print_data, bool print_data_start, bool print_data_close) {
    data.millis = millis();
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Millis\':");
        Serial.print(data.millis);
        if (print_data_close == true) Serial.println("}"); 
    }
}

void get_calibration_data(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close, bool print_summary) {
    bno.getCalibration(&sys, &gyr, &acc, &mag);
    // Add to structured data object
    data.calb_sys = sys;
    data.calb_acc = acc;
    data.calb_gyr = gyr;
    data.calb_mag = mag;
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        if (print_summary == true) Serial.print("SysSum: "+String((sys+acc+gyr+mag)/4));
        else {
            Serial.print("\'Sys_Cal\':");
            Serial.print(sys);
            Serial.print(",\'Acc_Cal\':");
            Serial.print(acc);
            Serial.print(",\'Gyr_Cal\':");
            Serial.print(gyr);
            Serial.print(",\'Mag_Cal\':");
            Serial.print(mag);
        }
        if (print_data_close == true) Serial.println("}");
    }
}

void get_acc(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Add to structured data object
    data.acc_x = acc.x();
    data.acc_y = acc.y();
    data.acc_z = acc.z();    
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Acc_X\':");
        Serial.print(acc.x());
        Serial.print(",\'Acc_Y\':");
        Serial.print(acc.y());
        Serial.print(",\'Acc_Z\':");
        Serial.print(acc.z());
        if (print_data_close == true) Serial.println("}"); 
    }
}

void get_gyr(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> gyr=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // Add to structured data object
    data.gyr_x = gyr.x();
    data.gyr_y = gyr.y();
    data.gyr_z = gyr.z();    
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Gyr_X\':");
        Serial.print(gyr.x());
        Serial.print(",\'Gyr_Y\':");
        Serial.print(gyr.y());
        Serial.print(",\'Gyr_Z\':");
        Serial.print(gyr.z());
        if (print_data_close == true) Serial.println("}");
    }    
}

void get_mag(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> mag=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // Add to structured data object
    data.mag_x = mag.x();
    data.mag_y = mag.y();
    data.mag_z = mag.z();
    
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Mag_X\':");
        Serial.print(mag.x());
        Serial.print(",\'Mag_Y\':");
        Serial.print(mag.y());
        Serial.print(",\'Mag_Z\':");
        Serial.print(mag.z());        
        if (print_data_close == true) Serial.println("}");
    }
}

void get_grav(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> grav=bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    // Add to structured data object
    data.grav_x = grav.x();
    data.grav_y = grav.y();
    data.grav_z = grav.z();
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Grav_X\':");
        Serial.print(grav.x());
        Serial.print(",\'Grav_Y\':");
        Serial.print(grav.y());
        Serial.print(",\'Grav_Z\':");
        Serial.print(grav.z());
        if (print_data_close == true) Serial.println("}");
    }
}

void get_linearAcc(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> linearAcc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Add to structured data object
    data.linearAcc_x = linearAcc.x();
    data.linearAcc_y = linearAcc.y();
    data.linearAcc_z = linearAcc.z();
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'LinearAcc_X\':");
        Serial.print(linearAcc.x());
        Serial.print(",\'LinearAcc_Y\':");
        Serial.print(linearAcc.y());
        Serial.print(",\'LinearAcc_Z\':");
        Serial.print(linearAcc.z());
        if (print_data_close == true) Serial.println("}");
    }
}

void get_euler(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    imu::Vector<3> euler=bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // Add to structured data object
    data.euler_x = euler.x();
    data.euler_y = euler.y();
    data.euler_z = euler.z();
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Euler_X\':");
        Serial.print(euler.x());
        Serial.print(",\'Euler_Y\':");
        Serial.print(euler.y());
        Serial.print(",\'Euler_Z\':");
        Serial.print(euler.z());
        if (print_data_close == true) Serial.println("}");
    }
}

void get_quat(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close, bool print_orientation) {
    imu::Quaternion quat=bno.getQuat();
    // Add to structured data object
    data.quat_w = quat.w();
    data.quat_x = quat.x();
    data.quat_y = quat.y();
    data.quat_z = quat.z();

    data.roll = rad2deg(atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()), 1-2*(quat.x()*quat.x()+quat.y()*quat.y()))); // math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
    data.pitch = rad2deg(asin(2*(quat.w()*quat.y()-quat.z()*quat.x())));                                             // math.asin(2*(q0*q2-q3*q1))
    data.yaw = rad2deg(atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()), 1-2*(quat.y()*quat.y()+quat.z()*quat.z())));   // atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2

    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print(",\'Quat_W\':");
        Serial.print(quat.w());
        Serial.print(",\'Quat_X\':");
        Serial.print(quat.x());
        Serial.print(",\'Quat_Y\':");
        Serial.print(quat.y());
        Serial.print(",\'Quat_Z\':");
        Serial.print(quat.z());
        if (print_orientation == true) {
            Serial.print(",\'Roll:\':");
            Serial.print(data.roll);
            Serial.print(",\'Pitch\':");
            Serial.print(data.pitch);
            Serial.print(",\'Yaw\':");
            Serial.print(data.yaw);
        }
        if (print_data_close == true) Serial.println("}");
    }
}

void get_temperature_data(Adafruit_BNO055 bno, bool print_data, bool print_data_start, bool print_data_close) {
    temperature = bno.getTemp();
    data.temperature =temperature;
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print("\',Temperature\':");
        Serial.print(temperature);
        if (print_data_close == true) Serial.println("}");
    }
}

void get_FSR_data(bool print_data, bool print_data_start, bool print_data_close){
    // data.FSR_1 = map(analogRead(FSR1_PIN), 0, 1023, 0, 100);
    // data.FSR_2 = map(analogRead(FSR2_PIN), 0, 1023, 0, 100);
    // data.FSR_3 = map(analogRead(FSR3_PIN), 0, 1023, 0, 100);
    data.FSR_1 = analogRead(FSR1_PIN);
    data.FSR_2 = analogRead(FSR2_PIN);
    data.FSR_3 = analogRead(FSR3_PIN);
    if (print_data == true) {
        if (print_data_start == true) Serial.print("{");
        Serial.print("\',FSR_1\':");
        Serial.print(data.FSR_1);
        Serial.print("\',FSR_2\':");
        Serial.print(data.FSR_2);
        Serial.print("\',FSR_3\':");
        Serial.print(data.FSR_3);
        if (print_data_close == true) Serial.println("}");
    }
}
