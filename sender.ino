#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t receiverMAC[] = {}; //flash your receiver esp and enter mac adress here 

#define TCA_ADDR    0x70
#define MPU_ADDR    0x68
#define ALPHA       0.95
#define SAMPLES     100
#define SDA_PIN     20
#define SCL_PIN     21
#define NUM_SENSORS 6

typedef struct {
  float fingers[5];
  float wrist;
} GlovePacket;

struct IMU {
  float pitch, roll, gx_off, gy_off, ax_off, ay_off, az_off;
};

IMU imus[NUM_SENSORS];
unsigned long lastTime = 0;
esp_now_peer_info_t peer;

float fingerOffsets[5] = {0, 0, 0, 0, 0};
float wristOffset = 0;

// order: pinky, ring, middle, index, thumb
const float fingerMax[5]  = {80, 80, 80, 80, 30};
const float wristCoeff[5] = {0.833f, 0.744f, 0.706f, 0.872f, 1.500f};

void tcaClose() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(10);
}

void mpuInit(uint8_t ch) {
  tcaSelect(ch);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  tcaClose();
}

void readRaw(uint8_t ch, IMU &imu, float &ax, float &ay, float &az, float &gx, float &gy) {
  tcaSelect(ch);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t axR = Wire.read() << 8 | Wire.read();
  int16_t ayR = Wire.read() << 8 | Wire.read();
  int16_t azR = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gxR = Wire.read() << 8 | Wire.read();
  int16_t gyR = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();

  ax = axR / 16384.0f - imu.ax_off;
  ay = ayR / 16384.0f - imu.ay_off;
  az = azR / 16384.0f - imu.az_off;
  gx = gxR / 131.0f   - imu.gx_off;
  gy = gyR / 131.0f   - imu.gy_off;

  tcaClose();
}

void calibrate(uint8_t ch, IMU &imu) {
  float sax=0, say=0, saz=0, sgx=0, sgy=0;
  float ax, ay, az, gx, gy;
  for (int i = 0; i < SAMPLES; i++) {
    readRaw(ch, imu, ax, ay, az, gx, gy);
    sax += ax; say += ay; saz += az; sgx += gx; sgy += gy;
    delay(5);
  }
  imu.ax_off = sax / SAMPLES;
  imu.ay_off = say / SAMPLES;
  imu.az_off = saz / SAMPLES - 1.0f;
  imu.gx_off = sgx / SAMPLES;
  imu.gy_off = sgy / SAMPLES;
}

void updateIMUs() {
  float ax, ay, az, gx, gy;
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6f;
  lastTime = now;

  for (int i = 0; i < NUM_SENSORS; i++) {
    readRaw(i + 2, imus[i], ax, ay, az, gx, gy);
    float pitchAcc = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / PI;
    float rollAcc  = atan2(ax, sqrt(ay*ay + az*az)) * 180.0f / PI;
    imus[i].pitch = ALPHA * (imus[i].pitch + gx * dt) + (1.0f - ALPHA) * pitchAcc;
    imus[i].roll  = ALPHA * (imus[i].roll  + gy * dt) + (1.0f - ALPHA) * rollAcc;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1);
  }

  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("TCA9548A not found");
    while(1);
  }
  Serial.println("Scanning channels...");
  for (int ch = 0; ch < 8; ch++) {
    tcaSelect(ch);
    Wire.beginTransmission(MPU_ADDR);
    int err = Wire.endTransmission();
    Serial.printf("  Channel %d: %s\n", ch, err == 0 ? "MPU FOUND" : "nothing");
    tcaClose();
  }

  for (int i = 0; i < NUM_SENSORS; i++) mpuInit(i + 2);
  delay(100);

  Serial.println("Calibrating, keep hand flat and open...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("  Sensor %d...\n", i + 1);
    calibrate(i + 2, imus[i]);
  }

  // warm up filter
  lastTime = micros();
  for (int i = 0; i < 50; i++) {
    updateIMUs();
    delay(10);
  }

  // zero at calibration position with per-finger wrist compensation
  updateIMUs();
  wristOffset = imus[5].roll;
  for (int i = 0; i < 5; i++) {
    fingerOffsets[i] = imus[i].pitch - (imus[5].roll * wristCoeff[i]);
  }

  Serial.println("Zeroed. Starting...");
  lastTime = micros();
}

void loop() {
  updateIMUs();

  float wristRoll = imus[5].roll - wristOffset;

  GlovePacket packet;
  for (int i = 0; i < 5; i++) {
    float wristComp = wristRoll * wristCoeff[i];
    float angle = (imus[i].pitch + wristComp) - fingerOffsets[i];
    packet.fingers[i] = constrain(angle / fingerMax[i] * 180.0f, 0, 180);
  }
  packet.wrist = constrain(wristRoll / -70.0f * 180.0f, 0, 180);

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&packet, sizeof(packet));

  Serial.printf("Wrist:%.1f  P:%.1f R:%.1f M:%.1f I:%.1f T:%.1f  [%s]\n",
    packet.wrist,
    packet.fingers[0], packet.fingers[1], packet.fingers[2],
    packet.fingers[3], packet.fingers[4],
    result == ESP_OK ? "OK" : "FAIL");

  delay(20);
}
