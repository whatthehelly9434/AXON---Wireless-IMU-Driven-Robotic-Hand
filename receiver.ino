#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

Servo servos[5];
const int servoPins[5] = {2, 3, 4, 5, 6}; //pin choices are arbitary, use what suits you
const bool invert[5]   = {true, true, false, true, false}; // TUNE: flip if servo moves wrong direction
const int stepDelay    = 1;                               // TUNE: higher = slower movement

typedef struct {
  float fingers[5];
  float wrist;
} GlovePacket;

int currentAngles[5];
int targetAngles[5];

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  GlovePacket packet;
  memcpy(&packet, data, sizeof(packet));

  Serial.printf("W:%.1f T:%.1f I:%.1f M:%.1f R:%.1f P:%.1f\n",
    packet.wrist,
    packet.fingers[0], packet.fingers[1],
    packet.fingers[2], packet.fingers[3], packet.fingers[4]);

  // fingers arrive 0=straight, 180=bent, already mapped
  // sender order: thumb(0), index(1), middle(2), ring(3), pinky(4)
  // servos wired pinky to thumb so reverse
  for (int i = 0; i < 5; i++) {
    int a = constrain((int)packet.fingers[4 - i], 0, 180);
    targetAngles[i] = invert[i] ? 180 - a : a;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1);
  }
  esp_now_register_recv_cb(onReceive);

  for (int i = 0; i < 5; i++) {
    servos[i].attach(servoPins[i]);
    currentAngles[i] = 0;
    targetAngles[i]  = 0;
    servos[i].write(0);
  }

  Serial.println("Receiver ready, waiting for data...");
}

void loop() {
  for (int i = 0; i < 5; i++) {
    if (currentAngles[i] < targetAngles[i])      currentAngles[i]++;
    else if (currentAngles[i] > targetAngles[i]) currentAngles[i]--;
    servos[i].write(currentAngles[i]);
  }
  delay(stepDelay);
}
