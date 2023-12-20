#include <Wire.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "LAPTOP";
const char* password = "12345678";

WiFiUDP udp;

MPU6050 mpu6050(Wire);

float offsetX = 0, offsetY = 0, offsetZ = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(20000);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Gather initial readings to calculate offsets
  const int readings = 100;
  for(int i = 0; i < readings; ++i) {
    mpu6050.update();
    offsetX += mpu6050.getAngleX();
    offsetY += mpu6050.getAngleY();
    offsetZ += mpu6050.getAngleZ();
    delay(100);
  }

  // Calculate the average to find the offset
  offsetX /= readings;
  offsetY /= readings;
  offsetZ /= readings;

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  mpu6050.update();

  float currentX = mpu6050.getAngleX() - offsetX;
  float currentY = mpu6050.getAngleY() - offsetY;
  float currentZ = mpu6050.getAngleZ() - offsetZ;

  String message = String(currentX) + "," + String(currentY) + "," + String(currentZ) + "\n";

  udp.beginPacket("192.168.23.88", 4210);
  udp.print(message);
  udp.endPacket();

  Serial.print(currentX, 5); 
  Serial.print(",");
  Serial.print(currentY, 5);
  Serial.print(",");
  Serial.println(currentZ, 5);

  delay(100);
}
