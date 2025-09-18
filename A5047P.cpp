#include <Arduino.h>
#include <SPI.h>

#define CS_PIN D10  // 使用 D10 作為 CS

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // STM32 Arduino 框架會自動使用 D11,D12,D13 作為 SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  Serial.println("STM32 SPI 初始化完成 (D10,D11,D12,D13)");
  
  // 基本 SPI 測試
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10);
  uint16_t test = SPI.transfer16(0xFFFF);  // 送出 0xFFFF 測試
  digitalWrite(CS_PIN, HIGH);
  
  Serial.print("SPI 基本測試: 0x");
  Serial.println(test, HEX);
}

uint16_t readAS5047P() {
  uint16_t command = 0x3FFF;  // ANGLEUNC 暫存器
  
  // AS5047P 需要兩次傳輸
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(2);
  SPI.transfer16(command);  // 第一次：送命令
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);  // 增加間隔時間
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(2);
  uint16_t result = SPI.transfer16(0x0000);  // 第二次：讀資料
  digitalWrite(CS_PIN, HIGH);
  
  return result;
}

void loop() {
  uint16_t raw_data = readAS5047P();
  
  Serial.print("原始資料: 0x");
  Serial.print(raw_data, HEX);
  
  if (raw_data & 0x4000) {
    Serial.println(" - 錯誤標誌");
  } else {
    uint16_t angle = raw_data & 0x3FFF;
    float angle_deg = (360.0 * angle) / 16383.0;
    Serial.print(" - 角度: ");
    Serial.print(angle_deg);
    Serial.println("°");
  }
  
  delay(100);
}