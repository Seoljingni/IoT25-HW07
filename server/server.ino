   
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define DEVICE_NAME "ESP32_Advertiser"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // 예시 UUID

void setup() {
  Serial.begin(115200);

  // BLE 초기화
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();  // 연결은 하지 않지만 필요

  // 서비스 UUID 설정 (선택 사항)
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pService->start();

  // 광고 설정
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);  // 서비스 UUID 포함
  pAdvertising->setScanResponse(true);         // 추가 정보 포함
  pAdvertising->setMinPreferred(0x06);         // 연결 설정 최적화
  pAdvertising->setMinPreferred(0x12);         // 연결 설정 최적화

  // 광고 시작
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising started (Advertiser only mode).");
}

void loop() {
  // 루프에서는 별도로 할 일 없음. 광고는 백그라운드에서 계속됨.
  delay(1000);
}
