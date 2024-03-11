/*

  lunarGateway.cpp - Library for connecting the ESP32 with the Lunar 2021 scale

  Created by Frowin Ellermann, 2022.

  Released under the MIT license.

  https://github.com/frowin/LunarGateway

*/

#ifndef LUNAR_GATEWAY_H
#define LUNAR_GATEWAY_H

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <stdio.h>

class lunarGateway {
public:
  lunarGateway();
  bool connect(BLEDevice *dev);
  bool connected();
  bool read();
  void sendId();
  void sendHeartBeat();
  void doTare();
  void startTimer();
  void notificationRequest();
  void stopTimer();
  void resetTimer();
  void getSettings();
  void decodeMessage(uint8_t* message);

  float weight;
  int battery;

private:
  unsigned long _lastHeartBeat;
  uint8_t _readBuffer[32];
  BLEDevice _peripheral;
  BLECharacteristic _commandChar;
  BLECharacteristic _readChar;
  uint8_t* _prepareRequest(int msgtype,uint8_t * message, size_t size);
  uint8_t* _substr(const uint8_t* src, int start_index, int end_index);
};

#endif