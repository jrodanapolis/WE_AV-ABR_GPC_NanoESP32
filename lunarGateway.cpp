/*

  lunarGateway.cpp - Library for connecting the ESP32 with the Lunar 2021 scale

  Created by Frowin Ellermann, 2022.

  Released under the MIT license.

  https://github.com/frowin/LunarGateway

*/

#include "lunarGateway.h"
#include <stdio.h>

lunarGateway::lunarGateway(){
  _lastHeartBeat = 0;
}

bool lunarGateway::read(){
  if (_readChar.valueUpdated()) {
    _readChar.readValue(_readBuffer, sizeof(_readBuffer));
    decodeMessage(_readBuffer);
    return true;
  }

  return false;
}

bool lunarGateway::connect(BLEDevice *dev){
  if (!dev)
    return false;
  if (dev->connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return false;
  }

  // discover peripheral attributes
  if (!dev->discoverAttributes()) {
    Serial.println("Attribute discovery failed!");
    dev->disconnect();
    return false;
  }

  BLEService service = dev->service("49535343-FE7D-4AE5-8FA9-9FAFD205E455");
  if (!service) {
    Serial.println("Peripheral does not have needed service!");
    dev->disconnect();
    return false;
  }
  _commandChar = service.characteristic("49535343-8841-43f4-a8d4-ecbe34729bb3");
  if (!_commandChar) {
    Serial.println("Peripheral does not have command characteristic!");
    dev->disconnect();
    return false;
  } else if (!_commandChar.canWrite()) {
    Serial.println("Peripheral does not have a writable command characteristic!");
    dev->disconnect();
    return false;
  }
  Serial.println("Found our command characteristic!");
  if (!_commandChar.canSubscribe()) {
    Serial.println("Peripheral command characteristic is not subscribable!");
  } else {
    if (_commandChar.subscribe()) {
      Serial.println("command subscription failed!");
    }
  }

  _readChar = service.characteristic("49535343-1e4d-4bd9-ba61-23c647249616");
  if (!_readChar) {
    Serial.println("Peripheral does not have read characteristic!");
    dev->disconnect();
    return false;
  }
  Serial.println("Found our read characteristic!");
  if (_readChar.canWrite())
    Serial.println("Peripheral does not have a writable read characteristic!");
  
  if (!_readChar.canSubscribe()) {
    Serial.println("Peripheral read characteristic is not subscribable!");
  } else {
    if (_readChar.subscribe()) {
      Serial.println("read subscription failed!");
    }
  }
    _peripheral = *dev;
  return true;
}

bool lunarGateway::connected(){
  if (_peripheral && _peripheral.connected())
    return true;
  return false;
}

void lunarGateway::getSettings(){
  sendId(); 
  uint8_t payload_msg[16] = {0};
  uint8_t* prepared_msg = _prepareRequest(6,payload_msg, (size_t)sizeof(payload_msg));
  _commandChar.writeValue(prepared_msg,sizeof(payload_msg)+5);
}

void lunarGateway::doTare(){
  sendId(); 
  uint8_t payload_msg[1] = {0};
  uint8_t* prepared_msg = _prepareRequest(4,payload_msg, (size_t)sizeof(payload_msg));
  _commandChar.writeValue(prepared_msg,sizeof(payload_msg)+5);
}

void lunarGateway::startTimer(){
  sendId(); 
  uint8_t payload_msg[2] = {0, 0};
  uint8_t* prepared_msg = _prepareRequest(13,payload_msg, (size_t)sizeof(payload_msg));
  _commandChar.writeValue(prepared_msg,sizeof(payload_msg)+5);
}

void lunarGateway::stopTimer(){
  sendId(); 
  uint8_t payload_msg[2] = {0,2};
  uint8_t* prepared_msg = _prepareRequest(13,payload_msg, (size_t)sizeof(payload_msg));
  _commandChar.writeValue(prepared_msg,sizeof(payload_msg)+5);
}

void lunarGateway::resetTimer(){
  sendId(); 
  uint8_t payload_msg[2] = {0,1};
  uint8_t* prepared_msg = _prepareRequest(13,payload_msg, (size_t)sizeof(payload_msg));
  _commandChar.writeValue(prepared_msg,sizeof(payload_msg)+5);
}

void lunarGateway::sendId(){
  uint8_t id_array[21] = {0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d, 0x00};
  _commandChar.writeValue(id_array,21);
}

void lunarGateway::notificationRequest(){
  sendId();
  uint8_t id_array[14] = {0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06 };
  _commandChar.writeValue(id_array,14);
}

void lunarGateway::sendHeartBeat(){
  if ((millis() - _lastHeartBeat) > 2750) {
    notificationRequest();
    sendId();
    uint8_t payload_msg[2] = {2,0};
    uint8_t* heartbeat_msg = _prepareRequest(0,payload_msg, (size_t)sizeof(payload_msg));
    _commandChar.writeValue(heartbeat_msg, sizeof(payload_msg)+5);
    _lastHeartBeat = millis();
  }
}

uint8_t* lunarGateway::_prepareRequest(int msgtype,uint8_t * message, size_t size){
  static uint8_t request[32];
  request[0] = 0xef;
  request[1] = 0xdd;
  request[2] = msgtype;
  uint8_t checksum1 = 0;
  uint8_t checksum2 = 0;
  uint8_t tempval = 0;
  for (int i = 0; i < size; i++) {
  tempval = message[i] & 0xff;
  request[3+i] = tempval;
  if (i % 2 == 0){
    checksum1 += tempval;
  }else{
    checksum2 += tempval;
  }
  }
  request[size + 3] = (checksum1 & 0xff);
  request[size + 4] = (checksum2 & 0xff);
  return request;
}

void lunarGateway::decodeMessage(uint8_t* message){
  if(false){ // set to true for debug information
    for (int i = 0; i < 32; i++){
      char buf[9];
      sprintf(buf, "%02x", message[i]);
      Serial.print(buf);
      Serial.print(" ");
    }
    Serial.print("\n");
  }

  int messageStart = -1;
  int messageEnd = -1;
  
  for (int i = 0; message[i]!=0 ; i++){
    if(message[i] == 0xEF && message[i+1] == 0xDD){
      messageStart=i;
      break;
    }
  }
  
  messageEnd  = messageStart+message[messageStart+3]+5;
  uint8_t msgTopic = message[messageStart+2];
  uint8_t msgType = message[messageStart+4];
  uint8_t* msgPayloadMessage = _substr(message, messageStart+5, messageEnd);
  uint8_t* msgPayloadSettings = _substr(message, messageStart+4, messageEnd);

  if(msgTopic == 0x08){
    battery = msgPayloadSettings[0] & 0x7f;
  }
  
  if(msgTopic == 0x0C && msgType == 0x05){
    int weight_int = ((msgPayloadMessage[1] & 0xff) << 8) + (msgPayloadMessage[0] & 0xff);
    char weight_unit = msgPayloadMessage[4] & 0xff;
    float weight_float = (float)weight_int;
    if(weight_unit == 1){
      weight_float /= 10.0;
    } else if(weight_unit == 2){
      weight_float /= 100.0;
    } else if(weight_unit == 3){
      weight_float /= 1000.0;
    } else if(weight_unit == 4){
      weight_float /= 10000.0;
    }
    if ((msgPayloadMessage[5] & 0x02) == 0x02){
      weight_float *= -1;
    }
    weight = weight_float;
  }
  if (msgPayloadMessage)
    delete(msgPayloadMessage);
  if (msgPayloadSettings)
    delete(msgPayloadSettings);
}

uint8_t* lunarGateway::_substr(const uint8_t* src, int start_index, int end_index) {
  int length = end_index - start_index;
  if (length < 0)
    return NULL;
  uint8_t* dest = new uint8_t[length + 1];
  std::copy(src + start_index, src + end_index, dest); 
  dest[length] = '\0';
  return dest;
}