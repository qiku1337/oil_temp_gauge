#include "KWP.h"
#include <Arduino.h>

#define DEBUG_LEVEL 1

KWP::KWP(uint8_t receivePin, uint8_t transmitPin){
  _OBD_RX_PIN = receivePin;
  _OBD_TX_PIN = transmitPin;

  pinMode(transmitPin, OUTPUT);
  digitalWrite(transmitPin, HIGH);

  obd = new NewSoftwareSerial(receivePin, transmitPin, false); // RX, TX, inverse logic

  #ifdef DEBUG_LEVEL
    Serial.println(F("KWP created"));
  #endif
}

KWP::~KWP(){
  delete obd;
  obd = NULL;
}

bool KWP::connect(uint8_t addr, int baudrate) {
  Serial.print(F("------connect addr="));
  Serial.print(addr);
  Serial.print(F(" baud="));
  Serial.println(baudrate);
  blockCounter = 0;
  obd->begin(baudrate);
  KWP5BaudInit(addr);
  char s[3];
  int size = 3;
  if (!KWPReceiveBlock(s, 3, size)) return false;
  if (    (((uint8_t)s[0]) != 0x55)
     ||   (((uint8_t)s[1]) != 0x01)
     ||   (((uint8_t)s[2]) != 0x8A)   ){
    Serial.println(F("ERROR: invalid magic"));
    disconnect();
    errorData++;
    return false;
  }
  connected = true;
  if (!readConnectBlocks()) return false;
  return true;
}

void KWP::disconnect() {
  connected = false;
}

int KWP::readBlock(uint8_t addr, int group, int maxSensorsPerBlock, SENSOR resGroupSensor[]) {
  SENSOR sensor;
  Serial.print(F("------readBlock "));
  Serial.println(group);
  char s[64];
  sprintf(s, "\x04%c\x29%c\x03", blockCounter, group);
  if (!KWPSendBlock(s, 5)) return false;
  int size = 0;
  KWPReceiveBlock(s, 64, size);
  if (s[2] != '\xe7') {
    Serial.println(F("ERROR: invalid answer"));
    disconnect();
    errorData++;
    return 0;
  }
  int count = (size-4) / 3;
  if (count > maxSensorsPerBlock) {
    Serial.println(F("ERROR: max sensors exceded"));
    disconnect();
    errorData++;
    return 0;
  }
  String blockDescs= getBlockDesc(addr, group);
  int len=blockDescs.length();
  char buf[len+1];
  blockDescs.toCharArray(buf, len+1);
  char* command = strtok(buf, ",");
  Serial.print(F("count="));
  Serial.println(count);
  int j=0;
  for (int idx=0; idx < count; idx++){
    byte k=s[3 + idx*3];
    byte a=s[3 + idx*3+1];
    byte b=s[3 + idx*3+2];
    String desc=String(command);
    SENSOR sensor = getSensorData(k, a, b);
    if(desc != "" && sensor.value != ""){
      resGroupSensor[j].type = sensor.type;
      resGroupSensor[j].a = sensor.a;
      resGroupSensor[j].b = sensor.b;
      resGroupSensor[j].desc = desc;
      resGroupSensor[j].value = sensor.value;
      resGroupSensor[j].units = sensor.units;
      j++;
    }
    command = strtok(0, ",");
  }
  return j;
}

SENSOR KWP::getSensorData(byte k, byte a, byte b) {
    SENSOR res;
    Serial.print(F("type="));
    Serial.print(k);
    Serial.print(F("  a="));
    Serial.print(a);
    Serial.print(F("  b="));
    Serial.print(b);
    Serial.print(F("  text="));
    String t = "";
    float v = 0;
    String units = "";
    char buf[32];
    switch (k){
      case 5:  v=a*(b-100)*0.1;       units=F("c");break;
      case 37: v=b; break; // oil pressure ?!
      default: sprintf(buf, "%2x, %2x      ", a, b); break;
    }

    if (units.length() != 0){
      dtostrf(v,4, 2, buf);
      t=String(buf) + " " + units;
    }
    Serial.println(t);

    res.type = k;
    res.a = a;
    res.b = b;
    res.value = String(buf);
    res.units = units;
    return res;
}

String KWP::getBlockDesc(uint8_t addr, int block){
  String blockDescs;
  if(addr == ADR_Dashboard){
    switch (block){
      case 1: blockDescs=F("Speed,Engine Speed,Oil pressure,Time"); break;
      case 2: blockDescs=F("Odometer,Fuel lvl,FuelSend,TAmbient"); break; // case 2: blockDescs=F("Odometer,Fuel level (l),Fuel Sender,Ambient"); break;
      case 3: blockDescs=F("Coolant Temp,Oil level,Oil,[N/A]"); break;
      case 22: blockDescs=F("Starting,Engine (ECM),Key condition,Number of"); break;
      case 23: blockDescs=F("Variable code,Key status,Fixed code,Immobilizer"); break;
      case 24: blockDescs=F("Instrument,Engine control,Emergency,Transponder"); break;
      case 25: blockDescs=F("Immobilizer,[N/A],[N/A],[N/A]"); break;
      case 50: blockDescs=F("Odometer,Engine Speed,Oil,Coolant"); break;
      case 125: blockDescs=F("Engine,Transmission,ABS,[N/A]"); break;
      case 126: blockDescs=F("Steering,Airbag,[N/A],[N/A]"); break;
    default: blockDescs=F(""); break;
    }

  }
  //Label 06A-906-032-AUM.lbl
  else if(addr == ADR_Engine){
    switch (block){

      default: blockDescs=F(""); break;
    }
  }
  else{
    Serial.println("Not description found for that address");
  }
  return blockDescs;
}

bool KWP::isConnected() {
  return connected;
}

void KWP::obdWrite(uint8_t data) {
  obd->write(data);
}

uint8_t KWP::obdRead() {
  unsigned long timeout = millis() + 1000;
  while (!obd->available()){
    if (millis() >= timeout) {
      Serial.println(F("ERROR: obdRead timeout"));
      disconnect();
      errorTimeout++;
      return 0;
    }
  }
  uint8_t data = obd->read();
  return data;
}

bool KWP::KWP5BaudInit(uint8_t addr){
  Serial.println(F("---KWP 5 baud init"));
  //delay(3000);
  send5baud(addr);
  return true;
}

void KWP::send5baud(uint8_t data) {
  #define bitcount 10
  byte bits[bitcount];
  byte even=1;
  byte bit;
  for (int i=0; i < bitcount; i++){
    bit=0;
    if (i == 0)  bit = 0;
      else if (i == 8) bit = even; // computes parity bit
      else if (i == 9) bit = 1;
      else {
        bit = (byte) ((data & (1 << (i-1))) != 0);
        even = even ^ bit;
      }
    Serial.print(F("bit"));
    Serial.print(i);
    Serial.print(F("="));
    Serial.print(bit);
    if (i == 0) Serial.print(F(" startbit"));
      else if (i == 8) Serial.print(F(" parity"));
      else if (i == 9) Serial.print(F(" stopbit"));
    Serial.println();
    bits[i]=bit;
  }
  for (int i=0; i < bitcount+1; i++){
    if (i != 0){
      delay(200);
      if (i == bitcount) break;
    }
    if (bits[i] == 1){
      digitalWrite(_OBD_TX_PIN, HIGH);
    } else {
      digitalWrite(_OBD_TX_PIN, LOW);
    }
  }
  obd->flush();
}

bool KWP::KWPSendBlock(char *s, int size) {
  Serial.print(F("---KWPSend sz="));
  Serial.print(size);
  Serial.print(F(" blockCounter="));
  Serial.println(blockCounter);
  Serial.print(F("OUT:"));
  for (int i=0; i < size; i++){
    uint8_t data = s[i];
    Serial.print(data, HEX);
    Serial.print(" ");
  }
  Serial.println();
  for (int i=0; i < size; i++){
    uint8_t data = s[i];
    obdWrite(data);
    if (i < size-1){
      uint8_t complement = obdRead();
      if (complement != (data ^ 0xFF)){
        Serial.println(F("ERROR: invalid complement"));
        disconnect();
        errorData++;
        return false;
      }
    }
  }
  blockCounter++;
  return true;
}

bool KWP::KWPReceiveBlock(char s[], int maxsize, int &size, bool init_delay) {
  bool ackeachbyte = false;
  uint8_t data = 0;
  int recvcount = 0;
  if (size == 0) ackeachbyte = true;
  Serial.print(F("---KWPReceive sz="));
  Serial.print(size);
  Serial.print(F(" blockCounter="));
  Serial.println(blockCounter);
  if (size > maxsize) {
    Serial.println("ERROR: invalid maxsize");
    return false;
  }
  unsigned long timeout = millis() + 2000;  // TODO: This allows connect to different Modules
  //unsigned long timeout = millis() + 1000;
  while ((recvcount == 0) || (recvcount != size)) {
    while (obd->available()){
      data = obdRead();
      s[recvcount] = data;
      recvcount++;
      if ((size == 0) && (recvcount == 1)) {
        size = data + 1;
        if (size > maxsize) {
          Serial.println("ERROR: invalid maxsize");
          return false;
        }
      }
      if ((ackeachbyte) && (recvcount == 2)) {
        if (data != blockCounter){
          Serial.println(F("ERROR: invalid blockCounter"));
          disconnect();
          errorData++;
          return false;
        }
      }
      if ( ((!ackeachbyte) && (recvcount == size)) ||  ((ackeachbyte) && (recvcount < size)) ){
        obdWrite(data ^ 0xFF);
      }
      timeout = millis() + 1000;
    }
    if (millis() >= timeout){
      Serial.println(F("ERROR: timeout"));
      disconnect();
      errorTimeout++;
      return false;
    }
  }
  Serial.print(F("IN: sz="));
  Serial.print(size);
  Serial.print(F(" data="));
  for (int i=0; i < size; i++){
    uint8_t data = s[i];
    Serial.print(data, HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  blockCounter++;
  return true;
}

bool KWP::KWPSendAckBlock() {
  Serial.print(F("---KWPSendAckBlock blockCounter="));
  Serial.println(blockCounter);
  char buf[32];
  sprintf(buf, "\x03%c\x09\x03", blockCounter);
  return (KWPSendBlock(buf, 4));
}

bool KWP::readConnectBlocks() {
  Serial.println(F("------readconnectblocks"));
  String info;
  while (true){
    int size = 0;
    char s[64];
    if (!(KWPReceiveBlock(s, 64, size))) return false;
    if (size == 0) return false;
    if (s[2] == '\x09') break;
    if (s[2] != '\xF6') {
      Serial.println(F("ERROR: unexpected answer"));
      disconnect();
      errorData++;
      return false;
    }
    String text = String(s);
    info += text.substring(3, size-2);
    if (!KWPSendAckBlock()) return false;
  }
  Serial.print("label=");
  Serial.println(info);
  return true;
}
