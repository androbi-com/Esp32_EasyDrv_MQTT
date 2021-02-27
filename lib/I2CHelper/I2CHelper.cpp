#include "I2CHelper.h"
#include "Arduino.h"

int scan_i2c(bool serial)
{
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {            
            if (serial) {
                Serial.print("I2C device found at address 0x");
                if (address < 16) {
                    Serial.print("0");
                }
                Serial.println(address, HEX);
            }
            nDevices++;
        }
        else if (error == 4)
        {
            if (serial) {
                Serial.print("Unknow error at address 0x");
                if (address < 16) {
                    Serial.print("0");
                }
                Serial.println(address, HEX);
            }
        }
    }
    if (nDevices==0) {
        Serial.println("No I2C devices found");
    }
    return nDevices;
}

bool check_i2c_addr(uint8_t address)
{
    byte error;
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    return (error == 0);
}


I2CHelper::I2CHelper(uint8_t client) {
    this->client = client;
}
I2CHelper::~I2CHelper() {
    //
}

void I2CHelper::writeAddr(uint8_t reg) {
    Wire.beginTransmission(this->client);
    Wire.write(reg);
    Wire.endTransmission();
}

uint8_t I2CHelper::readUint8() {
    uint8_t value;
    byte buff[1];
    Wire.readBytes(buff, 1);
    value = buff[0];
    return value;
}

int16_t I2CHelper::readInt16() {
    int16_t value;
    byte buff[2];
    Wire.readBytes(buff, 2);
    value = (buff[1] << 8) | (buff[0] << 0);
    return value;
}

uint16_t I2CHelper::readUint16() {
    uint16_t value;
    byte buff[2];
    Wire.readBytes(buff, 2);
    value = (buff[1] << 8) | (buff[0] << 0);
    return value;
}

int32_t I2CHelper::readInt32() {
    int32_t value;
    byte buff[4];
    Wire.readBytes(buff, 4);
    value = (buff[3] << 24) | (buff[2] << 16) | (buff[1] << 8) | (buff[0] << 0);
    return value;
}

uint32_t I2CHelper::readUint32() {
    uint32_t value;
    byte buff[4];
    Wire.readBytes(buff, 4);
    value = (buff[3] << 24) | (buff[2] << 16) | (buff[1] << 8) | (buff[0] << 0);
    return value;
}

void I2CHelper::readToInt16Array(int16_t *i16_arr, uint8_t len) {
    byte buff[2*len];
    Wire.readBytes(buff, 2*len);
    for (uint8_t i=0; i<len; i++) {
        i16_arr[i] = (buff[2*i+1] << 8) | (buff[2*i] << 0);
    }
    return;
}

void I2CHelper::readToUint32Array(uint32_t *i32_arr, uint8_t len) {
    byte buff[4*len];
    Wire.readBytes(buff, 4*len);
    for (uint8_t i=0; i<len; i++) {
        i32_arr[i] = (buff[4*i+3] << 24) | (buff[4*i+2] << 16) | (buff[4*i+1] << 8) | (buff[4*i] << 0);
    }
    return;
}

uint8_t I2CHelper::getUint8(uint8_t reg) {
    this->writeAddr(reg);
    Wire.requestFrom(this->client, (uint8_t) 1);
    return this->readUint8();
}

int16_t I2CHelper::getInt16(uint8_t reg) {
    this->writeAddr(reg);
    Wire.requestFrom(this->client, (uint8_t) 2);
    return this->readInt16();
}

int32_t I2CHelper::getInt32(uint8_t reg) {
    this->writeAddr(reg);
    Wire.requestFrom(this->client, (uint8_t) 4);
    return this->readInt32();
}


void I2CHelper::getInt16Array(uint8_t reg, int16_t *i16_arr, uint8_t len) {
    this->writeAddr(reg);
    Wire.requestFrom((int)this->client, (int)(2*len));
    this->readToInt16Array(i16_arr,len);
    return;
}

void I2CHelper::beginSend(uint8_t reg) {
    Wire.beginTransmission(this->client);
    Wire.write(reg);
}

void I2CHelper::endSend() {
    Wire.endTransmission();
}

void I2CHelper::sendByte(uint8_t reg, uint8_t value) {
    this->beginSend(reg);
    Wire.write(value);  // send value
    this->endSend();
}

void I2CHelper::writeFloat(float value) {
    FloatByte_t x;
    x.number = value;
    Wire.write(x.bytes, 4);
}

void I2CHelper::writeInt16(int16_t value) {
    uint8_t bytes[2];
    bytes[0] = (value >> 0)  & 0xFF;
    bytes[1] = (value >> 8)  & 0xFF;
    Wire.write(bytes,2);
}

void I2CHelper::writeUint8(uint8_t value) {
    Wire.write(value);
}
