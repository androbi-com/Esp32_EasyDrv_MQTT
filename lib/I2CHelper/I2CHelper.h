/*
 I2CHelper - functions for I2C messages
  Claus Denk
  https://androbi.com
*/

#ifndef I2CHelper_h
#define I2CHelper_h

#include <Arduino.h>
#include <Wire.h>

typedef union
{
    float number;
    uint8_t bytes[4];
} FloatByte_t;

int scan_i2c(bool serial);
bool check_i2c_addr(uint8_t address);

class I2CHelper {
   public:
      uint8_t client;

      I2CHelper(uint8_t client);
      ~I2CHelper();
      void writeAddr(uint8_t reg);
      void beginSend(uint8_t reg);
      void endSend();

      uint8_t readUint8();
      int16_t readInt16();
      uint16_t readUint16();
      int32_t readInt32();
      uint32_t readUint32();

      void readToInt16Array(int16_t *i16_arr, uint8_t len);
      void readToUint32Array(uint32_t *i32_arr, uint8_t len);

      uint8_t getUint8(uint8_t reg);
      int16_t getInt16(uint8_t reg);
      int32_t getInt32(uint8_t reg);

      void getInt16Array(uint8_t reg, int16_t *i16_arr, uint8_t len);
      void sendByte(uint8_t reg, uint8_t value);
      void writeFloat(float value);
      void writeInt16(int16_t value);
      void writeUint8(uint8_t value);
};

#endif
