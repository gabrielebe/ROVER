#include <Wire.h>

#define BAUD_RATE                9600
#define ACC_X                    A0
#define ACC_Y                    A1
#define ACC_Z                    A2
#define COMPASS_REG_A            0x00
#define COMPASS_REG_B            0x01
#define COMPASS_MODE_REG         0x02
#define COMPASS_DATA_OUT_X_MSB   0x03
#define COMPASS_ADDRESS          0x1E
#define COMPASS_PERIODE_MEASURE  56

int command = -1;

void matlabInit(void);
void compassInit(void);
void readMagneticAxis(unsigned int* x, unsigned int* y, unsigned int* z);
void write8(int address, int reg, int value);


void setup()
{
  matlabInit();
  compassInit();
}


void loop()
{
  if (Serial.available() > 0)
  {
    command = Serial.read();

    if (command == 'A')
    {
      unsigned int SensorValue = analogRead(ACC_X);
      Serial.println(SensorValue);
      SensorValue = analogRead(ACC_Y);
      Serial.println(SensorValue);
      SensorValue = analogRead(ACC_Z);
      Serial.println(SensorValue);
    }

    if (command == 'M')
    {
      unsigned int xMag, yMag, zMag;
      readMagneticAxis(&xMag, &yMag, &zMag);
      Serial.println(xMag);
      Serial.println(yMag);
      Serial.println(zMag);
    }
  }

  delayMicroseconds(1000);
}



void matlabInit(void)
{
  Serial.begin(BAUD_RATE);
  Serial.println('a');
  char a = 'b';

  while (a != 'a')
    a = Serial.read();
}


void compassInit(void)
{
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);
  delay(COMPASS_PERIODE_MEASURE);
}


void readMagneticAxis(unsigned int* xMag, unsigned int* yMag, unsigned int* zMag)
{
  *xMag = 0;
  *yMag = 0;
  *zMag = 0;

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_X_MSB);
  Wire.requestFrom(COMPASS_ADDRESS, 6);
  while (Wire.available() < 6);
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  Wire.endTransmission();

  *xMag = (unsigned int)(xlo | ((int16_t)xhi << 8));
  *yMag = (unsigned int)(ylo | ((int16_t)yhi << 8));
  *zMag = (unsigned int)(zlo | ((int16_t)zhi << 8));
}


void write8(int address, int reg, int value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

