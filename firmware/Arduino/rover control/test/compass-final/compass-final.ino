#include <Wire.h>

#define COMPASS_ADDRESS          0x1E
#define COMPASS_REG_A            0x00
#define COMPASS_REG_B            0x01
#define COMPASS_MODE_REG         0x02
#define COMPASS_DATA_OUT_BEGIN   0x03
#define RAD_TO_DEG               (57.295779513082320876798154814105F)
#define COMPASS_GAUSS_LSB        (0.73F)
#define COMPASS_PERIODE_MEASURE  56

float headingDetect(void);
void compassCompensate(float compassData[3]);
void readMagneticAxis(float rawData[3]);
void compassInit(void);
void write8(int address, int reg, int value);
byte read8(int address, int reg);


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  compassInit();
}


void loop()
{
  printHeading();
  delay(100);
}



void printHeading(void)
{
  float heading = headingDetect();
  Serial.print("Heading: ");
  Serial.println(heading);
}


float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagneticAxis(compassData);
  compassCompensate(compassData);

  if (compassData[1] && compassData[0])
    heading = atan2(compassData[1], compassData[0]);

  if (heading < 0)
    heading = heading + 2 * PI;

  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  return (heading * RAD_TO_DEG);
}


void compassCompensate(float compassData[3])
{
  double calibrationMatrix[3][3] =
  {
    {1.019, 0.001, 0.003},
    {0.032, 0.97, -0.048},
    { -0.02, -0.017, 1.274}
  };

  double bias[3] =
  {
    -34.311,
    -212.987,
    -122.717
  };

  for (int i = 0; i < 3; ++i)
    compassData[i] = compassData[i] - bias[i];

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      compassData[i] += calibrationMatrix[i][j] * compassData[j];
}


void readMagneticAxis(float rawData[3])
{
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_BEGIN);
  Wire.requestFrom(COMPASS_ADDRESS, 6);
  while (Wire.available() < 6);
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  Wire.endTransmission();

  rawData[0] = (float)((int16_t)(xlo | ((int16_t)xhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[1] = (float)((int16_t)(ylo | ((int16_t)yhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[2] = (float)((int16_t)(zlo | ((int16_t)zhi << 8))) * COMPASS_GAUSS_LSB;
}


void compassInit(void)
{
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);
  delay(COMPASS_PERIODE_MEASURE);
}


void write8(int address, int reg, int value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


byte read8(int address, int reg)
{
  byte value;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

