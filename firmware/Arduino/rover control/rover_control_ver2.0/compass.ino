#include <Wire.h>

#define COMPASS_ADDRESS           0x1E
#define COMPASS_REG_A             0x00
#define COMPASS_REG_B             0x01
#define COMPASS_MODE_REG          0x02
#define COMPASS_DATA_OUT_BEGIN    0x03
#define COMPASS_GAUSS_LSB         (0.73F)
#define COMPASS_PERIODE_MEASURE   56

#define RAD_TO_DEG        (57.295779513082320876798154814105F)
#define GRAVITY_EARTH     (9.80665F)


void compassInit(void)
{
  Wire.begin();
  write8bit(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8bit(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8bit(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);
  delay(COMPASS_PERIODE_MEASURE);
}


float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagnetometer(compassData);
  dataCompensate(compassData);

  if (compassData[1] && compassData[0])
    heading = atan2(compassData[1], compassData[0]);

  if (heading < 0)
    heading = heading + 2 * PI;

  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  return (heading * RAD_TO_DEG);
}


void dataCompensate(float compassData[3])
{
  double calibrationMatrix[3][3] =
  {
    {1.136, -0.007, 0.018},
    { -0.003, 1.023, -0.002},
    {0.045, -0.074, 1.398}
  };

  double bias[3] =
  {
    -38.215,
    -43.315,
    -10.107
  };

  for (int i = 0; i < 3; ++i)
    compassData[i] = compassData[i] - bias[i];

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      compassData[i] += calibrationMatrix[i][j] * compassData[j];
}


void readMagnetometer(float rawData[3])
{
  Wire.beginTransmission((byte)COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_BEGIN);
  Wire.requestFrom((byte)COMPASS_ADDRESS, (byte)6);
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


void write8bit(byte address, int reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

