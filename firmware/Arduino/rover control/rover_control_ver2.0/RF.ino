#define delayRF(numByte) (4 + numByte)


void serialRead(void)
{
  unsigned int index = 0;
  unsigned char dataArray[10] = {0};

  while (Serial1.available() > 0)
  {
    dataArray[index] = Serial1.read();
    index++;
    delayMicroseconds(500);

    if (index > 2)
      break;
  }

  if (index == 1)
  {
    switch (dataArray[0])
    {
      case 'a':
        motorForward(autoVelocity);
        state = AUTO_STATE;
        break;
    }
  }

  if (index == 2)
  {
    switch (dataArray[0])
    {
      case '0':
        if (stopMode == NORMAL_STOP)
          motorNormalStop();
        else if (stopMode == FAST_STOP)
          motorFastStop();
        break;

      case '1':
        motorForward(dataArray[1]);
        break;

      case '2':
        motorBackward(dataArray[1]);
        break;

      case '3':
        motorTurnRight(dataArray[1]);
        break;

      case '4':
        motorTurnLeft(dataArray[1]);
        break;
    }
  }
}


void serialReadAuto(void)
{
  unsigned char data = 0;

  if (Serial1.available() > 0)
    data = Serial1.read();

  if (data == 'm')
  {
    if (stopMode == NORMAL_STOP)
      motorNormalStop();
    else if (stopMode == FAST_STOP)
      motorFastStop();

    state = MANUAL_STATE;
  }
}


void send6charRF(char val0, char val1, char val2, char val3, char val4, char val5)
{
  char finalArray[7] = {val0, val1, val2, val3, val4, val5, '\n'};
  Serial1.write(finalArray, 7);
  delayRF(7);
}

