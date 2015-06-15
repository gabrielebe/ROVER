// Including all the libraries needed
#include <SoftwareSerial.h>     // This library is used to communicate with serial protocol with the RF module
#include <Wire.h>               // This library is used to communicate with the real time clock device
#include <SPI.h>                // This library is used by the adafruit graphics library
#include <Adafruit_GFX.h>       // This library is used to create graphics on the LCD display
#include <Adafruit_PCD8544.h>   // This library is used to display data on the LCD
#include <RTClib.h>             // This library is used to get data from the RTC device
#include "radio_comand.h"       // This is configuration file containing all the necessary information for modularity

void ConnectToPc(void)
{
  unsigned long waitTime = 0;		      // Variable used for TIMEOUT
  Serial.println('a');			      // Sending a character to PC
  char a = 'b';
  waitTime = millis() + TIMEOUT;	      // Starting timeout time
  display.setTextSize(1);
  display.println("Connecting to");	      // Displaying status on LCD
  display.setTextSize(3);
  display.println(" PC");
  display.display();

  while (a != 'a' && isTherePc)	              // Checking any Response
  {
    a = Serial.read();		              // Reading from Serial port
    isTherePc = millis() > TIMEOUT ? 0 : 1;   // Checking whether a timeout  has occurred or not

    if (isTherePc == 0)		              // If timeout has occurred show on display
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("No computer");
      display.println("Connection");
      display.display();
      delay(1000);
      display.clearDisplay();
    }
    else if (a == 'a')
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("Don't touch");
      display.println("Connected to PC");
      display.display();
      display.clearDisplay();
      isTherePc = 1;
    }
  }
}


char *receiveData(char type, unsigned int time = 1000)
{
  char dataArray[10] = {0};       // Buffer used to store communication data
  unsigned long wait = millis() + time;

  if (type == FIXED) while (!softSerial.available());
  if (type == TIME) while (millis() < wait && !softSerial.available());

  if (softSerial.available() > 0)
  {
    while (softSerial.available() && index < 10)
    {
      dataArray[index++] = softSerial.read();
      delayMicroseconds(100);
    }
    index--;

    if (index == 7)
    {
      if (dataArray[0] == ACCELEROMETER || dataArray[0] == COMPASS)
      {
        if (isTherePc)
        {
          Serial.println(dataArray[1] + (dataArray[2] << 8));     // Print the x axis
          Serial.println(dataArray[3] + (dataArray[4] << 8));     // Print the y axis
          Serial.println(dataArray[5] + (dataArray[6] << 8));     // Print the z axis
        }
      }
      index = 0;
      return dataArray;
    }

    else if (index == 3)
    {
      if (dataArray[0] == TEMPERATURE)
      {
        if (isTherePc)
        {
          Serial.println(dataArray[1] + (dataArray[2] << 8));
        }
      }
      index = 0;
      return dataArray;
    }
    else
    {
      index = 0;
    }
  }
}


void initialize(void)
{
  Serial.begin(115200);          // Used to connect with
  softSerial.begin(38400);       // This is the maximum velocity with S2 closed
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));

  display.begin();
  display.setContrast(65);
  display.clearDisplay();
  display.setTextSize(2);
  display.println(" ROVER");
  display.setTextSize(1);
  display.println(" Harpal Singh");
  display.println(" Gabriele Be'");
  display.println(" Alice Rosa");
  display.display();

  pinMode(SWITCH_UP, INPUT);
  pinMode(SWITCH_DOWN, INPUT);
  pinMode(SWITCH_LEFT, INPUT);
  pinMode(SWITCH_RIGHT, INPUT);
  pinMode(SWITCH_PICKER, INPUT);

  joystickX1_start = map(analogRead(joyX1), 0, 1023, 0, 50);
  joystickY1_start = map(analogRead(joyY1), 0, 1023, 0, 50);
  joystickX2_start = map(analogRead(joyX2), 0, 1023, 0, 50);
  joystickY2_start = map(analogRead(joyY2), 0, 1023, 0, 50);

  delay(2500);
  display.clearDisplay();
}


void screenRefresh(void)
{
  if ((millis() - waitTime) > REFRESHRATE)
  {
    waitTime = millis();
    showTime();
  }
}


void showTime(void)
{
  DateTime now = rtc.now();
  display.clearDisplay();
  display.setTextSize(1);
  display.println("     Date");
  display.setTextSize(2);

  if (now.day() <= 9 && now.day() >= 0)
    display.print(0);

  display.print(now.day(), DEC);
  display.print('/');

  if (now.month() <= 9 && now.month() >= 0)
    display.print(0);

  display.print(now.month(), DEC);
  display.setTextSize(1);
  display.println(now.year(), DEC);
  display.println("     Time");
  display.setTextSize(2);

  if (now.hour() <= 9 && now.hour() >= 0)
    display.print(0);

  display.print(now.hour(), DEC);
  display.print(':');

  if (now.minute() <= 9 && now.minute() >= 0)
    display.print(0);

  display.print(now.minute(), DEC);
  display.setTextSize(1);

  if (now.second() <= 9 && now.second() >= 0)
    display.print(0);

  display.println(now.second(), DEC);
  display.display();
  waitTime = millis() - REFRESHRATE + 1000;
}


void readButton(void)
{
  if (digitalRead(SWITCH_UP) == LOW)
  {
    menu_update(UP);
    delay(200);
  }

  if (digitalRead(SWITCH_DOWN) == LOW)
  {
    menu_update(DOWN);
    delay(200);
  }

  if (digitalRead(SWITCH_LEFT) == LOW)
  {
    menu_update(LEFT);
    delay(200);
  }

  if (digitalRead(SWITCH_RIGHT) == LOW)
  {
    menu_update(RIGHT);
    delay(200);
  }
}


void sendArmCommand(void)
{
  readPicker();
  byte joyX2_value = (byte)map(analogRead(joyX2), 0, 1023, 0, 50);
  byte joyY2_value = (byte)map(analogRead(joyY2), 0, 1023, 0, 50);

  if (joyX2_value != joystickX2_start || joyY2_value != joystickY2_start)
  {
    if (joyX2_value > joystickX2_start + 2)
    {
      joyX2_value = map(joyX2_value, joystickX2_start, 50, 0, armVelocity);
      moveCommand(ARM_UP, joyX2_value);
      delay(joyX2_value + 20);
    }

    else if (joyX2_value < joystickX2_start - 2)
    {

      joyX2_value = map(joyX2_value, joystickX2_start, 0, 0, armVelocity);
      moveCommand(ARM_DOWN, joyX2_value);
      delay(joyX2_value + 20);

    }
    if (joyY2_value >= joystickY2_start + 2)
    {
      joyY2_value = map(joyY2_value, joystickY2_start, 50, 0, armVelocity);
      moveCommand(ARM_LEFT, joyY2_value);
      delay(joyY2_value + 20);

    }
    else if (joyY2_value < joystickY2_start - 2)
    {
      joyY2_value = map(joyY2_value, joystickY2_start, 0, 0, armVelocity);
      moveCommand(ARM_RIGHT, joyY2_value);
      delay(joyY2_value + 20);
    }
  }
}


void readPicker(void)
{
  static boolean last_status = 0;
  if (digitalRead(SWITCH_PICKER))
  {
    last_status = !last_status;
    moveCommand(ARM_PICKER, last_status);
    delay(100);
  }
}


void sendMoveCommand(void)
{
  byte joyX1_value = (byte)map(analogRead(joyX1), 0, 1023, 0, 50);
  byte joyY1_value = (byte)map(analogRead(joyY1), 0, 1023, 0, 50);

  if (joyX1_value != joystickX1_start || joyY1_value != joystickY1_start)
  {
    if (joyX1_value >= joystickX1_start + 2)
    {
      joyX1_value = map(joyX1_value, joystickX1_start, 50, 0, maxVelocity);
      moveCommand(GO_BACKWARD, joyX1_value);
    }

    else if (joyX1_value < joystickX1_start - 2)
    {
      {
        joyX1_value = map(joyX1_value, joystickX1_start, 0, 0, maxVelocity);
        moveCommand(GO_FORWARD, joyX1_value);
      }
    }
    if (joyY1_value >= joystickY1_start + 2)
    {

      joyY1_value = map(joyY1_value, joystickY1_start, 50, 0, maxVelocity);
      moveCommand(GO_LEFT, joyY1_value);

    }
    else if (joyY1_value < joystickY1_start - 2)
    {
      joyY1_value = map(joyY1_value, joystickY1_start, 0, 0, maxVelocity);
      moveCommand(GO_RIGHT, joyY1_value);

    }
  }
  else
  {
    moveCommand(GO_STOP, 255);
  }
}


void menu_update(char instruction)
{
  delayScreenRefresh;
  static int Menu_position = 0;

  switch (instruction)
  {
    case LEFT:
      if (Menu_position > 0) Menu_position--;
      break;

    case RIGHT:
      if (Menu_position < NUMBER_OF_FUNCTIONS) Menu_position++;
      break;

    case UP:
      switch (Menu_position)
      {
        case 0: changeSensor(UP);
          break;
        case 1: toggleMode();
          break;
        case 2: toggleLcdMode();
          break;
        case 3: changeVelocity(5);
          break;
        case 4: toggleArm();
          break;
        case 5: changeArmVelocity(1);
          break;
        case 6: toggleStop();
          break;
      }
      break;

    case DOWN:
      switch (Menu_position)
      {
        case 0: changeSensor(DOWN);
          break;
        case 1: toggleMode();
          break;
        case 2: toggleLcdMode();
          break;
        case 3: changeVelocity(-5);
          break;
        case 4: toggleArm();
          break;
        case 5: changeArmVelocity(-1);
          break;
        case 6: toggleStop();
          break;
      }
      break;
  }
  updateScreen(Menu_position);
}


void updateScreen(char level)
{
  switch (level)
  {
    case 0:  showSensor(last_sensor);
      break;

    case 1:  showMode();
      break;

    case 2:  showLcdMode();
      break;

    case 3:  showVelocity();
      break;

    case 4:  showArmMode();
      break;

    case 5:  showArmVelocity();
      break;

    case 6:  showStopMode();
      break;
  }
}


void changeSensor(char change)
{
  if (change == UP) last_sensor++;
  if (change == DOWN) last_sensor--;
  if (last_sensor < 0) last_sensor = 0;
  if (last_sensor > 2) last_sensor = 2;
}


void showSensor(byte sensor)
{
  int variable;
  char *array;
  char stringArray[4] = {0};

  while (softSerial.available() > 0)
    variable = softSerial.read();

  variable = 0;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  switch (sensor)
  {
    case 0: softSerial.write(TEMPERATURE);
      display.println("Temperature:");
      break;

    case 1: softSerial.write(ACCELEROMETER);
      display.println("Accelerometer:");
      break;

    case 2: softSerial.write(COMPASS);
      display.println("Compass:");
      break;
  }

  display.display();
  delay(delayRF(1));
  array = receiveData(TIME);

  if (array[0] == -1)
  {
    display.setTextSize(2);
    display.println("ERROR!");
  }
  else
    switch (array[0])
    {
      case TEMPERATURE:
        variable = array[1];
        display.setTextSize(2);
        display.print(variable, DEC);
        display.println("°C");
        break;

      case ACCELEROMETER:

        break;

      case COMPASS:
        variable = array[1];
        display.setTextSize(1);
        display.setCursor(0, 1);
        display.print("X: ");
        display.println(variable, DEC);
        variable = array[2];
        display.print("Y: ");
        display.println(variable, DEC);
        variable = array[3];
        display.print("Z: ");
        display.println(variable, DEC);
        break;
    }

  display.display();
}


void changeArmVelocity(int change)
{
  armVelocity += change;

  if (armVelocity <= 0)
    armVelocity = 1;
  else if (armVelocity > 10)
    armVelocity = 10;
}


void showArmVelocity(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Arm change:");
  display.setTextSize(2);
  display.println(armVelocity, DEC);
  display.display();
}


void showStopMode(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Stop mode:");
  display.setTextSize(2);

  if (last_stop_mode == 1)
    display.println("FAST");
  else if (last_stop_mode == 0)
    display.println("NORMAL");

  display.display();
}


void toggleStop(void)
{
  char array[2] = {'s', '0'};
  if (last_stop_mode == 0)
  {
    array[1] = '1';
    softSerial.write(array, 2);
    last_stop_mode = 1;
  }
  else if (last_stop_mode == 1)
  {
    array[1] = '0';
    softSerial.write(array, 2);
    last_stop_mode = 0;
  }
}


void showArmMode(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Arm is:");
  display.setTextSize(2);

  if (last_arm_mode == 1)
    display.print("ENABLE");
  else if ( last_arm_mode == 0)
    display.println("DISABLE");

  display.display();
}


void toggleArm(void)
{
  char array[2] = {'h', '0'};
  if (last_arm_mode == 0)
  {
    array[1] = '1';
    softSerial.write(array, 2);
    last_arm_mode = 1;
  }
  else if ( last_arm_mode == 1)
  {
    array[1] = '0';
    softSerial.write(array, 2);
    last_arm_mode = 0;
  }
}


void changeVelocity(int change)
{
  maxVelocity += change;
  if (maxVelocity < 50)
    maxVelocity = 50;
  else if (maxVelocity > 255)
    maxVelocity = 255;
}


void showVelocity(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Velocity:");
  display.setTextSize(2);
  display.println(maxVelocity, DEC);
  display.display();
}


void showMode(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Mode is: ");
  display.setTextSize(2);

  if (last_mode == 'a')
    display.println("AUTO");
  else if ( last_mode == 'm')
    display.println("MANUAL");

  display.display();
}


void showLcdMode(void)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("LCD mode is:");
  display.setTextSize(2);

  if (last_lcd_mode == 0)
    display.println("NORMAL");
  else if ( last_lcd_mode == 1)
    display.println("INVERT");

  display.display();
}


void toggleLcdMode(void)
{
  if (last_lcd_mode == 0)
  {
    display.command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYINVERTED);
    last_lcd_mode = 1;
  }
  else if ( last_lcd_mode == 1)
  {
    display.command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
    last_lcd_mode = 0;
  }
  display.display();
}


void toggleMode(void)
{
  if (last_mode == 'a')
  {
    softSerial.write('m');
    last_mode = 'm';
  }
  else if (last_mode == 'm')
  {
    softSerial.write('a');
    last_mode = 'a';
  }
}


void moveCommand(char Command, byte velocity)
{
  byte dataPacket[2] = {0};
  dataPacket[0] = Command;
  dataPacket[1] = velocity;
  softSerial.write(dataPacket, 2);
  delay(delayRF(2));
}



void setup()
{
  initialize();         // Initializing all the pins
  ConnectToPc();        // Attempting to connect to the PC
  delayScreenRefresh;   // Delaying the time display
}


void loop()
{
  if (isTherePc != 1)                  // Do this routine only if there isn't any connection to the PC
  {
    screenRefresh();                   // Show the time if it is needed
    receiveData(SINGLE);               // Single check if there is any data available in the Serial
    readButton();                      // Check if any of the 4 buttons is pressed

    if (last_arm_mode == 1)            // Do this only if the arm is enabled
      sendArmCommand();                // Send commands to move the arm

    if (last_mode == 'm')              // Do this only if the rover is in manual mode
      sendMoveCommand();               // Send commands to move the rover

  }
  else if (Serial.available() > 0)     // If there is a connection to the PC wait for instructions
  {
    softSerial.write(Serial.read());   // If there is any instruction send it to the rover
    delay(delayRF(1));                 // Delay needed to transmit the
    receiveData(TIME);                 // Check continuously if there is any data on the softwareSerial
  }
}

