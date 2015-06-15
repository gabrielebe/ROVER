#ifndef RADIO_COMAND
#define RADIO_COMAND

// Code in common between Arduino Uno and Mega that specifies instructions
#define GO_STOP       	0
#define GO_FORWARD    	1
#define GO_BACKWARD   	2
#define GO_LEFT       	3
#define GO_RIGHT      	4
#define VOID          	5
#define ARM_UP		6
#define ARM_DOWN	7
#define ARM_LEFT  	8
#define ARM_RIGHT     	9
#define ARM_PICKER    	10
#define ACCELEROMETER  	11
#define COMPASS		12
#define TEMPERATURE     13
#define UP 		14
#define	DOWN 	        15
#define LEFT 	        16
#define RIGHT 	        17

#define TIMEOUT  		5000    // Timeout for the pc detection
#define NUMBER_OF_FUNCTIONS	6

// Pin definitions on Arduino Uno
// Defining the LCD pins
#define RST 	13
#define CE  	12
#define DC  	11
#define DIN 	10
#define CLK  	9

// Defining the pushbutton pins
#define SWITCH_UP    	4       // Pin used to read up button
#define SWITCH_DOWN  	5       // Pin used to read down button
#define SWITCH_LEFT  	6       // Pin used to read left button
#define SWITCH_RIGHT 	3       // Pin used to read right button

// defining the joystick pins
#define SWITCH_PICKER  	2       // Pin used to change the mode
#define joyX1 		A0      // Vertical of first joystick
#define joyY1 		A1      // Horizontal of first joystick
#define joyX2 		A2      // Vertical of second joystick
#define joyY2 		A3      // Horizontal of second joystick

#define TIME   3
#define FIXED  2
#define SINGLE 1
#define REFRESHRATE 2500

#define delayRF(numByte)    (4 + numByte)    // The maximum number of byte is 240
#define delayScreenRefresh  (waitTime = millis())

Adafruit_PCD8544 display = Adafruit_PCD8544(CLK, DIN, DC, CE, RST);    // Here a new display object is being created
SoftwareSerial softSerial(8, 7);                                       // A new software Serial object is being created
RTC_Millis rtc;                                                        // A new time object is being created

// Here are declared some variables whose function is to store the status of some configurations
byte last_mode = 'm';     // m -> rover is in manual mode    a -> rover is in automatic mode
byte last_lcd_mode;       // 0 -> LCD is in normal mode      1 -> LCD is in inverted mode
byte last_arm_mode = 1;   // 0 -> arm is enabled             1 -> arm is disabled
byte last_stop_mode = 0;  // 0 -> stop mode is normal        1 -> stop mode is fast
byte last_sensor = 0;     // This variable is used to store the index of the sensor to display on the LCD
int maxVelocity = 180;    // This is the maximum velocity the robot can reach
int armVelocity = 5;      // This is the maximum movement the arm can do

char index;                               // numbers of byte received
char Command = -1;                        // Command received
char isTherePc = 1;                       // Flag that indicates if the PC is connected
unsigned long waitTime;                   // Variable used for the timing out
int joystickX1_start, joystickY1_start;   // These variable stores the rest condition of the joysticks
int joystickX2_start, joystickY2_start;


/**
 * @brief This function is used to control the PC communication availability
 * @details The function initially send a character to through the serial port and then wait for the echo of the same character.
 * If there isn't any PC response for some time the TIMEOUT fires and set the variable isTherePc to 0. If there is a PC the variable is equal to 1.
 * While the remote is attempting t connect the status is displayed on the LCD.
 *
 */
void ConnectToPc(void);


/**
 * @brief This function is used to receive data from the software serial port
 * @details It initially checks whether there is any data available in the  port buffer, if there is any it starts to copy the data in its dataArray buffer
 * whose length is 10 and is of type char. every time it reads a byte it waits 100 milliseconds in order to give enough time for the transmitter to send another byte.
 * After the wait if there isn't any data in the serial port it get out from the while loop. Every time a byte is read an index variable is increased. After all the
 * readings the dataArray is analyzed depending on the value of the index.
 *
 * @param type it accept a parameter which refers to the type of reading the user wants. it can be SINGLE if it wants to check only once if there is anything
 * receiving buffer or FIXED if the function has to wait until any data is received. There is a third method of displaying data which doesn't freeze the system forever.
 * the TIME type let the user to control any serial communication waiting for a certain amount of time
 * @param If TIME is chosen the user can also send how many milliseconds the function should check and this is given as a second parameter.
 * @return the function returns a pointer to a char array
 */
char *receiveData(char, unsigned int);


/**
 * @brief This function is used to for initializations
 * @details In this function all the pin are initialized together with the serial communications, the display, the rtc and the joystick calibration variables
 *
 */
void initialize(void);


/**
 * @brief Used to refresh the display screen
 * @details This function is used to refresh the screen when new sensor values are transmitted from the rover to show them on the LCD screen
 *
 */
void screenRefresh(void);


/**
 * @brief Used to display time on screen
 * @details This function is used to display the time gathered from the rtc. It displays slight bigger time and hours and underneath seconds are displayed.
 *
 */
void showTime(void);


/**
 * @brief This function is used to read the switch button
 * @details This function evaluate which switch button has been pressed and then recall the menu function to update the menu levels and update the screen.
 * It delays the whole system by 200 milliseconds if any of the switch is pressed.
 *
 */
void readButton(void);


/**
 * @brief Used to control the arm
 * @details This function is used to handle all the commands needed to read the related joystick and then send instructions to the rover.
 * the system is delayed by at maximum of 50 milliseconds and is used to stabilize the arm movement.
 *
 */
void sendArmCommand(void);


/**
 * @brief picker controller
 * @details this function is used to read first the picker pin on the joystick and then send to the rover if the picker needed to be toggled.
 * This function delays by 100 milliseconds if the button is pressed
 */
void readPicker(void);


/**
 * @brief This function is used to control the robot movement
 * @details This function reads first the status of the joystick and maps the value from 0 to 50 and then checks whether these values are different
 * from the rest condition. If they are the correct instruction is send to the rover in order to move it.
 * The maximum delay that this function can create is of less than 10 milliseconds.
 *
 */
void sendMoveCommand(void);


/**
 * @brief Menu handler
 * @details This function is used to create a listed menu. The menu is divided into levels and depending on the level it take care of different task when UP and
 * DOWN instruction are sent. The level changes when it receives LEFT or RIGHT instructions. After changing it's internal condition the updateScreen is called
 * which is needed to update the information displayed on the LCD screen.
 *
 * @param instruction  Based on this parameter the menu changes it's level or do a task. Valid values are: LEFT - RIGHT - UP - DOWN. Every other values are not
 * considered.
 */
void menu_update(char instruction);


/**
 * @brief This function is used to update information on the display based on the level of the menu it is on
 *
 * @param level This indicates which screen should be displayed on the screen.
 */
void updateScreen(char level);


/**
 * @brief This function is used to change the sensor to be shown on the screen
 *
 * @param change It can be UP or DOWN to indicate the change of sensor.
 */
void changeSensor(char change);


/**
 * @brief This function is used to gather sensor values from the rover and display on the LCD
 * @details In order to do the task it first clean the software serial port and then send a command to the rover which should send back values based on the sensor.
 * While it do the task the details are shown on the display. The delay of this function can be limitless because it wait forever if the data is not sent back.
 *
 * @param sensor This is the parameter used to determine which sensor should be displayed on the LCD.
 */
void showSensor(byte sensor);


/**
 * @brief This function is used to change the arm velocity
 *
 * @param change this parameter determines how much the change should be.
 */
void changeArmVelocity(int change);


/**
 * @brief This function shows the actual value of the arm velocity.
 */
void showArmVelocity(void);


/**
 * @brief This function is used to display the stop mode that is being used
 */
void showStopMode(void);


/**
 * @brief This function is used to toggle the stop mode
 * @details In order to set the stop mode two byte are being sent. first is a stop command and the second is how it should be fast or normal.
 */
void toggleStop(void);


/**
 * @brief It is used to show the servo status
 */
void showArmMode(void);


/**
 * @brief It is used to turn on or off the servos
 */
void toggleArm(void);


/**
 * @brief It is used to change the current maximum velocity
 * @param change It is the change needed.
 */
void changeVelocity(int change);


/**
 * @brief It is used to show the current velocity limit.
 */
void showVelocity(void);


/**
 * @brief It is used to show the current mode of function of the rover.
 */
void showMode(void);


/**
 * @brief It is used to show the current LCD mode.
 */
void showLcdMode(void);


/**
 * @brief It is used to switch the LCD from normal to inverted or inverted to normal mode of displaying.
 */
void toggleLcdMode(void);


/**
 * @brief It is used to switch between manual and auto mode
 */
void toggleMode(void);


/**
 * @brief This function is used to send all the two byte commands
 * @details The commands are sent as a single two byte string. It accepts two parameters and stack them together to make the string.
 * The maximum delay it can make is of less then 10.
 *
 * @param Command It is the first byte of the string
 * @param velocity It is the second byte of the string
 */
void moveCommand(char Command, byte velocity);


#endif

