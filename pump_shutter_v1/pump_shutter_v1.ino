/* pump shutter for Cuk Lab
  by Ilya Vinogradov 12/30/2020

  shutter program for servo shutter operation
  includes serial comm to change/read/flash store servo angles and 
  three states for shutter operation.  Three states are open, 
  closed, and acquire. Last state's shutter operation is via
  external trigger.
*/

#include <Servo.h>
#include <FlashStorage.h>

Servo myservo;  // create servo object to control a servo
int servoPin = 6; //servo pin

// Create a structure for servo motor positions. "valid" variable is set to 
// "true" once structure is filled with actual data for the first time.
typedef struct {
  boolean valid;
  int pos_upper;
  int pos_lower;
} motor_pos;
motor_pos flash_motor_pos;  //reserve portion of memory for copying flash  memory
FlashStorage(my_flash_store, motor_pos);  //reserve portion of flash memory for motor positions
int pos_upper = 60;    // variable to store the upper servo position
int pos_lower = 30;   // variable to store the lower servo position

int buttonStatus = 0;    // digital sensor
int buttonPin = 5;       // digital sensor pin

char serialBuffer[10];  // serial command buffer of 10 characters
int inByte = 0; // byte output from serial port
int serialIndex = 0; // counts index of serial buffer
bool readSerialBuffer = false; //decides whether to read serial buffer

void setup() {

  pinMode(buttonPin, INPUT_PULLUP); // sets buttonPin to input with pull up (off = HI)
  
  // reads flash memory for motor positions
  flash_motor_pos = my_flash_store.read();
  // if this is the first read then valid should be false
  if (flash_motor_pos.valid==true) {
    pos_upper = flash_motor_pos.pos_upper;
    pos_lower = flash_motor_pos.pos_lower;
  }

  myservo.write(pos_lower);
  myservo.attach(servoPin);  // attaches the servo on pin 6 to the servo object
  
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {  }
  
}

// read and check angle from serial string
int getAngle(char angle_str[], int *angle) {
  int angleOK = -1; // -1 is used as a flag that the angle was not valid
  int pos_tmp = 0;

  //check that first character is a number between 0 and 9
  if ((angle_str[0] >= '0') && (angle_str[0] <= '9')) {
    pos_tmp = atoi(angle_str);  //extract number from string
    //check that the number is between 0 and 180 (servo limits)
    if ((pos_tmp >=0) && (pos_tmp <= 180)) {  
      *angle = pos_tmp;
      angleOK = pos_tmp;  //remove flag for bad angle
    }
  }

  // -1 is used as a flag that the angle was not valid
  if (angleOK < 0)  {
    Serial.println("Invalid angle");
  }
  
  return angleOK;
}

// loops through this code under normal operation
void loop() {
  // return;
  // First, handle serial input and storing into buffer
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();

    // write the incoming byte to the serial buffer
    if (serialIndex < (sizeof(serialBuffer)/sizeof(serialBuffer[0])-1)) {
        serialBuffer[serialIndex] = inByte; // set byte into serialBuffer
    } else  {
        serialBuffer[serialIndex] = 10; // byte value for new line which triggers reading the buffer
    }

    // when new line byte is encountered, toggle reading of serial buffer
    if (serialBuffer[serialIndex] == 10) {
      readSerialBuffer = true;
    }

    serialIndex++;  //increment serial buffer index every time serial is read
    
  }

  // Second, handle interpretation of serial buffer
  if (readSerialBuffer == true) {
    switch (serialBuffer[0])  {
      case 's':
        switch (serialBuffer[1])  {
          case 'l': //set lower servo angle
            getAngle(&serialBuffer[2], &pos_lower);
            break;
            
          case 'u': //set upper servo angle
            getAngle(&serialBuffer[2], &pos_upper);
        }
        break;
        
//      case 'm':
//
//      case 'r':

      case 'f': // read/write servo angles to flash
        switch (serialBuffer[1]) {
          case 'w': //write to flash
            //update values in struct that will be sent to flash
            flash_motor_pos.valid = true; //flag that flash has been written to
            flash_motor_pos.pos_upper = pos_upper;
            flash_motor_pos.pos_lower = pos_lower;

            //write struct to flash
            my_flash_store.write(flash_motor_pos);
            break;
            
          case 'r': //read to flash
            // reads flash memory for motor positions
            flash_motor_pos = my_flash_store.read();
            // if this is the first read then valid should be false and there is nothing to read
            if (flash_motor_pos.valid==true) {
              pos_upper = flash_motor_pos.pos_upper;
              pos_lower = flash_motor_pos.pos_lower;
            }
            break;
            
          default:
            Serial.println("Invalid flash command");
        }
        break;
      
      default:
        Serial.println("Invalid command");
    }
    
    Serial.write(serialBuffer,serialIndex);
    serialIndex = 0; // reset serial buffer
    readSerialBuffer = false;  
  }
  
  buttonStatus = digitalRead(buttonPin);

  // Third manage state of Arduino and shutter operation
  if (buttonStatus==1) {
    myservo.write(pos_lower);
    
  } else {
    myservo.write(pos_upper);
  }

}
