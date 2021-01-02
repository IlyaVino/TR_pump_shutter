/* pump shutter for Cuk Lab
  by Ilya Vinogradov 12/30/2020

  shutter program for servo shutter operation
  includes serial comm to change/read/flash store servo angles and 
  three states for shutter operation.  Three states are open, 
  closed, and acquire. Last state's shutter operation is via
  external trigger.

  To do:
  1) Add trigger code from mode 2 (triggerd shutter operation)
  2) Clean up code in loop and move to functions
  3) Reduce number of global variables
  4) fix variable names to be more clear
*/

#include <Servo.h>
#include <FlashStorage.h>

Servo myservo;  // create servo object to control a servo
int servoPin = 6; //servo pin

// Create a structure for servo motor positions. "valid" variable is set to 
// "true" once structure is filled with actual data for the first time.
typedef struct {
  boolean valid;
  int u;
  int l;
} motor_pos;
motor_pos flash_motor_pos;  //reserve portion of memory for copying flash  memory
FlashStorage(my_flash_store, motor_pos);  //reserve portion of flash memory for motor positions
int u = 60;    // variable to store the upper servo position
int l = 30;   // variable to store the lower servo position

//button is used to switch between shutter open/closed state (mode = 1 or 0)

int buttonPin = 5;       // digital sensor pin
int shutterMode = 0;  //keeps track of button mode. 0 = closed, 1 = open, 2 = trigger

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
    u = flash_motor_pos.u;
    l = flash_motor_pos.l;
  }

  myservo.write(l); //attach pin takes some time, this avoids switching while writing
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

// remove button bounce artifact by checking button status 4 consecutive times
// returns button status. -1 is flag for unknown status
int buttonDebounce()  {
  #define DEBUND_BITS 0b111111  //register length for button off
  static unsigned int b = 0;  // register to keep track of true button state in case button press has ringing
  
  // this updates button status into button register (used to avoid ringing error)
  b = b << 1; //shift button register to left
  b = (b & ~1) | digitalRead(buttonPin); //clear first register and populate with button status
  
  if ((b & DEBUND_BITS) == DEBUND_BITS) { //if button is HIGH register length times in a row
    return 1;  
  }
  if ((b & DEBUND_BITS) == 0) { //if button is LOW register length times in a row
    return 0;
  }
  return -1;  //happens when register is mixed HIGH and LOW
}

// switches mode for shutter opperation upon button press and keeps track of old button status
void buttonPressSwitchState()  {
  static int bOldState = 1;
  int bNewState = buttonDebounce();
  if (bNewState != -1) {
    if ((bNewState == 0) && (bOldState == 1)) {
      switch (shutterMode)  {
        case 0:
          shutterMode = 1;
          break;
        case 1:
          shutterMode = 0;
          break;
      }
    }
    
    bOldState = bNewState;
  }

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
            getAngle(&serialBuffer[2], &l);
            break;
            
          case 'u': //set upper servo angle
            getAngle(&serialBuffer[2], &u);
            break;
        }
        break;

      //set mode of operation
      case 'm': 
        switch (serialBuffer[1])  {
          case '0': //shutter closed
            shutterMode = 0;
            break;
          case '1': //shutter open
            shutterMode = 1;
            break;
          case '2': //shutter triggered operation
            shutterMode = 2;
            break;
          default:
            Serial.println("Invalid mode command");  
        }
        break;
        
      case 'r': //read to serial port current state:
        switch (serialBuffer[1])  {
          case 'm': //read mode
            Serial.println(shutterMode); 
            break;
          case 'l': //read lower angle
            Serial.println(l); 
            break;
          case 'u': //read upper angle
            Serial.println(u); 
            break;
          default:
            Serial.println("Invalid read command");
        }
        break;

      case 'f': // read/write servo angles to flash
        switch (serialBuffer[1]) {
          case 'w': //write to flash
            //update values in struct that will be sent to flash
            flash_motor_pos.valid = true; //flag that flash has been written to
            flash_motor_pos.u = u;
            flash_motor_pos.l = l;

            //write struct to flash
            my_flash_store.write(flash_motor_pos);
            break;
            
          case 'r': //read to flash
            // reads flash memory for motor positions
            flash_motor_pos = my_flash_store.read();
            // if this is the first read then valid should be false and there is nothing to read
            if (flash_motor_pos.valid==true) {
              u = flash_motor_pos.u;
              l = flash_motor_pos.l;
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

  buttonPressSwitchState(); //if button is pressed undergo state swich if state is 0 or 1
  
  // Third manage state of Arduino and shutter operation
  switch (shutterMode)  {
    case 0:
      myservo.write(l);
      break;
    case 1:
      myservo.write(u);
      break;
    //case 2:
    //;
      //put trigger code here
  }
}
