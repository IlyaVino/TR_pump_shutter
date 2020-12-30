/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int servoPin = 6; //servo pin
int pos_upper = 60;    // variable to store the upper servo position
int pos_lower = 0;   // variable to store the lower servo position

int buttonStatus = 0;    // digital sensor
int buttonPin = 5;       // digital sensor pin

char serialBuffer[10];  // serial command buffer of 10 characters
int inByte = 0; // byte output from serial port
int serialIndex = 0; // counts index of serial buffer
bool readSerialBuffer = false; //decides whether to read serial buffer

void setup() {
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  myservo.attach(servoPin);  // attaches the servo on pin 6 to the servo object

  pinMode(buttonPin, INPUT_PULLUP); // sets buttonPin to input with pull up (off = HI)
}

int getAngle(char angle_str[], int *angle) {
  int angleOK = -1;
  int pos_tmp = 0;

  if ((angle_str[0] >= '0') && (angle_str[0] <= '9')) {
    pos_tmp = atoi(angle_str);
    if ((pos_tmp >=0) && (pos_tmp <= 180)) {
      *angle = pos_tmp;
      angleOK = pos_tmp;
    }
  }
  
  if (angleOK < 0)  {
    Serial.println("Invalid angle");
  }
  
  return angleOK;
}

void loop() {

  // First handle serial input and storing into buffer
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
    
    serialIndex++;
    
  }

  // Second handle interpretation of serial buffer
  if (readSerialBuffer == true) {
    switch (serialBuffer[0])  {
      case 'l': //set lower servo angle
        getAngle(&serialBuffer[1], &pos_lower);
        break;
      case 'u':
        getAngle(&serialBuffer[1], &pos_upper);
        break;
//      case 'm':
//
//      case 'r':
      
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
