/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules
  
 Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>

#include "BluefruitConfig.h"
#include "piezo.h"
#include "lights.h"

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(3);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(4);

//Name your RC here
String BROADCAST_NAME = "The Mogo Ranger";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

int robotSpeed = 100;
uint8_t maxSpeed = 100;


/***---------- Module Defines ----------***/
// Set your forward, reverse, and turning speeds
#define ForwardSpeed                200
#define ReverseSpeed                200
#define TurningSpeed                100

// Thresholds for using quaternions and angles
#define YAW_THRESHOLD           10
#define HIGH_ROLL_THRESHOLD     15
#define LOW_ROLL_THRESHOLD      -15
#define MH_ROLL_THRESHOLD       10
#define ML_ROLL_THRESHOLD       -10
#define HIGH_PITCH_THRESHOLD    10
#define LOW_PITCH_THRESHOLD     6
#define QUAT_INTERVAL           250


// Enumeration of possible robot states
typedef enum {
    FWD, REV, TURN_FWD, TURN_REV, TURN_STOP, STOP
} RobotState;

// Enumeration of possible light states
typedef enum {
    FULL,VAR, OFF
} LightsState;

/***---------- Module Variables ----------***/
//piezo control bool
boolean isPlaying = false;

//lights control bool
boolean lightsOn = false;

//use quartenions or buttons
boolean useQuat = false;
double orientation[3];

//time stamps
unsigned long curr_time = 0;
unsigned long prev_time = 0;

static RobotState robotState;
static LightsState lightsState;


/***---------- Function Declarations ----------***/
void BLEsetup();
void checkGeneral();
void handleAcceleration();
void handleDeceleration();
void controlSpeed();
void balanceSpeed();
void turnRight();
void turnLeft();


/**************************************************************************/
/*!
    @brief  Sets up the HW and the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //Serial.begin(9600);

  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialize the module */
  BLEsetup();

  //Setup Piezo
  setupPiezo();

  //Robot Initial State
  robotState = STOP;

  //Lights Initial State
  lightsState = OFF;
}


/***** LOOP *****/
void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  readController();

  checkGeneral();
    
}


// check other stuff
void checkGeneral() {
  switch (robotState) {
    case FWD: case REV:
      break;
    case TURN_FWD: case TURN_REV: case TURN_STOP:
      blinker();
      break;
    case STOP: 
      if(isPlaying) {
        if(playTune()) {isPlaying = false;}
      }
      break;                  
  }
}

// read commands
bool readController(){

 // Buttons
  if (packetbuffer[1] == 'B') {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    if (pressed) {
//      Serial.println("button pressed");
      if(buttnum == 1) {
        useQuat = false;
        robotState = STOP;
        maxSpeed = 0;
        controlSpeed();
        L_MOTOR->run(RELEASE);
        R_MOTOR->run(RELEASE);
      }
      
      if(buttnum == 2){
        switch (robotState) {
          case FWD: case TURN_FWD:
            honk();
            break;
          case REV: case TURN_REV:
            honk();
            break;
          case STOP: case TURN_STOP:
            isPlaying = !isPlaying;
            break;                   
        }
      }

      if(buttnum == 3){
        // toggles the use of quaternions
        useQuat = !useQuat;
        Serial.println("Quaternion Toggled");
      }

      if(buttnum == 4){
        toggleLights();
      }
      
      //up
      if(buttnum == 5){
        switch (robotState) {        
          case FWD:
            handleAcceleration();
            break;
          case REV:
            handleDeceleration();
            break;
          case STOP:
            robotState = FWD;
            L_MOTOR->run(FORWARD);
            R_MOTOR->run(FORWARD);
            isPlaying = false; 
            break;
          default: break;                  
        }
      }

      //down
      if(buttnum == 6){
        switch (robotState) {
          case FWD:
            handleDeceleration();
            break;
          case REV:
            handleAcceleration();
            break;
          case STOP:
            robotState = REV;
            L_MOTOR->run(BACKWARD);
            R_MOTOR->run(BACKWARD);
            isPlaying = false;
            break;
          default:  break;                 
        }
      }

      //left
      if(buttnum == 7){
        switch (robotState) {
          case FWD:
            robotState = TURN_FWD;
            turnLeft();
            break;
          case REV:
            robotState = TURN_REV;
            turnLeft();
            break;
          case TURN_FWD: case TURN_REV:
            turnLeft();
            break;
          case STOP:
            robotState = TURN_STOP;
            L_MOTOR->run(RELEASE);
            R_MOTOR->run(FORWARD);
            maxSpeed = TurningSpeed;
            break;                   
        }
        // indicate left
        setBlinkDirection(0); 
      }

      //right
      if(buttnum == 8){
        switch (robotState) {
          case FWD:
            robotState = TURN_FWD;
            turnRight();
            break;
          case REV:
            robotState = TURN_REV;
            turnRight();
            break;
          case TURN_FWD: case TURN_REV:
            turnRight();
            break;
          case STOP:
            robotState = TURN_STOP;
            L_MOTOR->run(FORWARD);
            R_MOTOR->run(RELEASE);
            maxSpeed = TurningSpeed;
            break;                   
        }
        //indicate right
        setBlinkDirection(1);        
      }

      controlSpeed();
      
    } else {
      //when no button is pressed
      switch (robotState) {
        case TURN_FWD:
          balanceSpeed();
          robotState = FWD;
          restoreLights();
          break;        
        case TURN_REV:
          balanceSpeed();
          robotState = REV;
          restoreLights();
          break;
        case TURN_STOP:
          maxSpeed = 0;
          controlSpeed();
          L_MOTOR->run(RELEASE);
          R_MOTOR->run(RELEASE);
          robotState = STOP;
          restoreLights();
          break;
        default:  break;                 
      }        
    }

    
  } else if (packetbuffer[1] == 'Q' && useQuat && (millis()-prev_time > QUAT_INTERVAL) ) {
    findOrientation();
    driveRover();
//    delay(50);
    prev_time = millis();
  }
}


// Increase the speed of the motors
void handleAcceleration() {
  if (maxSpeed <= 235) {
    maxSpeed = maxSpeed + 20;
  }  
}

// Decrease the speed of the motors
void handleDeceleration() {
  if (maxSpeed >= 20) {
    maxSpeed = maxSpeed - 20;
  }
  if (maxSpeed <= 0) {
    robotState = STOP;  
  }
}


// Change the speed of the motors to the desired level
void controlSpeed() {
  if(maxSpeed > robotSpeed) {
    // speed up the motors
    for (int speed = robotSpeed; speed < maxSpeed; speed+=5) {
      L_MOTOR->setSpeed(speed);
      R_MOTOR->setSpeed(speed);
      delay(5); // 250ms total to speed up
    }
  } else {
    //slow down the motors
    for (int speed = robotSpeed; speed > maxSpeed; speed-=5) {
      L_MOTOR->setSpeed(speed);
      R_MOTOR->setSpeed(speed);
      delay(5); // 50ms total to slow down
    }
  }
  robotSpeed = maxSpeed;
}


// Turn the robot right
void turnRight() {
  L_MOTOR->setSpeed(min(255, robotSpeed + 30));
  R_MOTOR->setSpeed(max(0, robotSpeed - 30));
}

// Turn the robot left
void turnLeft() {
  L_MOTOR->setSpeed(max(0, robotSpeed - 30));
  R_MOTOR->setSpeed(min(255, robotSpeed + 30));
}


// Balance the speeds after a turn
void balanceSpeed() {
  L_MOTOR->setSpeed(robotSpeed);
  R_MOTOR->setSpeed(robotSpeed);
}


// toggle the light mode
void toggleLights() {
  switch (lightsState) {
    case OFF:
      lightsOnFull();
      lightsState = FULL;
      break;
    case FULL:
      lightsOff();
      lightsState = OFF;
      break;
    case VAR:
      lightsState = FULL;
      break;
  }
}


// Restore Lights after blinking
void restoreLights() {
  switch (lightsState) {
    case OFF:
      lightsOff();
      break;
    case FULL:
      lightsOnFull();
      break;
    case VAR:
      break;
  }
}


// Finds the device pitch, yaw and roll to control rover
void findOrientation() {
//  Serial.println("Quaternion Data:");
  
  float x = *( (float*)(packetbuffer + 2) );
//  Serial.print("x = ");
//  Serial.println(x, 7);
  

  float y = *( (float*)(packetbuffer + 6) );
//  Serial.print("y = ");
//  Serial.println(y, 7);

  float z = *( (float*)(packetbuffer + 10) );
//  Serial.print("z = ");
//  Serial.println(z, 7); 

  float w = *( (float*)(packetbuffer + 14) );
//  Serial.print("w = ");
//  Serial.println(w, 7); 

  double yaw = atan2(2*z*w - 2*y*x , 1 - 2*pow(z,2) - 2*pow(x,2));
  yaw = yaw * 180 / PI;
//  Serial.print("yaw = ");
//  Serial.println(yaw, 7); 

  double pitch = asin(2*y*z + 2*x*w);
  pitch = pitch * 180 / PI;
//  Serial.print("pitch = ");
//  Serial.println(pitch, 7); 

  double roll = atan2(2*y*w - 2*z*x , 1 - 2*pow(y,2) - 2*pow(x,2));
  roll = roll * 180 / PI;
//  Serial.print("roll = ");
//  Serial.println(roll, 7);

  orientation[1] = yaw;
  orientation[2] = pitch;
  orientation[3] = roll;
}

void driveRover() {
  controlLongitudinal();
  controlLateral();

}


// control forward and reverse by using roll angle
void controlLongitudinal() {
  int roll = (int) round(orientation[3]);

//  Serial.print("roll = ");
//  Serial.println(roll); 

  // sets the robot state depending on the roll angle
  if (roll > HIGH_ROLL_THRESHOLD) {
    robotState = FWD;
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(FORWARD);
//    Serial.println("Forward");
  } else if (roll < LOW_ROLL_THRESHOLD) {
    robotState = REV;
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(BACKWARD);
//    Serial.println("Reverse");
  } else if (roll > ML_ROLL_THRESHOLD && roll < MH_ROLL_THRESHOLD) {
    robotState = STOP;
//    Serial.println("Stop");
  }

  // sets the robot speed depending on the state and roll angle
  switch (robotState) {
    case FWD:
      roll = min(90, roll);
      maxSpeed = map(roll, 0, 90, 0, 255);
      break;
    case REV:
      roll = abs(max(-90, roll));
      maxSpeed = map(roll, 0, 90, 0, 255);
      break;
    case STOP:
      maxSpeed = 0;
      controlSpeed();
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE); 
      break;
  }

//  Serial.println(maxSpeed);

  controlSpeed();

}

// control left and right by using yaw angle
void controlLateral() {
  int pitch = (int) round(orientation[2]);
  
//  Serial.print("pitch = ");
//  Serial.println(pitch);

  switch (robotState) {
    case FWD: case REV:
      if (abs(pitch) > HIGH_PITCH_THRESHOLD) {
        if(pitch > 0) {
          turnLeft();
        } else {
          turnRight();
        }
      }
      break;
    case STOP:
      if (abs(pitch) > HIGH_PITCH_THRESHOLD) {
        if(pitch > 0) {
          L_MOTOR->run(RELEASE);
          R_MOTOR->run(FORWARD);
        } else {
          L_MOTOR->run(FORWARD);
          R_MOTOR->run(RELEASE);
        }
        maxSpeed = TurningSpeed;                  
      }
      break;
    default: break;
  }
}


void BLEsetup(){
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  //Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);

  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);

  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("*****************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("*****************"));
}


