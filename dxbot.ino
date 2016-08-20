
#include <Servo.h>
#include "IRLremote.h"
#include "PinChangeInterrupt.h"

//pin defines
#define ONBOARD_LED 13
#define MOTOR_A_ENABLE 6
#define MOTOR_A_IN_1 7
#define MOTOR_A_IN_2 8
#define MOTOR_B_IN_1 10
#define MOTOR_B_IN_2 9
#define MOTOR_B_ENABLE 11
#define SERVO_PIN 5

//IR code defines
#define IR_FWD 0x81110090
#define IR_RWD 0x87170090
#define  IR_LEFT 0x83130090
#define IR_RIGHT 0x85150090
#define IR_LEFT_FWD 0x80100090
#define IR_RIGHT_FWD 0x82120090
#define IR_LEFT_RWD 0x86160090
#define IR_RIGHT_RWD 0x88180090
#define IR_SLOW 0x96060090
#define IR_MED 0x9F0F0090
#define IR_FAST 0x98080090

Servo myservo;

//default motor speeds
uint8_t motor_a_speed = 255;
uint8_t motor_b_speed = 255;
uint8_t base_speed = 255;

//command time out, this keeps track of when the last command was recieved so it knows when to shut off the motors
uint32_t last_command_time = 0;

// do you want to block until the last data is received
// or do you always want to update the data to the newest input
#define IRL_BLOCKING true

// choose a valid PinChangeInterrupt pin of your Arduino board
#define pinIR 3

// temporary variables to save latest IR input
uint8_t IRProtocol = 0;
uint16_t IRAddress = 0;
uint32_t IRCommand = 0;

void setup() {
  // start serial debug output
  Serial.begin(115200);
  Serial.println("Startup");

  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(MOTOR_A_ENABLE, OUTPUT);
  pinMode(MOTOR_A_IN_1, OUTPUT);
  pinMode(MOTOR_A_IN_2, OUTPUT);
  pinMode(MOTOR_B_IN_1, OUTPUT);
  pinMode(MOTOR_B_IN_2, OUTPUT);
  pinMode(MOTOR_B_ENABLE, OUTPUT);

  myservo.attach(SERVO_PIN);

  digitalWrite(ONBOARD_LED, LOW);
  digitalWrite(MOTOR_A_ENABLE, LOW);
  digitalWrite(MOTOR_A_IN_1, LOW);
  digitalWrite(MOTOR_A_IN_2, LOW);
  digitalWrite(MOTOR_B_IN_1, LOW);
  digitalWrite(MOTOR_B_IN_2, LOW);
  digitalWrite(MOTOR_B_ENABLE, LOW);

  analogWrite(SERVO_PIN, 127);
  // choose your protocol here to reduce flash/ram/performance
  // see readme for more information
  attachPCINT(digitalPinToPCINT(pinIR), IRLinterrupt<IR_ALL>, CHANGE);
}

void loop() {
  // temporary disable interrupts and print newest input
  uint8_t oldSREG = SREG;
  cli();
  if (IRProtocol) {
    // print as much as you want in this function
    // see readme to terminate what number is for each protocol
    Serial.println();
    Serial.print("Protocol:");
    Serial.println(IRProtocol);
    Serial.print("Address:");
    Serial.println(IRAddress, HEX);
    Serial.print("Command:");
    Serial.println(IRCommand, HEX);


    if (IRProtocol == 4 && IRAddress == 0x2002) {
      Serial.println("match");
      if (IRCommand == IR_FWD ) {
        Serial.println("fwd");
        digitalWrite(MOTOR_A_IN_1, HIGH);
        digitalWrite(MOTOR_A_IN_2, LOW);
        digitalWrite(MOTOR_B_IN_1, HIGH);
        digitalWrite(MOTOR_B_IN_2, LOW);
        motor_a_speed = base_speed; //use stored speed
        motor_b_speed = base_speed;
        myservo.write(90);
        last_command_time = millis();
      }
      else if (IRCommand == IR_RWD) {
        digitalWrite(MOTOR_A_IN_1, LOW);
        digitalWrite(MOTOR_A_IN_2, HIGH);
        digitalWrite(MOTOR_B_IN_1, LOW);
        digitalWrite(MOTOR_B_IN_2, HIGH);
        motor_a_speed = base_speed; //use stored speed
        motor_b_speed = base_speed;
        myservo.write(90);
        last_command_time = millis();
      } else if (IRCommand == IR_LEFT) {
        digitalWrite(MOTOR_A_IN_1, LOW);
        digitalWrite(MOTOR_A_IN_2, HIGH);
        digitalWrite(MOTOR_B_IN_1, HIGH);
        digitalWrite(MOTOR_B_IN_2, LOW);
        motor_a_speed = 255; //hardcoded max speed for spin in place, otherwise it cant move
        motor_b_speed = 255;
        myservo.write(190);
        last_command_time = millis();
      } else if (IRCommand == IR_RIGHT ) {
        digitalWrite(MOTOR_A_IN_1, HIGH);
        digitalWrite(MOTOR_A_IN_2, LOW);
        digitalWrite(MOTOR_B_IN_1, LOW);
        digitalWrite(MOTOR_B_IN_2, HIGH);
        motor_a_speed = 255; //hardcoded max speed for spin in place, otherwise it cant move
        motor_b_speed = 255;
        myservo.write(30);
        last_command_time = millis();
      } else if (IRCommand == IR_LEFT_FWD ) {
        digitalWrite(MOTOR_A_IN_1, HIGH);
        digitalWrite(MOTOR_A_IN_2, LOW);
        digitalWrite(MOTOR_B_IN_1, HIGH);
        digitalWrite(MOTOR_B_IN_2, LOW);
        motor_a_speed = base_speed >> 2; //hardcoded division by 4 to approximate fwd and turning
        motor_b_speed = base_speed;
        myservo.write(120);
        last_command_time = millis();
      } else if (IRCommand ==  IR_RIGHT_FWD) {
        digitalWrite(MOTOR_A_IN_1, HIGH);
        digitalWrite(MOTOR_A_IN_2, LOW);
        digitalWrite(MOTOR_B_IN_1, HIGH);
        digitalWrite(MOTOR_B_IN_2, LOW);
        motor_a_speed = base_speed ;
        motor_b_speed = base_speed >> 2;  //hardcoded division by 4 to approximate fwd and turning
        myservo.write(60);
        last_command_time = millis();
      } else if (IRCommand == IR_LEFT_RWD) {
        digitalWrite(MOTOR_A_IN_1, LOW);
        digitalWrite(MOTOR_A_IN_2, HIGH);
        digitalWrite(MOTOR_B_IN_1, LOW);
        digitalWrite(MOTOR_B_IN_2, HIGH);
        myservo.write(90);
        motor_a_speed = base_speed >> 2;  //hardcoded division by 4 to approximate rwd and turning
        motor_b_speed = base_speed ;
        last_command_time = millis();
      } else if (IRCommand == IR_RIGHT_RWD) { //right rwd
        digitalWrite(MOTOR_A_IN_1, LOW);
        digitalWrite(MOTOR_A_IN_2, HIGH);
        digitalWrite(MOTOR_B_IN_1, LOW);
        digitalWrite(MOTOR_B_IN_2, HIGH);
        myservo.write(90);
        motor_a_speed = base_speed;
        motor_b_speed = base_speed >> 2;  //hardcoded division by 4 to approximate rwd and turning
        last_command_time = millis();
      } else if (IRCommand ==  IR_SLOW) base_speed = 128;
      else if (IRCommand == IR_MED)     base_speed = 192;
      else if (IRCommand == IR_FAST)    base_speed = 255;
    }

    // reset variable to not read the same value twice
    IRProtocol = 0;
  }
  SREG = oldSREG;



  if (millis() - last_command_time < 200) { //check to see if button is held down
    analogWrite(MOTOR_A_ENABLE, motor_a_speed);
    analogWrite(MOTOR_B_ENABLE, motor_b_speed);
  } else { //otherwise motor off
    analogWrite(MOTOR_A_ENABLE, 0);
    analogWrite(MOTOR_B_ENABLE, 0);
  }
}

void IREvent(uint8_t protocol, uint16_t address, uint32_t command) {
  // called when directly received a valid IR signal.
  // do not use Serial inside, it can crash your program!

  // dont update value if blocking is enabled
  if (IRL_BLOCKING && !IRProtocol) {
    // update the values to the newest valid input
    IRProtocol = protocol;
    IRAddress = address;
    IRCommand = command;

  }
}


