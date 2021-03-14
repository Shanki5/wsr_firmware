#include <AccelStepper.h>

#define STEPPER_COUNT 3
#define shield_en 8

#define XLIMIT 9
#define YLIMIT 10
#define ZLIMIT 11



int YOFFSET = -10;
int ZOFFSET = 130;


AccelStepper YMOTOR(1, 3, 6);
AccelStepper ZMOTOR(1, 4, 7);


void zhome() {
 int initial_zhome = -1;
 //  Set Max Speed and Acceleration of each Steppers at startup for homing
 ZMOTOR.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 ZMOTOR.setAcceleration(100.0);  // Set Acceleration of Stepper
 // Start Homing procedure of Stepper Motor at startup
 while (!digitalRead(ZLIMIT)) {  // Make the Stepper move CCW until the switch is activated
   ZMOTOR.moveTo(initial_zhome);  // Set the position to move to
   initial_zhome--;  // Decrease by 1 for next move if needed
   ZMOTOR.run();  // Start moving the stepper
   delay(5);
 }
 ZMOTOR.setCurrentPosition(0);  // Set the current position as zero for now
 ZMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 ZMOTOR.setAcceleration(50.0);  // Set Acceleration of Stepper
 ZMOTOR.runToNewPosition(ZOFFSET);

}

void yhome() {
 int initial_yhome = 1;
 //  Set Max Speed and Acceleration of each Steppers at startup for homing
 YMOTOR.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 YMOTOR.setAcceleration(100.0);  // Set Acceleration of Stepper
 // Start Homing procedure of Stepper Motor at startup
 while (!digitalRead(YLIMIT)) {  // Make the Stepper move CCW until the switch is activated
   YMOTOR.moveTo(initial_yhome);  // Set the position to move to
   initial_yhome++;  // Decrease by 1 for next move if needed
   YMOTOR.run();  // Start moving the stepper
   delay(5);
 }
 YMOTOR.setCurrentPosition(0);  // Set the current position as zero for now
 YMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 YMOTOR.setAcceleration(50.0);  // Set Acceleration of Stepper
 YMOTOR.runToNewPosition(YOFFSET);

}
void setup(){
  pinMode(shield_en, OUTPUT);
  digitalWrite(shield_en,LOW);
  delay(5);
  pinMode(ZLIMIT, INPUT_PULLUP);
  pinMode(YLIMIT, INPUT_PULLUP);
  zhome();
  yhome();
 
}

void loop(){
  
}
