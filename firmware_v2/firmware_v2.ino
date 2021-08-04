//include necessary libraries
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/String.h>


//define names for pins
#define shield_en 8

#define STEPPER_COUNT 3

#define XLIMIT 9
#define YLIMIT 10
#define ZLIMIT 11


//offset values for homing sequence
#define YOFFSET -50
#define YOFFSET2 -60
#define ZOFFSET 0
#define XOFFSET -10

// new target available for each stepper
boolean stepperNewTarget[STEPPER_COUNT] = {false, false, false};

boolean Xmoving = false, Ymoving = false, Zmoving = false;

// current stepper target for each stepper
int stepperTarget[STEPPER_COUNT] = {0, 0, 0};

//stepper initialization
AccelStepper XMOTOR(1, 2, 5);
AccelStepper YMOTOR(1, 3, 6);
AccelStepper ZMOTOR(1, 4, 7);

ros::NodeHandle nh;

//ros callback function
void motors_cb(const std_msgs::String& cmd_msg) {
  int f = 0;
  double angle;
  char* token = strtok((char*)cmd_msg.data, ",");
  while(token != NULL)
  {
    angle = atof(token);
    if(f == 3)
    {
      if(angle == 1.0)
      {
        digitalWrite(13,HIGH);
      }
      else
      {
        digitalWrite(13,LOW);
      }
    }
    else if(angle != stepperTarget[f])
    {
      stepperTarget[f] = angle;
      stepperNewTarget[f] = true;
    }
    char result[8]; // Buffer big enough for 7-character float
    char log_msg[10];
    dtostrf(angle, 6, 2, result); // Leave room for too large numbers!
    sprintf(log_msg,"angle =%s", result);
    nh.loginfo(log_msg);
    f++;
    token = strtok(NULL, ",");
  }
//  for(f=0; f < STEPPER_COUNT; f++) {
//    
//    angle = cmd_msg.data[f];
//    if(angle != stepperTarget[f]) {
//      stepperTarget[f] = angle;
//      stepperNewTarget[f] = true;  
//      char result[8]; // Buffer big enough for 7-character float
//      char log_msg[10];
//      dtostrf(angle, 6, 2, result); // Leave room for too large numbers!
//      sprintf(log_msg,"angle =%s", result);
//      nh.loginfo(log_msg);
//    }    
//  }
}




// Subscriber node declaration, specifies the topic to which subscribe and the callback funtion
ros::Subscriber<std_msgs::String> sub("joint_states", motors_cb);


std_msgs::String controller_status_msg;
ros::Publisher control_status_pub("controller_status", &controller_status_msg);


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
 YMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 YMOTOR.setAcceleration(50.0);  // Set Acceleration of Stepper
 XMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 XMOTOR.setAcceleration(50.0);
 // Start Homing procedure of Stepper Motor at startup
 while (!digitalRead(YLIMIT)) {  // Make the Stepper move CCW until the switch is activated
   XMOTOR.moveTo(initial_yhome);
   YMOTOR.moveTo(initial_yhome);  // Set the position to move to
   initial_yhome++;  // Decrease by 1 for next move if needed
   XMOTOR.run();
   YMOTOR.run();  // Start moving the stepper
   delay(5);
 }
 YMOTOR.setCurrentPosition(0);  // Set the current position as zero for now
 XMOTOR.setCurrentPosition(0);
 YMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 YMOTOR.setAcceleration(50.0);  // Set Acceleration of Stepper
 YMOTOR.runToNewPosition(YOFFSET);
 YMOTOR.setCurrentPosition(0);

}

void xhome() {
 int initial_xhome = 1;
 //  Set Max Speed and Acceleration of each Steppers at startup for homing
 XMOTOR.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 XMOTOR.setAcceleration(100.0);  // Set Acceleration of Stepper
 YMOTOR.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 YMOTOR.setAcceleration(100.0);
 // Start Homing procedure of Stepper Motor at startup
 while (!digitalRead(XLIMIT)) {  // Make the Stepper move CCW until the switch is activated
   XMOTOR.moveTo(initial_xhome);  // Set the position to move to
   YMOTOR.moveTo(-initial_xhome);
   initial_xhome++;  // Decrease by 1 for next move if needed
   XMOTOR.run(); // Start moving the stepper
   YMOTOR.run();
   delay(5);
 }
 XMOTOR.setCurrentPosition(0);  // Set the current position as zero for now
 YMOTOR.setCurrentPosition(0);
 XMOTOR.setMaxSpeed(50.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
 XMOTOR.setAcceleration(50.0);  // Set Acceleration of Stepper
 XMOTOR.runToNewPosition(XOFFSET);
 YMOTOR.runToNewPosition(YOFFSET2);

}
void setup(){


  
  pinMode(shield_en, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(shield_en,LOW);

  pinMode(ZLIMIT, INPUT_PULLUP);
  pinMode(YLIMIT, INPUT_PULLUP);
  pinMode(XLIMIT, INPUT_PULLUP);

  delay(5);
  zhome();
  yhome();
  xhome();
  XMOTOR.setCurrentPosition(255);
  YMOTOR.setCurrentPosition(-510);
  // ros setup
  nh.getHardware()->setBaud(115200); 
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(control_status_pub);
  
}

void processX(){
  if(stepperNewTarget[0]){
    XMOTOR.moveTo(stepperTarget[0]);
    XMOTOR.setMaxSpeed(1000);
    XMOTOR.setSpeed(50);
    Xmoving = true;
    stepperNewTarget[0] = false;
  }

  if(Xmoving){
    XMOTOR.runSpeedToPosition();
    if(XMOTOR.currentPosition() == stepperTarget[0]){
      Xmoving = false;
    }
  }
}

void processY(){
  if(stepperNewTarget[1]){
    YMOTOR.moveTo(stepperTarget[1]);
    YMOTOR.setMaxSpeed(1000);
    YMOTOR.setSpeed(50);
    Ymoving = true;
    stepperNewTarget[1] = false;
  }

  if(Ymoving){
    YMOTOR.runSpeedToPosition();
    if(YMOTOR.currentPosition() == stepperTarget[1]){
      Ymoving = false;
    }
  }
}

void processZ(){
  if(stepperNewTarget[2]){
    ZMOTOR.moveTo(stepperTarget[2]);
    ZMOTOR.setMaxSpeed(1000);
    ZMOTOR.setSpeed(50);
    Zmoving = true;
    stepperNewTarget[2] = false;
  }

  if(Zmoving){
    ZMOTOR.runSpeedToPosition();
    if(ZMOTOR.currentPosition() == stepperTarget[2]){
      Zmoving = false;
    }
  }
}

void loop(){
  processX();
  processY();
  processZ();
  if(Xmoving || Ymoving || Zmoving)
  {
    controller_status_msg.data = "1"; 
  }
  else
  {
    controller_status_msg.data = "0";   
  }
  control_status_pub.publish(&controller_status_msg);
  nh.spinOnce();
  delay(10);  
}
