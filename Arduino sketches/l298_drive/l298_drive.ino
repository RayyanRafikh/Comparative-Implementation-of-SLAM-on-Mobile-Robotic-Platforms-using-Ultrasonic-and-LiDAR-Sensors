/*
Code is based of the implementation here:
https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/
Reference Author: Automatic Addison
*/

#include <L298N.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

// Assigning the pins for the motors
const unsigned int EN_A = 11;
const unsigned int IN1_A = 6;
const unsigned int IN2_A = 7;

const unsigned int EN_B = 10;
const unsigned int IN1_B = 4;
const unsigned int IN2_B = 5;

// Assigning the pins for the encoders
const unsigned int ENC_LEFT_TICKS = 3;
const unsigned int ENC_RIGHT_TICKS = 2;

const unsigned int ENC_LEFT_POS = 9;
const unsigned int ENC_RIGHT_POS = 8;

// Initializing the motor object
L298N motor_l(EN_A, IN1_A, IN2_A);
L298N motor_r(EN_B, IN1_B, IN2_B);

ros::NodeHandle nh;   // Initializing the node handle

//const float a = 0.0325;  // Enter the radius of the wheel here
//const float d = 0.19;  // Enter the Trackwidth(distance between the wheels) here

const float a = 0.0325;
const float d = 0.3;

float vel_x;
float ang_z;
float wl, wr;

float val_l, val_r;

// Define wheel directions: true is forward, false is backward
boolean l_dir = true;
boolean r_dir = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Publishers for wheel tick counts
std_msgs::Int16 r_tick_count;
std_msgs::Int16 l_tick_count;

ros::Publisher r_tick_pub("r_ticks", &r_tick_count);
ros::Publisher l_tick_pub("l_ticks", &l_tick_count);

// The callback function
void messageCb(const geometry_msgs::Twist& msg)
{
  vel_x = msg.linear.x;
  ang_z = msg.angular.z;

  // Clamping the linear velocity
  vel_x = max(-1, vel_x);
  vel_x = min(1, vel_x);

  // Clamping the angular velocity
  ang_z = max(-1, ang_z);
  ang_z = min(1, ang_z);

  // Inverse kinematics
  wl = (vel_x - (ang_z*d))/a;
  wr = (vel_x + (ang_z*d))/a;

  if(wl == 0 && wr == 0)
  {
    motor_r.stop();  // For stopping the motors
    motor_l.stop();
  }
  else
  {
    val_l = map(abs(wl), 0, 37, 75, 255);  // Scaling wheel velocities to (-255,255)
    val_r = map(abs(wr), 0, 37, 75, 255);

    motor_l.setSpeed(val_l);       // Output PWM signals to motor driver
    motor_r.setSpeed(val_r);

    if(wl > 0)motor_l.forward();     // Setting the appropriate directions
    else motor_l.backward();
    
    if(wr > 0)motor_r.forward();
    else motor_r.backward();
  }
  delay(10);
}


// Increment the number of ticks
void l_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_LEFT_POS);
 
  if(val == LOW) {
    l_dir = false; // Reverse
  }
  else {
    l_dir = true; // Forward
  }
   
  if (l_dir) {
     
    if (l_tick_count.data == encoder_maximum) {
      l_tick_count.data = encoder_minimum;
    }
    else {
      l_tick_count.data++;  
    }    
  }
  else {
    if (l_tick_count.data == encoder_minimum) {
      l_tick_count.data = encoder_maximum;
    }
    else {
      l_tick_count.data--;  
    }   
  }
}


// Increment the number of ticks
void r_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_RIGHT_POS);
 
  if(val == LOW) {
    r_dir = true; // Reverse
  }
  else {
    r_dir = false; // Forward
  }
   
  if (r_dir) {
     
    if (r_tick_count.data == encoder_maximum) {
      r_tick_count.data = encoder_minimum;
    }
    else {
      r_tick_count.data++;  
    }    
  }
  else {
    if (r_tick_count.data == encoder_minimum) {
      r_tick_count.data = encoder_maximum;
    }
    else {
      r_tick_count.data--;  
    }   
  }
}


// Initializing subscriber object
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

void setup()
{
  pinMode(ENC_LEFT_TICKS, INPUT_PULLUP);
  pinMode(ENC_LEFT_POS, INPUT);
  pinMode(ENC_RIGHT_TICKS, INPUT_PULLUP);
  pinMode(ENC_RIGHT_POS, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_TICKS), l_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_TICKS), r_tick, RISING);

  nh.initNode();  // Starting the ROS Node
  nh.subscribe(sub);
  nh.advertise(r_tick_pub);
  nh.advertise(l_tick_pub);
}

void loop()
{
  r_tick_pub.publish(&r_tick_count);
  l_tick_pub.publish(&l_tick_count);
  nh.spinOnce();
  delay(100);
}
