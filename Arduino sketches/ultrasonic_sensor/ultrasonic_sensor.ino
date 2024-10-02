#include <Stepper.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::NodeHandle nh;

std_msgs::UInt8MultiArray scan;
ros::Publisher scan_pub("laser_data",&scan);

// Pins for the HCSR_04 Ultrasonic module
const int trigPin = 8;
const int echoPin = 9;

long duration;
byte distance[91];


/* 
Number of steps per rotation
32 Internal Teeth of Rotor = 1 rotation at shaft => 32 Pulses gives 1 rotation of Rotor shaft
Gear Ratio from Rotor Shaft to Output Shaft = 64:1 => 1 Rotation of Rotor shaft = 1/64 Rotations of Output Shaft
=> 64 Rotations of Rotor Shaft = 1 Rotation of Output Shaft
=> Total Input Pulses (Steps) required = 32x64 = 2048 Steps (Pulses) for one Revolution of Output shaft
=> Angular Resolution = 360/2048 ~= 0.175 degrees / Step (Pulse)
*/
const int stepsPerRevolution = 2048;
const int totalangle = 180;
const int totalsteps = (stepsPerRevolution / 360) * totalangle;
const int angle = 2;
const int arraysize = (totalangle/angle) + 1 ;
int steps = (stepsPerRevolution / 360) * angle;
const int rpm = 15;

int current_index = 1;
int iter = 1;

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 10, 12, 11, 13);

void setup()
{
  // Setting up the pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW); // Setting the Trigger Pin to LOW

  // Initializing the nodehandle and the publisher
  nh.initNode();
  nh.advertise(scan_pub);

  scan.data_length = 91;  // Initializing the messsage length

  myStepper.setSpeed(rpm);  // Setting the speed of the stepper
}

void loop()
{
  // Generating the Ultrasonic Pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); // Timing the return pulse
  distance[current_index] = min(duration * 0.0343 / 2, 255);  // Determining the distance
  
  myStepper.step(steps * iter); // Moving the stepper in corresponding direction
  delayMicroseconds(50);
  
  current_index = current_index + iter;  // Iterating the index
  
  // Publishing the message and updating the iterator at the limits
  if(current_index == 90)
  {
    scan.data = distance;
    scan_pub.publish(&scan);
    iter = -1;
  }
  else if(current_index == 0)
  {
    scan.data = distance;
    scan_pub.publish(&scan);
    iter = 1;
  }
  
  nh.spinOnce();
}
