//Arduino Stepper Library
#include <Stepper.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

//Variables & connections for Ultrasonic Sensor HCSR04 
const int trigPin = 8;
const int echoPin = 9;
long duration;
int distance;
int current = 0;

ros::NodeHandle nh;

std_msgs::UInt8MultiArray scan;
ros::Publisher scan_pub("laser_data",&scan);

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
const int steps = (stepsPerRevolution / 360) * angle;
//Define Rotation Speed of Rotor
const int rpm = 15;
long duration;
byte distance[91];

// Create an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 10, 12, 11, 13);

void setup() {
  // Nothing to do for Stepper as Stepper Library sets pins as outputs
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  nh.initNode();
  nh.advertise(scan_pub);
  scan.data_length = 91;  // Initializing the messsage length
}

void loop() {
  
  if (current!= 0){
  digitalWrite(trigPin, LOW);
  myStepper.setSpeed(rpm);
  myStepper.step(steps);
  current -= 2;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.0343 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Angle: ");
  Serial.print(current);
  Serial.print(" Distance: ");
  Serial.println(distance);
  }
  else{
  digitalWrite(trigPin, LOW);
  myStepper.setSpeed(rpm);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0343 / 2;
  Serial.print("Angle: ");
  Serial.print(current);
  Serial.print(" Distance: ");
  Serial.println(distance);
  }
  
  for (int i =0; i <= (totalsteps-1); i = i + steps){
  digitalWrite(trigPin, LOW);
  //delayMicroseconds(2);
  //Set speed at the specified RPM
  myStepper.setSpeed(rpm);
  //Rotate CW 
  myStepper.step(steps);
  current += 2;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.0343 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Angle: ");
  Serial.print(current);
  Serial.print(" Distance: ");
  Serial.println(distance);
//  delay(100);
  }
  scan.data = distance;
  scan_pub.publish(&scan);
  nh.spinOnce();
  delay(10000);

//  current = 180;
  for (int i = (totalsteps-1); i >= 0; i = i - steps){
  digitalWrite(trigPin, LOW);
  //delayMicroseconds(2);
  //Set speed at the specified RPM
  myStepper.setSpeed(rpm);
  //Rotate CW 
  myStepper.step(-steps);
  current -= 2;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.0343 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Angle: ");
  Serial.print(current);
  Serial.print(" Distance: ");
  Serial.println(distance);
//  delay(100);
  }
  Serial.print("Out of loop 2");
  Serial.print("Current Angle");
  Serial.println(current);
  scan.data = distance;
  scan_pub.publish(&scan);
  nh.spinOnce();
}
