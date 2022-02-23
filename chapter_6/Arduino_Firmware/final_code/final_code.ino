 
//Name: robot_firmware

//Author: Lentin Joseph

//ROS Arduino code publishing sensor data and subscribing to motor commands 


#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Float32.h>


////////////////////////////////////////////////////////////////////////////////
//Motor, Encoder and IR sensor pin definition

int encoder_pinA = 2;    
int encoder_pinB = 3;
                             
volatile int pulses1 = 0;  
volatile int pulses2 = 0;      

#define IR_PIN A0

//Motor A
int enableA = 5;
int MotorA1 = 6;
int MotorA2 = 7;
 
//Motor B
int enableB = 8;
int MotorB1 = 9;
int MotorB2 = 10;

/////////////////////////////////////////////////////////////////////////////////////

//ROS Node handle
ros::NodeHandle  nh;
//Left and right speed
int r_speed = 0, l_speed = 0;

//Direction flag for encoder
int left_direction = 1;
int right_direction = 1;


/////////////////////////////////////////////////////////////////////////////////////////////////////////

void left_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{

   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
   l_speed = msg.data;

}


void right_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
   r_speed = msg.data;

}

void reset_cb(const std_msgs::Bool& msg)
{

    l_speed = 0;
    r_speed = 0;

    pulses1 = 0;
    pulses2 = 0;

  
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Mapping function one range to another range

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


////////////////////////////////////////////////////////////////////////////////////////////////
//Publisher for Left and Right encoder

std_msgs::Int32 l_encoder_msg;
ros::Publisher l_enc_pub("left_ticks", &l_encoder_msg);

std_msgs::Int32 r_encoder_msg;
ros::Publisher r_enc_pub("right_ticks", &r_encoder_msg);

//Sharp distance publisher

std_msgs::Float32 sharp_msg;
ros::Publisher sharp_distance_pub("obstacle_distance", &sharp_msg);

//Subscribers for left and right speed

ros::Subscriber<std_msgs::Int32> left_speed_sub("set_left_speed",&left_speed_cb);  // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Int32> right_speed_sub("set_right_speed",&right_speed_cb);  // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Bool> reset_sub("reset",&reset_cb);  // creation of subscriber object sub for recieving the cmd_vel



////////////////////////////////////////////////////////////////////////////////////////////////


// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        

// Loop frequency: 100 ms
const long interval = 50;           


//////////////////////////////////////////////////////////////////////////////////////////////////////
//ISR for two encoders

void counter1(){
 
        pulses1 = pulses1 + left_direction;    
     
}

void counter2(){
 
        pulses2 = pulses2 + right_direction;    
     
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setting encoder pins as interrupts

void setup_wheelencoder()
{
 
   pinMode(encoder_pinA, INPUT);
   attachInterrupt(digitalPinToInterrupt (encoder_pinA), counter1, RISING);
   pulses1 = 0;
   pinMode(encoder_pinB, INPUT);
   attachInterrupt(digitalPinToInterrupt (encoder_pinB), counter2, RISING);
   pulses2 = 0;
   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read IR values and publish

void update_IR()
{
 
  float volts = analogRead(IR_PIN)*0.0048828125;  // value from sensor * (5/1024)
  float distance = 13*pow(volts, -1); // worked out from datasheet graph

  sharp_msg.data = distance;
  sharp_distance_pub.publish(&sharp_msg);
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_Motor()
{

  //If left speed is greater than zero
  if(l_speed >= 0)
  {
    digitalWrite (MotorA1, LOW);
    digitalWrite (MotorA2, HIGH);

    analogWrite(enableA,abs(l_speed));  

    left_direction = 1;
    
  }
  else
  {

    digitalWrite (MotorA1, HIGH);
    digitalWrite (MotorA2, LOW);

    analogWrite(enableA,abs(l_speed));    

    left_direction = -1;

    
  }

  if(r_speed >= 0)
  {

    digitalWrite (MotorB1, HIGH);
    digitalWrite (MotorB2, LOW);
    analogWrite(enableB,abs(r_speed));    

    right_direction = 1;

     
  }
  else
  {

    digitalWrite (MotorB1, LOW);
    digitalWrite (MotorB2, HIGH);
    analogWrite(enableB,abs(r_speed));    

    right_direction = -1;

   
  }
  

  
}

void setup()
{
  //Setting Serial1 and bluetooth as default serial port for communication via Bluetooth
  
  nh.getHardware()->setPort(&Serial1);
  nh.getHardware()->setBaud(9600);

  pinMode (enableA, OUTPUT);
  pinMode (MotorA1, OUTPUT);
  pinMode (MotorA2, OUTPUT);  
   
  pinMode (enableB, OUTPUT);
  pinMode (MotorB1, OUTPUT);
  pinMode (MotorB2, OUTPUT); 
   
  pinMode(LED_BUILTIN, OUTPUT);  

  //Setup wheel encoders
  setup_wheelencoder();

  //Initialize ROS node
  nh.initNode();

  //Setup publisher
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
  nh.advertise(sharp_distance_pub);

  //Setup subscriber
  nh.subscribe(left_speed_sub);
  nh.subscribe(right_speed_sub);
  nh.subscribe(reset_sub);
  
}

void loop()
{

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;

    l_encoder_msg.data = pulses1;
    r_encoder_msg.data = pulses2;

    l_enc_pub.publish(&l_encoder_msg);
    r_enc_pub.publish(&r_encoder_msg);

    update_IR();
  
  }

  update_Motor();
  nh.spinOnce();

  delay(20);
}
