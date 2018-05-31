//Name: motor_interfacing_code

//Author: Lentin Joseph

//This code will help to test the motor rotation for a differential drive robot. The pin can be change according to the connection

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//serial port enable

bool usb_serial_enable_bit = false;
bool ble_serial_enable_bit = true;


//Ultrasonic sensor enable bit

bool ultrasonic_set = true;



//Flags for Selecting data to BLE and Arduino serial for data send

bool Arduino_serial = 0;
bool Bluetooth_serial = 1;

//For serial read flags
int usb_serial_en = 0;
int ble_serial_en = 1;

bool blinkState;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Ultrasonic sensor interfacing module

#include <NewPing.h>

#define TRIGGER_PIN  46  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     42  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>



  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Messenger object
Messenger Messenger_Handler = Messenger();

#define RESET_PIN 12
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//This is PIN2 of Arduino Mega
#define MPU_6050_INT 0  
#define LED_PIN 13

#define OUTPUT_READABLE_YAWPITCHROLL



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*Testing is like follows
 1) Move Forward
 2) Move Backward
 3) Rotate clockvice
 4) Rotate anticlockvice
 5) Adjusting speed from 0 to 255 forward and backward


 This code is having generic function which can be re-used in your code

 */


 /* Motor driver Pin definitions and mapping to Arduino
  * 
  *  
  */

/* LEFT MOTOR PIN DEFINITIONS 

ARDUINO DIGITAL PIN    ||||   MOTOR DRIVER (L298 PIN)

          4                           ENA
     
          5                           IN1
          
          6                           IN2
          
          7                           ENB
          
          8                           IN4
          
          9                           IN3
          


*/

//PIN NUMBER ON 'IN' IS REVERSE

  #define MOTOR_LEFT_ENA 5   // Connected to driver enb
  #define MOTOR_LEFT_IN1 6
  #define MOTOR_LEFT_IN2 7


  #define MOTOR_RIGHT_ENB 8  //Connected to driver ena
  #define MOTOR_RIGHT_IN3 9
  #define MOTOR_RIGHT_IN4 10


//Motor speed variable 
int motor_left_speed = 0;
int motor_right_speed = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * This section is about defining encoder pins
 * We are using only 1 encoder for dead recokening, but also define provision to add a second encoder
 * 
 */

  //Only one encoder is working , left motor encoder
  #define MOTOR_ENCODER_LEFT 3
  #define MOTOR_ENCODER_LEFT_NO 1
  
  #define MOTOR_ENCODER_RIGHT 2 
  #define MOTOR_ENCODER_RIGHT_NO 0


  
// Declaring variables to keep encoder values

  volatile long left_encoder_ticks;
  volatile long right_encoder_ticks;

  //int left_encoder_ticks;
  //int right_encoder_ticks;

  int left_motor_direction_flag = 1;
  int right_motor_direction_flag = 1;
  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function is to initialize the motor pins that is defined as MACROS
void Setup_Motors()
{

  //Motor Left
  pinMode(MOTOR_LEFT_ENA,OUTPUT);
  pinMode(MOTOR_LEFT_IN1,OUTPUT);
  pinMode(MOTOR_LEFT_IN2,OUTPUT);

  //Motor Right
  pinMode(MOTOR_RIGHT_ENB,OUTPUT);
  pinMode(MOTOR_RIGHT_IN3,OUTPUT);
  pinMode(MOTOR_RIGHT_IN4,OUTPUT);


  // configure LED for testing
  pinMode(LED_PIN, OUTPUT);
  


  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function setup the encoders

void Setup_Encoders()
{
  //Motor EncoderLeft
  pinMode(MOTOR_ENCODER_LEFT,INPUT);
  digitalWrite(MOTOR_ENCODER_LEFT,HIGH); 
  attachInterrupt(MOTOR_ENCODER_LEFT_NO, Count_Left, CHANGE);

  //Motor EncoderRight
  pinMode(MOTOR_ENCODER_RIGHT,INPUT);
  digitalWrite(MOTOR_ENCODER_RIGHT,HIGH); 
  attachInterrupt(MOTOR_ENCODER_RIGHT_NO, Count_Right, CHANGE);


  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ISR for Left encoder
void Count_Left()
{

  left_encoder_ticks += left_motor_direction_flag;
  //Serial.println("Encoder ticks");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ISR for Right encoder
void Count_Right()
{

  right_encoder_ticks += right_motor_direction_flag;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reset encoder; This will reset encoder values

void Reset_Encoder()
{
  left_encoder_ticks = 0;
  right_encoder_ticks = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//To change the test speed, you can change this variable
int test_speed = 60;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//The encoder ticks per revolution, which is found from testing

const int encoder_ticks_rev = 8;




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Setup Serial Ports
//This function initialize Serial 0 and Serial 1 port
void Setup_Serial(int baud_rate,bool arduino_serial,bool bluetooth_serial)
{

    if(arduino_serial == true){
      Serial.begin(baud_rate);}
      
    if(bluetooth_serial ==true){  
      Serial1.begin(baud_rate);
    }

    else if(arduino_serial == false and bluetooth_serial == false)
    {
            Serial.begin(baud_rate);
    }
  
  
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Setup Message Handler

void Setup_Messenger()
{

  pinMode(RESET_PIN,OUTPUT);

  //Set up Messenger 

  
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Update_Ultrasonic()
{
 if(ultrasonic_set == true)
  {

  //delay(50);
  
  int distance = sonar.ping_cm();

  
  if(Arduino_serial == true)
  {
  
   Serial.print("u ");
   if(distance)
    
       Serial.print(distance);
   else
      Serial.print(0);
       
   Serial.print("\n");
   
  }

  if(Bluetooth_serial == true)
  {
   Serial1.print("u ");
   if(distance)
       Serial1.print(distance);
   else
      Serial1.print(0);
       
   Serial1.print("\n");


    
  }

      
}
  
}







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//The following function will help to move robot in any direction with any speed

//We can move the robot forward and reverse upto speed of value 255, If speed is positive, it will move forward, othervice reverse.
void Move_Robot(int motor_speed_left,int motor_speed_right)
{


     // For forward motion
     if(motor_speed_left > 0 && motor_speed_left <= 255)
     {

        //Setting encoder direction flag
        left_motor_direction_flag = 1;
      
        analogWrite(MOTOR_LEFT_ENA, motor_speed_left);//Sets speed variable via PWM 
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
     }

     if(motor_speed_right > 0 && motor_speed_right <= 255)
     {

        //Setting encoder direction flag
        right_motor_direction_flag = 1;
        
        analogWrite(MOTOR_RIGHT_ENB, motor_speed_right);//Sets speed variable via PWM 
        
        digitalWrite(MOTOR_RIGHT_IN3, HIGH); //The two motors are rotating in direction, so here the signal is inversesd than the first
        digitalWrite(MOTOR_RIGHT_IN4, LOW);
     }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

       // For reverse motion


     if(motor_speed_left < 0 && motor_speed_left >= -255 )
  
     {

         //Setting encoder direction flag
        left_motor_direction_flag = -1;
       
        analogWrite(MOTOR_LEFT_ENA, abs(motor_speed_left));//Taking abs
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
     }

     if(motor_speed_right < 0 && motor_speed_right >= -255)
     {

        //Setting encoder direction flag
        right_motor_direction_flag = -1;
      
        analogWrite(MOTOR_RIGHT_ENB, abs(motor_speed_right));//Sets speed variable via PWM 
        
        digitalWrite(MOTOR_RIGHT_IN3, LOW); //The two motors are rotating in direction, so here the signal is inversesd than the first
        digitalWrite(MOTOR_RIGHT_IN4, HIGH);
     }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

       // For Stopping the robot


     if(motor_speed_left == 0 )
     {
        analogWrite(MOTOR_LEFT_ENA, motor_speed_left);//Sets speed variable via PWM 
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
     }

     if(motor_speed_right == 0)
     {
        analogWrite(MOTOR_RIGHT_ENB, motor_speed_right);//Sets speed variable via PWM 
        
        digitalWrite(MOTOR_RIGHT_IN3, HIGH); //The two motors are rotating in direction, so here the signal is inversesd than the first
        digitalWrite(MOTOR_RIGHT_IN4, HIGH);
     }

     
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial(int usb_serial,int ble_serial)

{

  if(usb_serial == 1)
  {
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       
       Messenger_Handler.process(data);
     
     
    } 
  }
  if(ble_serial == 1)
  {
   while(Serial1.available() > 0)
    {
     
       int data = Serial1.read();
       
       Messenger_Handler.process(data);
     
     
    } 

  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     Set_Speed();
     //return; 
    
    
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reset function
void Reset()
{

  Reset_Encoder();
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  delay(3000);
  digitalWrite(RESET_PIN,HIGH);
 
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{
    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();

  //Serial.println(motor_left_speed);
  //Serial.println(motor_right_speed);
  
 
  
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors()
{
  
  Move_Robot(motor_left_speed,motor_right_speed);

//Arduino_serial,Bluetooth_serial

  if(Arduino_serial == true){
  //Serial.print("\n");
  Serial.print("s ");
  //Serial.print(" ");
  Serial.print(motor_left_speed);
  Serial.print(" ");
  Serial.print(motor_right_speed);  
  Serial.print("\n");
  }
  
  if(Bluetooth_serial == true){
  //Serial1.print("\n");
  Serial1.print("s ");
  //Serial1.print(" ");
  Serial1.print(motor_left_speed);
  Serial1.print(" ");
  Serial1.print(motor_right_speed);  
  Serial1.print("\n");
  }


  // blink LED to indicate activity
  blinkState = !blinkState;
  delay(50);
  digitalWrite(LED_PIN, blinkState);
  


}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both encoder value through serial port
void Update_Encoders()
{

  if(Arduino_serial == true)
  {
  //Serial.print("\n");
  Serial.print("e ");
  //Serial.print(" ");
  Serial.print(left_encoder_ticks);
  Serial.print(" ");
  Serial.print(right_encoder_ticks);
  Serial.print("\n");
  
  }
  if(Bluetooth_serial == true)
  {

  //Serial1.print("\n");
  Serial1.print("e ");
  //Serial1.print(" ");
  Serial1.print(left_encoder_ticks);
  Serial1.print(" ");
  Serial1.print(right_encoder_ticks);
  Serial1.print("\n");

    
  }
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update all
void Update_all()
{
  //This will read from the index serial port
  Read_From_Serial(usb_serial_en,ble_serial_en);
  
  Update_Motors();
  
  Update_Encoders();
  
  Update_Ultrasonic();
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

    Setup_Serial(9600,usb_serial_enable_bit,ble_serial_enable_bit);
    
    Setup_Motors();
    
    Setup_Encoders();
    
    Messenger_Handler.attach(OnMssageCompleted);


}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {


  Update_all();

    
}


