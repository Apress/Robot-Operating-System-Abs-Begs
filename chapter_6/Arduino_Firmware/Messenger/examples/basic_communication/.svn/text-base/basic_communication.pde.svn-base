// This example demonstrates Messenger's callback  & attach methods
// It outputs all the values of the default analog pins
// It sets the state (digitalWrite) of pins when receiving messages


#include <Messenger.h>

// Define a metronome
unsigned long previousMillis = 0;
//Set the default interval to every 20 milliseconds
unsigned long interval = 20;

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger message = Messenger(); 


// Define messenger function
void messageCompleted() {
  int pin = 0;
  // Loop through all the available elements of the message
  while ( message.available() ) {
    int pin = message.readInt();
    int state = message.readInt();
    // Set the pin as determined by the message
    digitalWrite( pin, state);
  }
}

void setup() {
  // Initiate Serial Communication
  Serial.begin(115200); 
  message.attach(messageCompleted);
}

void loop() {
  // The following line is the most effective way of 
  // feeding the serial data to Messenger
  while ( Serial.available( ) ) message.process(Serial.read( ) );

  // Output the analog values
  if ( millis() - previousMillis > interval ) {
    previousMillis = millis();
    for ( byte i = 0; i < 6; i++) {
      Serial.print( analogRead(i) );
      Serial.print(' ');
    }
    Serial.println();
  }

}

