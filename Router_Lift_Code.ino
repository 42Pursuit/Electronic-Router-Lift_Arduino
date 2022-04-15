#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define SS_SWITCH        24
#define SEESAW_ADDR      0x36

#define redLED 5 //LED

#define upperLimitSwitch A1 //Limit switch top
#define lowerLimitSwitch A0 //Limit switch bottom

#define dirPin 12 //Goes to DIR pin on stepper driver
#define pulsePin 13 //Goes to PUL pin on stepper driver

#define rockerUp 7 //Rocker Switch up
#define rockerDown 6 //Rocker Switch up

#define touchProbe 8 //Continuity magnet connection

#define goToTop 11
#define goToBottom 9
#define safetyButton 10

Adafruit_seesaw ss;

int32_t encoder_position;
int32_t stepper_position;

int smallStep = 1;
int largeStep = 8;

int stepSize = largeStep;      // the starting step size of the encoder

long buttonTime = 0;   // the last time the output pin was toggled
long debounce = 200;   // the debounce time of the rotary encoder push button

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Looking for seesaw!");

  if (! ss.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();
  stepper_position = encoder_position;

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();

  pinMode(dirPin,OUTPUT);
  pinMode(pulsePin,OUTPUT);

  pinMode(rockerUp,INPUT_PULLUP);
  pinMode(rockerDown,INPUT_PULLUP);

  pinMode(upperLimitSwitch,INPUT_PULLUP);
  pinMode(lowerLimitSwitch,INPUT_PULLUP);

  pinMode(touchProbe,INPUT_PULLUP);
  pinMode(redLED,OUTPUT);

  pinMode(goToTop,INPUT_PULLUP);
  pinMode(goToBottom,INPUT_PULLUP);
  pinMode(safetyButton,INPUT_PULLUP);
}

void loop()
{
  if (! ss.digitalRead(SS_SWITCH) && millis() - buttonTime > debounce) //Checks if the rotary encoder button is pushed
  {
    if (stepSize == smallStep)
      stepSize = largeStep;
    else
      stepSize = smallStep;

    buttonTime = millis();
  }


  int32_t new_position = ss.getEncoderPosition();  //Checks encoder position

  if (encoder_position != new_position) //Checks if the encoder moved
  {
    if (new_position > encoder_position && digitalRead(lowerLimitSwitch) == HIGH)
    {
      Serial.println(new_position);         // display new position
      encoder_position = new_position;      // and save for next round
    }
    else if (new_position < encoder_position && digitalRead(upperLimitSwitch) == HIGH)
    {
      Serial.println(new_position);         // display new position
      encoder_position = new_position;      // and save for next round
    }
    else
    {
      encoder_position = new_position;
      stepper_position = encoder_position;
    }
  }

  if (stepper_position < encoder_position && digitalRead(lowerLimitSwitch) == HIGH)
  {
    stepper_position++;

    digitalWrite(dirPin,HIGH);

    for (int i = 0; i <= stepSize; i++) //Rotates the stepper motor 'stepSize' steps
    {
    digitalWrite(pulsePin, HIGH);
    digitalWrite(pulsePin, LOW);
    delay(1);
    }
  }
    if (stepper_position > encoder_position && digitalRead(upperLimitSwitch) == HIGH)
  {
    stepper_position--;

    digitalWrite(dirPin,LOW);

    for (int i = 0; i <= stepSize; i++) //Rotates the stepper motor 'stepSize' steps
  {
    digitalWrite(pulsePin, HIGH);
    digitalWrite(pulsePin, LOW);
    delay(1);
    }
  }
  while (digitalRead(rockerUp) == LOW && digitalRead(lowerLimitSwitch) == HIGH) //While rocker switch 'up' is pressed, and the limit switch hasn't been reached
  {
    digitalWrite(dirPin,HIGH);
    digitalWrite(pulsePin, HIGH);
    digitalWrite(pulsePin, LOW);
    delay(2);
  }

  while (digitalRead(rockerDown) == LOW && digitalRead(upperLimitSwitch) == HIGH) //While rocker switch 'down' is pressed, and the limit switch hasn't been reached
  {
    digitalWrite(dirPin,LOW);
    digitalWrite(pulsePin, HIGH);
    digitalWrite(pulsePin, LOW);
    delay(2);
  }

  if (digitalRead(touchProbe) == LOW) //If the touch probe makes connection, turn the LED on. Else, turn if off
    digitalWrite(redLED, HIGH);
  else
    digitalWrite(redLED, LOW);


  if (digitalRead(goToTop) == LOW && digitalRead(safetyButton) == LOW &&  digitalRead(upperLimitSwitch) == HIGH) //If the 'go to top' button is pressed, and the limit switch hasn't been reached
  {
    digitalWrite(dirPin,HIGH);

    while (digitalRead(upperLimitSwitch) == HIGH && digitalRead(rockerDown) == HIGH && digitalRead(rockerUp) == HIGH) //Keep rotating the motor until the limit switch is reached, and while the rocker switch isn't pressed
    {
      digitalWrite(pulsePin, HIGH);
      digitalWrite(pulsePin, LOW);
      delayMicroseconds(1500);
    }
  }


  if (digitalRead(goToBottom) == LOW && digitalRead(safetyButton) == LOW && digitalRead(lowerLimitSwitch) == HIGH) //If the 'go to bottom' button is pressed, and the limit switch hasn't been reached
  {
    digitalWrite(dirPin,LOW);

    while (digitalRead(lowerLimitSwitch) == HIGH && digitalRead(rockerDown) == HIGH && digitalRead(rockerUp) == HIGH) //Keep rotating the motor until the limit switch is reached, and while the rocker switch isn't pressed
    {
      digitalWrite(pulsePin, HIGH);
      digitalWrite(pulsePin, LOW);
      delayMicroseconds(1500);
    }
  }
}
