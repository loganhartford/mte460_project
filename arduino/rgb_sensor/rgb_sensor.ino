#include <Servo.h>

#define S0 7
#define S1 5
#define S2 3
#define S3 2
#define OUT_PIN 4
#define LED_PIN 8

#define LED_r 9
#define LED_g 10
#define LED_b 11

#define LED_ON 1

#define SERVO 6
#define RED_POS 0
#define BLUE_POS 60
#define GREEN_POS 120
#define YELLOW_POS 180

Servo servo;

unsigned int red;
unsigned int green;
unsigned int blue;

bool motorOn = false;
bool blockPresent = false;

void turnMotorOn(bool on);
unsigned int readColor(int s2, int s3);
void getColors(float alpha);
void setRGBLED(String color);
void interpretColors();

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_r, OUTPUT);
  pinMode(LED_g, OUTPUT);
  pinMode(LED_b, OUTPUT);

  servo.attach(SERVO);

  // S0 and S1 configuration for output frequency scaling (f0):
  // S0  |  S1  |  Output Frequency Scaling
  // LOW | LOW  | Power down
  // LOW | HIGH | 2%
  // HIGH| LOW  | 20%
  // HIGH| HIGH | 100%
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

  Serial.begin(9600);

  // Set LEDs
  digitalWrite(LED_PIN, LED_ON);
  setRGBLED("off");
  
  // Set servo to default postition
  servo.write(0);
}

void loop() {
  // Read color sensor
  getColors(0.2);

  int gr = abs(green - red);
  
  // Nothing there
  if ((green > blue) && (blue > red) && (gr < 400))
  {
    blockPresent = false;
    if (!motorOn)
    {
      // Send signal to turn motor on
      turnMotorOn(true);
    }
  }
  else // Something there
  {
    blockPresent = true;
    // Delay to let block get into pos (will have to adjust)
    delay(100);
    
    if (motorOn)
    {
      // Send signal to turn motor off
      turnMotorOn(false);
    }

    // Take an average
    for (int i = 0; i < 10; i++)
    {
      getColors(0.01); // Slower alpha
    }

    // check for false positive
    if ((green > blue) && (blue > red) && (gr < 400))
    {
      blockPresent = false;
    }
    else if ((blue > green) && (green > red))
    {
      setRGBLED("yellow");
      servo.write(YELLOW_POS);
    }
    else if ((green < red) && (green < blue))
    {
      setRGBLED("green");
      servo.write(GREEN_POS);
    }
    else if ((red < green) && (red < blue))
    {
      setRGBLED("red");
      servo.write(RED_POS);
    }
    else if ((blue < green) && (blue < red))
    {
      setRGBLED("blue");
      servo.write(BLUE_POS);
    }

    // This delay is just so we can see what color the led goes to
    // can delete this later
    delay(300);
    
    // Send signal to turn motor on
    turnMotorOn(true);

    // Let block clear the color sensor
    while (blockPresent)
    {
      getColors(0.1);
      if ((green > blue) && (blue > red) && (gr < 400))
      {
        blockPresent = false;
      }
    }
  }
  delay(10);
}

void turnMotorOn(bool on)
{
  motorOn = on;
  // Send signal to PLC
  if (motorOn)
  {
    setRGBLED("white");
  }
  else
  {
    setRGBLED("purple");
  }
}

// S2 and S3 configuration for photodiode type selection:
// S2  |  S3  |  Photodiode Type
// LOW | LOW  | red
// LOW | HIGH | blue
// HIGH| LOW  | Clear (no filter)
// HIGH| HIGH | green
unsigned int readColor(int s2, int s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3); 
  
  // Measure pulse duration based on the current state of the output pin
  return pulseIn(OUT_PIN, digitalRead(OUT_PIN) == HIGH ? LOW : HIGH);
}

float filteredRed = 0;
float filteredGreen = 0;
float filteredBlue = 0;

void getColors(float alpha) {
  int newRed = readColor(LOW, LOW);
  delay(20);
  int newBlue = readColor(LOW, HIGH);
  delay(20);
  int newGreen = readColor(HIGH, HIGH);
  delay(20);

  // Apply low-pass filter
  filteredRed = alpha * newRed + (1 - alpha) * filteredRed;
  filteredBlue = alpha * newBlue + (1 - alpha) * filteredBlue;
  filteredGreen = alpha * newGreen + (1 - alpha) * filteredGreen;

  // If needed, cast to int if you want whole numbers
  red = (int)filteredRed;
  blue = (int)filteredBlue;
  green = (int)filteredGreen;

  // Can view in serial plotter
  Serial.print("Blue:");
  Serial.println(blue);
  Serial.print("Red:");
  Serial.println(red);
  Serial.print("Green:");
  Serial.println(green);
}

void setRGBLED(String color) {

  if (color == "red") {
    analogWrite(LED_r, 255);
    analogWrite(LED_g, 0);
    analogWrite(LED_b, 0);
  }
  else if (color == "green") {
    analogWrite(LED_r, 0);
    analogWrite(LED_g, 255);
    analogWrite(LED_b, 0);
  }
  else if (color == "blue") {
    analogWrite(LED_r, 0);
    analogWrite(LED_g, 0);
    analogWrite(LED_b, 255);
  }
  else if (color == "yellow") {
    analogWrite(LED_r, 255);
    analogWrite(LED_g, 255);
    analogWrite(LED_b, 0);
  }
  else if (color == "white")
  {
    analogWrite(LED_r, 255);
    analogWrite(LED_g, 255);
    analogWrite(LED_b, 255);
  }
  else if (color == "purple")
  {
    analogWrite(LED_r, 255);
    analogWrite(LED_g, 0);
    analogWrite(LED_b, 255);
  }
  else {
    analogWrite(LED_r, 0);
    analogWrite(LED_g, 0);
    analogWrite(LED_b, 0);
  }


}

// Will probably have to change this based on how it runs in the lab
void interpretColors() 
{
 
}
