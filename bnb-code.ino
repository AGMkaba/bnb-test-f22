#include <Servo.h>  // add servo library
#include<PID_v1.h> // add PID package by Brett Beauregard

const int potpin = A3;                                                     // analog pin used to connect the potentiometer strip
const int dial = A0;                                                      // analog pin used to connect the potentiometer dial
const int servoPin = 9;                                                   //Servo Pin 9

float val;
float bal;
float cal;
float val2;
float bal2;
float cal2;

float Kp = 1.3;                                                    //Initial Proportional Gain
float Ki = 0.51;                                                      //Initial Integral Gain
float Kd = 0.70;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;
// the setup routine runs once when you press reset:

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.

Servo myServo;                                                       //Initialize Servo.

void setup() {

  Serial.begin(9600);                                                //Begin Serial
  myServo.attach(servoPin);                                     //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls position as the input to the PID algorithm
  Setpoint = setPosition();
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
  myPID.SetOutputLimits(-50, 45);                                    //Set Output limits to -50 and 30 degrees. NOT UNIVERSAL

}
void loop()
{
  Input = readPosition();
  Setpoint = setPosition();

  ServoOutput = 100 + Output;                                          // horizontal (level to how I mounted the servo) NOT UNIVERSAL
  myServo.write(ServoOutput); //Writes value of Output to servo
  delay(10);
  //Serial.println(Setpoint); // Use for testing
  //Serial.println(Input); // Use for Testing

  // Below lines can be used to plot ball position and steady state (steady state set by potentiometer)
  Serial.print(Input);
  Serial.print(',');
  Serial.println(Setpoint);

  myPID.Compute(); // Compute does the math that spits out where your servo should rotate to
}

float readPosition() { // this corresponds to the soft-potentiometer
  delay(10);
  //pinMode(potpin, INPUT);
  // read the input on analog pin 1:
  float sensorValue = cal;
  // print out the value you read:
  //Serial.println(sensorValue);          // prints the read value ( this has been checked, I am getting a correct reading)
  val = analogRead(A3);                   // reads the value of the potentiometer (value between 0 and 1096)
  bal = map(val, 0, 1023, 0, 1968);       // scale it to use it with the servo (value between 0 and 12)
  cal = (18.6 - (bal / 100)); // Mess with this to change scale of soft-potentiometer
  return cal;
}

float setPosition() {
  delay(10);
  pinMode(dial, INPUT);
  // read the input on analog pin 2:
  float sensorValue = cal2;
  //Serial.println(sensorValue);            // prints the read value ( this has been checked, I am getting a correct reading)
  val2 = analogRead(A0);                  // reads the value of the potentiometer (value between 0 and 1023) and maps it to (0 to 1200)
  bal2 = map(val2, 0, 1023, 0, 1200);     // scale it to use it with the servo (value between 0 and 12)
  cal2 = ((bal2 / 100));
  return cal2;
}
