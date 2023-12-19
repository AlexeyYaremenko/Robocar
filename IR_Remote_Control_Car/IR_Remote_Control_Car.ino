#include <IRremote.hpp>

#include <IRremote.h>
#include <Servo.h>

int RECV_PIN = 12;
// IRrecv irrecv(RECV_PIN);
decode_results results;

#define IR_Go      70
#define IR_Back    21
#define IR_Left    68
#define IR_Right   67
#define IR_Stop    64
#define IR_ESC     0x00ff52ad

Servo myservo;
int inputPin = A0;      // Ultrasonic module ECHO to A0
int outputPin = A1;     // Ultrasonic module TRIG to A1

#define Mode_1 22
#define Mode_2 25
#define Mode_3 13

#define Lpwm_pin  5     // Adjusting speed 
#define Rpwm_pin  6     // Adjusting speed
int pinLB = 2;          // Defining pin 2 left rear
int pinLF = 4;          // Defining pin 4 left front
int pinRB = 7;          // Defining pin 7 right rear
int pinRF = 8;          // Defining pin 8 right front
unsigned char Lpwm_val = 120;
unsigned char Rpwm_val = 150;
int Car_state = 0;
int Car_mode = 1;

int Car_mode_1_pin = 3;
int Car_mode_2_pin = 13;

// 0, 1, 3, 13

unsigned char DuoJiao = 90;    // Initialized angle of the motor at 90°

#define SensorLeft    9    // Input pin of left sensor
#define SensorMiddle  10   // Input pin of middle sensor
#define SensorRight   11   // Input pin of right sensor

unsigned char SL;         // State of left sensor
unsigned char SM;         // State of middle sensor
unsigned char SR;         // State of right sensor

unsigned char old_SL, old_SM, old_SR;

void Sensor_IO_Config()
{
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);
}

void Sensor_Scan(void)
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void M_Control_IO_config(void)
{
  pinMode(pinLB, OUTPUT);  // Pin 2
  pinMode(pinLF, OUTPUT);  // Pin 4
  pinMode(pinRB, OUTPUT);  // Pin 7 
  pinMode(pinRF, OUTPUT);  // Pin 8
  pinMode(Lpwm_pin, OUTPUT);  // Pin 5 (PWM)
  pinMode(Rpwm_pin, OUTPUT);  // Pin 6 (PWM)   
}

void Set_Speed(unsigned char Left, unsigned char Right)
{
  analogWrite(Lpwm_pin, Left);
  analogWrite(Rpwm_pin, Right);
}

void advance()  // Going forward
{
  digitalWrite(pinRB, LOW);   // Making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);   // Making motor move towards left rear
  digitalWrite(pinLF, HIGH); 
  Car_state = 1;   
}

void turnR()  // Turning right (dual wheel)
{
  digitalWrite(pinRB, LOW);   // Making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);   // Making motor move towards left front
  Car_state = 4;
}

void turnL()  // Turning left (dual wheel)
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);    // Making motor move towards right front
  digitalWrite(pinLB, LOW);    // Making motor move towards left rear
  digitalWrite(pinLF, HIGH);
  Car_state = 3;
}

void stopp()  // Stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  Car_state = 5;
}

void back()  // Back up
{
  digitalWrite(pinRB, HIGH);   // Making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);   // Making motor move towards left rear
  digitalWrite(pinLF, LOW);
  Car_state = 2;  
}

void IR_Control(void)
{
  unsigned long Key;
  if (IrReceiver.decode()) // Judging if serial port receives data   
  {
    Key = IrReceiver.decodedIRData.command;
    switch (Key)
    {
      case IR_Go: advance();   // UP
        break;
      case IR_Back: back();   // Back
        break;
      case IR_Left: turnL();   // Left    
        break;
      case IR_Right: turnR(); // Right
        break;
      case IR_Stop: stopp();   // Stop
        break;
      default: 
        break;      
    } 
    IrReceiver.resume();
  } 
}

void Self_Control(void) // Self-going, ultrasonic obstacle avoidance
{
  int H;
  myservo.write(DuoJiao);
  H = Ultrasonic_Ranging(1);
  delay(300);

  if (Ultrasonic_Ranging(1) < 15)
  {
    stopp();
    delay(100);
    back();
    delay(50);
  }

  if (Ultrasonic_Ranging(1) < 30)
  {
    stopp();
    delay(100);
    myservo.write(0);
    int L = ask_pin_L(2);
    delay(300);
    myservo.write(180);;
    int R = ask_pin_R(3);
    delay(300);

    if (ask_pin_L(2) > ask_pin_R(3))
    {
      back();
      delay(100);
      turnL();
      delay(400);
      stopp();
      delay(50);
      myservo.write(DuoJiao);
      H = Ultrasonic_Ranging(1);
      delay(500);
    }

    if (ask_pin_L(2) <= ask_pin_R(3))
    {
      back();
      delay(100);
      turnR();
      delay(400);
      stopp();
      delay(50);
      myservo.write(DuoJiao);
      H = Ultrasonic_Ranging(1);
      delay(300);
    }

    if (ask_pin_L(2) < 35 && ask_pin_R(3) < 35)
    {
      stopp();
      delay(50);
      back();
      delay(50);
    }
  }
  else
  {
    advance();
  }
}

int Ultrasonic_Ranging(unsigned char Mode) // Function of ultrasonic distance detecting, MODE=1, displaying, no displaying under other situation
{
  int old_distance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int distance = pulseIn(inputPin, HIGH); // Reading the duration of high level
  distance = distance / 58;               // Transform pulse time to distance
  if (Mode == 1)
  {
    Serial.print("\n H = ");
    Serial.print(distance, DEC);
    return distance;
  }
  else
    return distance;
}

int ask_pin_L(unsigned char Mode)
{
  int old_Ldistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int Ldistance = pulseIn(inputPin, HIGH);
  Ldistance = Ldistance / 58; // Transform pulse time to distance
  if (Mode == 2)
  {
    Serial.print("\n L = ");
    Serial.print(Ldistance, DEC);
    return Ldistance;
  }
  else
    return Ldistance;
}

int ask_pin_R(unsigned char Mode)
{
  int old_Rdistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // 
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int Rdistance = pulseIn(inputPin, HIGH);
  Rdistance = Rdistance / 58; // Transform pulse time to distance
  if (Mode == 3)
  {
    Serial.print("\n R = ");
    Serial.print(Rdistance, DEC);
    return Rdistance;
  }
  else
    return Rdistance;
}

void IR_CarModeSwitch(void)
{
  unsigned long Key;

  if (IrReceiver.decode()) // Judging if serial port receives data   
  {
    Key = IrReceiver.decodedIRData.command;
    switch (Key)
    {
      case Mode_1:
      case IR_Stop:
        stopp();
        myservo.write(DuoJiao);
        Car_mode = 1;

        Serial.println('Car mode 1');

        digitalWrite(Car_mode_1_pin, LOW);
        digitalWrite(Car_mode_2_pin, LOW);
        break;
      case Mode_2:
        stopp();
        myservo.write(DuoJiao);
        Car_mode = 2;

        Serial.println('Car mode 2');
        
        digitalWrite(Car_mode_1_pin, HIGH);
        digitalWrite(Car_mode_2_pin, LOW);
        break;
      case Mode_3:
        stopp();
        myservo.write(DuoJiao);
        Car_mode = 3;

        Serial.println('Car mode 3');

        digitalWrite(Car_mode_1_pin, LOW);
        digitalWrite(Car_mode_2_pin, HIGH);
        break;
      default: 
        break;      
    } 

    IrReceiver.resume();
  } 
}

void SensorMoving()
{
  Sensor_Scan();

  if (SM == HIGH)  // Middle sensor in black area
  {
    if (SL == LOW && SR == HIGH) // Black on left, white on right, turn left
    {
      turnR();
    }
    else if (SR == LOW && SL == HIGH) // White on left, black on right, turn right
    {
      turnL();
    }
    else  // White on both sides, going forward
    {
      advance();
    }
  }
  else  // Middle sensor on white area
  {
    if (SL == LOW && SR == HIGH)  // Black on left, white on right, turn left
    {
      turnR();
    }
    else if (SR == LOW && SL == HIGH)  // White on left, black on right, turn right
    {
      turnL();
    }
    else  // All white, stop
    {
      back();
      delay(100);
      stopp();
    }
  }
}

void setup() 
{ 
  myservo.attach(A2);
  Sensor_IO_Config();
  M_Control_IO_config();
  Set_Speed(Lpwm_val, Rpwm_val);
  myservo.write(DuoJiao);        // Setting initialized motor angle
  // irrecv.enableIRIn(); // Start the receiver
  IrReceiver.begin(RECV_PIN, false);
  pinMode(inputPin, INPUT);      // Starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    // IO of ultrasonic module
  Serial.begin(9600);   // Initializing serial port, Bluetooth used as serial port, setting baud ratio at 9600 
  stopp();
  delay(1000);

  pinMode(Car_mode_1_pin, OUTPUT);
  pinMode(Car_mode_2_pin, OUTPUT);
}

void loop() 
{
  IR_CarModeSwitch();
  if (Car_mode == 1) {
    IR_Control();
  } else if (Car_mode == 2) {
    Self_Control();
  } else if (Car_mode == 3) {
    SensorMoving();
  }
}
