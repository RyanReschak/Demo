//Erik Salazar
//Feb 10, 2021
//Velocity left and Right of Wheels
//Description: calculate the velocities of the left and right wheel  
#define PI 3.1415926535897932384626433832795
#define MV1 9
#define MV2 10
#define VS1 7
#define VS2 8
#define pin4 4
#define pin12 12
#define a  2
#define b  5  //initialize pins
#define c  3
#define d  6
long int counter1 = 0; //keeps track of position of encoder
long int counter2 = 0;
const float r = 7.3;  //these values are the constants of the body of the car and wheels in (cm)
const float base = 24.45; //in cm

//1 means right wheel and 2 means left wheel
float AngularVelocity1 = 0;  //initialize all variables
float AngularVelocity2 = 0;
float StartTime1 = 0;
float EndTime1 = 0;
float StartTime2 = 0;
float EndTime2 = 0;
float StartLoop = 0;
float EndLoop = 0;
float p_dot = 0;
float phi = 0;
float phi_dot = 0;
float Position1 = 0;
float Position2 = 0;
int PWMOutput1 = 0;
int PWMOutput2 = 0;


void setup() {
  
  // set pinmodes for digital inputs to have pullup resistor:
  pinMode(a, INPUT_PULLUP);
  digitalWrite(a, HIGH);
  pinMode(b, INPUT_PULLUP);
  digitalWrite(b, HIGH);
  pinMode(c, INPUT_PULLUP);
  digitalWrite(c, HIGH);
  pinMode(d, INPUT_PULLUP);
  digitalWrite(d, HIGH);
  pinMode(MV1,OUTPUT);
  pinMode(MV2,OUTPUT);
  pinMode(VS1,OUTPUT);
  pinMode(VS2,OUTPUT);
  pinMode(pin4,OUTPUT);
  pinMode(pin12,INPUT);
  digitalWrite(pin4,HIGH);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  StartTime1 = micros();
  StartTime2 = micros();  //these are too keep track of the time in the loop and interrupt
  StartLoop = micros();
  attachInterrupt(digitalPinToInterrupt (a),interruptEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt (c),interruptEncoder2, RISING);  //initialize the interrupts and sets the flag
  
}

// the loop routine runs over and over again forever:
void loop() {
  //Serial.print((StartLoop/1000000),5); //print time
  //Serial.print("\t"); //print tab
  //digitalWrite(VS1,HIGH);
  if ((Position1+Position2)/2*r < 305){
    analogWrite(MV1, 100);
    analogWrite(MV2, 100);
  }else{
    analogWrite(MV1, 0);
    analogWrite(MV2, 0);
  }
  
  //p_dot = r*(AngularVelocity1 + AngularVelocity2)/2;  //equation for the velocity of the left wheel
  Serial.println((Position1+Position2)/2*r,5);
  //Serial.print("\t");
  //phi_dot = r*(AngularVelocity1 - AngularVelocity2)/base; //equation for the velocity of the right wheel
  //Serial.print(phi_dot,5);
 
  //Serial.print("\t");
  //Serial.println(abs(p_dot)*(StartLoop/1000000));
  /*Serial.print(counter1);
  Serial.print("\t");
  Serial.println(counter2);*/
  //AngularVelocity1 = 0;
  //AngularVelocity2 = 0; //resets the velocity to zero because the car should not be moving 
  StartLoop = EndLoop; // set the beginning time to current time
  EndLoop = micros(); // initialize the end time
  }

//Right Wheel
void interruptEncoder1 (){ //interrupt
  // read the input pin:
  if (digitalRead(a) == digitalRead(b)){ //checks to see if A channel is equal to Chanel B
    EndTime1 = micros();  // end time for interrupt1 to calculate angular velocity
    counter1 = counter1 - 2;
    Position1 = (counter1*2*PI)/1600;
    //AngularVelocity1 = (-2*2*PI/3200)/((EndTime1 - StartTime1)/1000000);
    
  }else{
    
    EndTime1 = micros();
    counter1 = counter1 + 2;
    Position1 = (counter1*2*PI)/1600;
    //AngularVelocity1 = (2*2*PI/3200)/((EndTime1 - StartTime1)/1000000);
  }
  StartTime1 = EndTime1; //reset the start time to current time
}

//Left Wheel
void interruptEncoder2 (){ //interrupt
  // read the input pin:
  if (digitalRead(c) == digitalRead(d)){ //checks to see if A channel is equal to Chanel B
    EndTime2 = micros();
    counter2 = counter2 + 2;
    Position2 = (counter2*2*PI)/1600;
    AngularVelocity2 = (2*2*PI/3200)/((EndTime2 - StartTime2)/1000000);
  }else{
    EndTime2 = micros();
    counter2 = counter2 - 2;
    Position2 = (counter2*2*PI)/1600;
    //AngularVelocity2 = (-2*2*PI/3200)/((EndTime2 - StartTime2)/1000000);
  }
  StartTime2 = EndTime2;
}

void Distance() {
  //Angular Position of Wheel
  currPos = (Position1+Position2)/2*r;
  
  //currPos=AngularPosition1;
  
  error=setPosition-currPos;
  
  I=I+(Ts/1000)*error;
  
  PWMOutput = error*(Kp+Ki*I);
  
  if(abs(PWMOutput)>255){
    PWMOutput=constrain(PWMOutput,-1,1)*255;
    error = constrain(error,-1,1)*min(255/Kp,abs(error));
  }
 
  //Both wheels moving forward
  digitalWrite(VS1,HIGH);
  digitalWrite(VS2,HIGH);
  
  PWMOutput = abs(PWMOutput);

  analogWrite(MV1, PWMOutput1);
  analogWrite(MV2, PWMOutput1);
  
  Ts=micros()-Tc;
  Tc=micros();

  
}

void RotateBot() {
  //Angular Position of Wheel
  currPosRight = Position1;
  currPosLeft = Position2;
  //currPos=AngularPosition1;
  
  error=setPositionAngle-currPos;
  
  I=I+(Ts/1000)*error;
  
  PWMOutput = error*(Kp+Ki*I);
  
  if(abs(PWMOutput)>255){
    PWMOutput=constrain(PWMOutput,-1,1)*255;
    error = constrain(error,-1,1)*min(255/Kp,abs(error));
  }
 
 
  digitalWrite (VS1, HIGH);
  PWMOutput = abs(PWMOutput);

  analogWrite(outPin, PWMOutput);
  Ts=micros()-Tc;
  Tc=micros();

  
}
