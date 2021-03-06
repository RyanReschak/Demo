
//
//
//RyanReschak
///
//Demo
//1
//00
//Code
//Issues
//Pull requests
//Actions
//Projects
//Wiki
//Security
//Insights
//Demo/Demo1_Motor_drive/Demo1_Motor_drive.ino
//@MWehrlen
//MWehrlen added motor driver code in place of previous prototypes
//Latest commit 1f21c16 3 days ago
// History
// 2 contributors
//@3rikSal@MWehrlen
//265 lines (217 sloc)  7.27 KB
#define PI 3.1415926535897932384626433832795
#define PWMperVolt  32.69
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
const float KpTheta =-105.627;
const float Kd = 0;
const float KpRoh = 0.64981382;
const float KpPhi = -5.9204389;

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
float upperThreshold = 0.1;
float phi_dot = 0;
float Position1 = 0;
float Position2 = 0;
float setPosition = 0;
float desAngle = PI/2;
float desForwardSpeed =0;
float desTurningRate =0;
float currForwardSpeed =0;
float currTheta =0;
float currTurningRate =0;
float errorForwardSpeed =0;
float errorTurningSpeed =0;
float errorTheta =0;
float errorThetaPast=0;
float Vabs =0;
float Vdelta =0;
float Va1 =0;
float Va2 =0;
float Ts=0;
float Tc;
float D=0;
int PWMOutput1 = 0;
int PWMOutput2 = 0;
boolean done  = false;
static unsigned int state;


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
  Ts=micros();
  StartTime1 = micros();
  StartTime2 = micros();  //these are too keep track of the time in the loop and interrupt
  StartLoop = micros();
  state =0;
  attachInterrupt(digitalPinToInterrupt (a),interruptEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt (c),interruptEncoder2, RISING);  //initialize the interrupts and sets the flag
  
}

// the loop routine runs over and over again forever:
void loop() {
    MotionController();
    switch(state){
      case 0:
        if(desForwardSpeed == 0 && ((errorTheta) <= upperThreshold) && desAngle > 0) {
//          Serial.println("Hello");
          AngularVelocity1 = 0;
          AngularVelocity2 = 0;
          desTurningRate = 0;
          desAngle = 0;
          currTheta = 0;
          desForwardSpeed = 10;
          setPosition = 30.48;
          Position1 = 0;
          Position2 = 0;
          counter1 = 0;
          counter2 = 0;
          state = 1;
        }else if(desForwardSpeed !=0 && desAngle ==0){
          state =1;
        }else if(desForwardSpeed == 0 && (errorTheta >= upperThreshold) && desAngle < 0){
          AngularVelocity1 = 0;
          AngularVelocity2 = 0;
          desTurningRate = 0;
          desAngle = 0;
          currTheta = 0;
          desForwardSpeed = 10;
          setPosition = 30.48;
          Position1 = 0;
          Position2 = 0;
          counter1 = 0;
          counter2 = 0;
          state = 1;
        }
        break;
      case 1:
        if((Position1+Position2)/2*r >= setPosition){
          desForwardSpeed = 0;
          desTurningRate = 0;
          desAngle = 0;
          currTheta = 0;
          errorTheta = 0;
          AngularVelocity1 = 0;
          AngularVelocity2 = 0;
          Position1 = 0;
          Position2 = 0;
          counter1 = 0;
          counter2 = 0;
          state =1;
        }
        
        break;
    }
  }

//Right Wheel
void interruptEncoder1 (){ //interrupt
  // read the input pin:
  if (digitalRead(a) == digitalRead(b)){ //checks to see if A channel is equal to Chanel B
    EndTime1 = micros();  // end time for interrupt1 to calculate angular velocity
    counter1 = counter1 - 2;
    Position1 = (counter1*2*PI)/1600;
    AngularVelocity1 = (-2*2*PI/3200)/((EndTime1 - StartTime1)/1000000);
    
  }else{
    
    EndTime1 = micros();
    counter1 = counter1 + 2;
    Position1 = (counter1*2*PI)/1600;
    AngularVelocity1 = (2*2*PI/3200)/((EndTime1 - StartTime1)/1000000);
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
    AngularVelocity2 = (-2*2*PI/3200)/((EndTime2 - StartTime2)/1000000);
  }
  StartTime2 = EndTime2;
}

void MotionController(){ // this should be all that is required to run the bots motion provided with a desired angle and forward velocity
  currTheta = r*(Position1 - Position2)/base; // where d is the distantce between the wheels
  errorTheta = desAngle-currTheta;
 
//  if(errorTheta <=upperThreshold || errorTheta >=lowerThreshold){
//    errorTheta = 0;
//  }
  
  if(Ts>0){
    D=(errorTheta-errorThetaPast)/Ts;
  }
  else{
    D=0;
  }
  
  desTurningRate= errorTheta*KpTheta+Kd*D;//set by another controller will figure out
  
  errorThetaPast=errorTheta;
  
  currForwardSpeed = r*(AngularVelocity1 + AngularVelocity2)/2;//some setPosition Bullshit

  currTurningRate =  r*(AngularVelocity1 - AngularVelocity2)/base;//stuff
  
  errorForwardSpeed= desForwardSpeed-currForwardSpeed;
  errorTurningSpeed = desTurningRate-currTurningRate;
  
  Vabs = errorForwardSpeed*KpRoh;
  Vdelta = errorTurningSpeed*KpPhi;

  Va1 = (Vabs+Vdelta)/2;
  Va2 = (Vabs-Vdelta)/2;
  
  PWMOutput1 = Va1*PWMperVolt;
  PWMOutput2 = Va2*PWMperVolt;
  

  if(abs(PWMOutput1)>175){
    PWMOutput1=constrain(PWMOutput1,-1,1)*175;
    errorForwardSpeed = constrain(errorForwardSpeed,-1,1)*min(175/KpRoh,abs(errorForwardSpeed));
  }
  if(abs(PWMOutput2)>175){
    PWMOutput2=constrain(PWMOutput2,-1,1)*175;
    errorTurningSpeed = constrain(errorTurningSpeed,-1,1)*min(175/KpPhi,abs(errorTurningSpeed));
  }
 
  if (PWMOutput1 >0){
    digitalWrite (VS1,HIGH);
  }else{
    digitalWrite(VS1,LOW);
  }
   if (PWMOutput1 >0){
    digitalWrite (VS2, HIGH);
  }else{
    digitalWrite(VS2,LOW);
  }
  //digitalWrite (VS1, HIGH);
  PWMOutput1 = abs(PWMOutput1);
  PWMOutput2 = abs(PWMOutput2);
  
  analogWrite(MV1, PWMOutput1);
  analogWrite(MV2, PWMOutput2);
  
  Ts=micros()-Tc;
  Tc=micros();

}
