
//
//
//RyanReschak
//Final Demo
//MWehrlen added motor driver code in place of previous prototypes
//@3rikSal@MWehrlen
//265 lines (217 sloc)  7.27 KB
#define SLAVE_ADDRESS 0x04
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

int currState = -1; //reason is to update at start
int desiredState = 0; 

long int counter1 = 0; //keeps track of position of encoder
long int counter2 = 0;
const float r = 7.3;  //these values are the constants of the body of the car and wheels in (cm)
const float base = 24.45; //in cm
float circumference = 39; //Don't fucking change this is fine

const float KpTheta = 18.0944;
const float Kd = 0;
const float KpRoh = 0.47634;
const float KiRoh = 0;
const float KpPhi = 5.234;
const float KiPhi = 0;

enum moving {SEARCHING,CALIBRATE,TURNING,FORWARD,TURN90,REVERSE,CIRCLE,WAIT};
moving movingState = SEARCHING;

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
float upperThreshold = 0.005;
float phi_dot = 0;
float Position1 = 0;
float Position2 = 0;
float setPosition = 0;
float desAngle = 0;
float desAnglePast=0;
float desForwardSpeed =10;
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
float IRoh=0;
float IPhi=0;
int PWMOutput1 = 0;
int PWMOutput2 = 0;
int turnCount=0;
boolean done  = false;
boolean newVal = false;
boolean sendY = false;
boolean firstCircle = false;
int numLoops=0;
static unsigned int state;

//Serial Communication
float receivedPosition = 0;
float receivedAngle = 0;
bool receivedFirstComplete = false;

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

  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
    move();
    // MotionController();
    /*switch(state){
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
        if((r*(Position1+Position2)/2 >= setPosition)){
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
    }*/
}


void move(){
  switch(movingState){// switch case to determine state of movement of the bot
  case SEARCHING://looking for the marker (waiting for serial event)
    //Serial.println(movingState);
    
    if(receivedPosition!=0){//if there is a desired position to move to
      
      movingState=CALIBRATE; // go to turning state
      //set new angle and position
//      setPosition = receivedPosition-20;
//      if(receivedAngle>0){
//        desAngle=receivedAngle-.03;
//      }else{
//        desAngle=receivedAngle+.03;
//      }
      desAngle=receivedAngle;
      desAnglePast=receivedAngle;
      
      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
      
    }else{//turn slowly in place
      //digitalWrite (VS1,HIGH);
      digitalWrite (VS2,LOW);

      //Va1 = 2.25;
      //Va2 = 2.75;
      Va1 = 1.5;
      Va2 = 2;
    }
    break;

  case CALIBRATE:
    Va1 = 0;
    Va2 = 0;
    newVal=false;
    //if(turnCount<=1){
      
      desAngle=receivedAngle;
      //setPosition=receivedPosition-40;
      setPosition=receivedPosition-38;
      if(desAnglePast!=desAngle){
        movingState=TURNING;
        
        //turnCount++;
      }
    
//    }else{
//      movingState=FORWARD;
//    }


    break;
    
  case TURNING://turning to face marker
    //Serial.println(movingState);
    //establish new error in angle positioning
    currTheta = r*(Position1 - Position2)/base; // where d is the distantce between the wheels
    errorTheta = desAngle-currTheta;
    //if there is no error in the angle or no set angle
    if(((errorTheta <= upperThreshold) && desAngle > 0)||((errorTheta >= upperThreshold) && desAngle < 0)||desAngle==0){
      movingState=FORWARD;

      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
      
    }else{//turn in place until desired angle is reached 
      if(desAngle>0){//(right)
        digitalWrite (VS1,HIGH);
        digitalWrite (VS2,HIGH);
        Va1 = 3;
        Va2 = 3;
        
      }else{//(left)
        digitalWrite (VS1,LOW);
        digitalWrite (VS2,LOW);
        Va1 = 2;
        Va2 = 2;
      }
    }
    break;
  
  case FORWARD://moving towards the marker
    //Serial.println(movingState);
    //set enable pins for forward motion
    digitalWrite (VS1,HIGH);
    digitalWrite (VS2,LOW);
    if((Position1+Position2)/2*r >= setPosition){// if the desired positon has been reached
      movingState=TURN90;//change moving state to circle the marker
      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
      
    }else{//move forward
      Va1 = 4;
      Va2 = 4.3;
    }
    break;

  case TURN90:
    desAngle=-PI/2;
    //Serial.println(movingState);
    //establish new error in angle positioning
    currTheta = r*(Position1 - Position2)/base; // where d is the distantce between the wheels
    errorTheta = desAngle-currTheta;
    //if there is no error in the angle or no set angle
    if(((errorTheta <= upperThreshold) && desAngle > 0)||((errorTheta >= upperThreshold) && desAngle < 0)||desAngle==0){
      movingState=CIRCLE;
      firstCircle = true;
      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
      
    }else{//turn in place until desired angle is reached 
      if(desAngle>0){//(right)
        digitalWrite (VS1,HIGH);
        digitalWrite (VS2,HIGH);
        Va1 = 3;
        Va2 = 3.2;
        
      }else{//(left)
        digitalWrite (VS1,LOW);
        digitalWrite (VS2,LOW);
        Va1 = 2;
        Va2 = 2;
      }
    }
    break;
   
    
  case CIRCLE://circling around the marker
    //This is preliminary before it starts to circle (only runs once when called)
    if (firstCircle) {
      //does the serial delay with a send to start capturing again
      sendY = true;
      firstCircle = false;
      //If this is the last run, do the quarter circle
      if(numLoops>=5) { //final needs to be 5
        circumference = circumference/4;
      }
    }
    //Serial.println(movingState);
    if(Position1>=circumference){//if the bot has traveled the desired circumference
      movingState=WAIT;//change moving state to wair
      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
    } 
    else if(newVal){
      movingState=CALIBRATE;//change moving state to wait
      desAngle=receivedAngle;
      desAnglePast=receivedAngle;
      //reinitilize the variables
      Position1 = 0;
      Position2 = 0;
      counter1 = 0;
      counter2 = 0;
      numLoops++;
      
    }else{//turn in a circle
      digitalWrite (VS1,HIGH);
      digitalWrite (VS2, LOW);
    
      //Va1 = 4;
      //Va2 = 1.8;
      Va1 = 3;
      Va2 = 1.3;
    }

//      movingState=WAIT;
    
    break;
    
  case WAIT://this is the end state and nothing is done here right now
  //Serial.println(movingState);
  
    Va1 = 0;
    Va2 = 0;
    break;
  }
  //convert to PWM values from listed volts
  PWMOutput1 = Va1*PWMperVolt;
  PWMOutput2 = Va2*PWMperVolt;
  //send relevant output values to pins
  analogWrite(MV1, PWMOutput1);
  analogWrite(MV2, PWMOutput2);

  if (sendY) {
    delay(1000);
    //while(Position1<=circumference/5);
    Serial.println("Y");
    sendY = false;
  }
}


void serialEvent(){
  //Useful for testing without RPI comment everything else out if this is true
  //receivedAngle = Serial.parseFloat();
  //receivedPosition = Serial.parseFloat();
  
  String sAngle = "";
  String sPos = "";
  bool firstString = false;
  String data;
  newVal=true;
  while(Serial.available() > 0){
    data = Serial.readStringUntil('\n');
  }
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ' ') {
      firstString = true;
    } else if (!firstString){
      sAngle += data[i];
    } else if (firstString) {
      sPos += data[i];
    }
  }
  
  receivedAngle = sAngle.toFloat();
  receivedPosition = sPos.toFloat();
  
  //desiredState = FORWARD;
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



//void MotionController(){ // this should be all that is required to run the bots motion provided with a desired angle and forward velocity
//  currTheta = r*(Position1 - Position2)/base; // where d is the distantce between the wheels
//  errorTheta = desAngle-currTheta;
// 
////  if(errorTheta <=upperThreshold || errorTheta >=lowerThreshold){
////    errorTheta = 0;
////  }
//  
//  if(Ts>0){
//    D=(errorTheta-errorThetaPast)/Ts;
//  }
//  else{
//    D=0;
//  }
//
////  IRoh = IRoh+Ts*errorForwardSpeed;
////  IPhi = IPhi+Ts*errorTuringSpeed;
//  
//  desTurningRate= errorTheta*KpTheta+Kd*D;//set by another controller will figure out
//  
//  errorThetaPast=errorTheta;
//  
//  currForwardSpeed = r*(AngularVelocity1 + AngularVelocity2)/2;//some setPosition Bullshit
//  Serial.println(desForwardSpeed);
//  currTurningRate =  r*(AngularVelocity1 - AngularVelocity2)/base;//stuff
//  //Serial.println(currForwardSpeed);
//  errorForwardSpeed= desForwardSpeed-currForwardSpeed;
//  //Serial.println(errorForwardSpeed);
//  errorTurningSpeed = desTurningRate-currTurningRate;
//  
//  Vabs = errorForwardSpeed*KpRoh;
//  Vdelta = errorTurningSpeed*KpPhi;
//
////  Vabs = errorForwardSpeed*KpRoh+KiRoh*IRoh;
////  Vdelta = errorTurningSpeed*KpPhi+KiPhi*IPhi;
//
//  Va1 = (Vabs+Vdelta)/2;
//  Va2 = (Vabs-Vdelta)/2;
//  
//  PWMOutput1 = Va1*PWMperVolt;
//  PWMOutput2 = Va2*PWMperVolt;
//  
//
//  if(abs(PWMOutput1)>175){
//    PWMOutput1=constrain(PWMOutput1,-1,1)*175;
//    errorForwardSpeed = constrain(errorForwardSpeed,-1,1)*min(175/KpRoh,abs(errorForwardSpeed));
//  }
//  if(abs(PWMOutput2)>175){
//    PWMOutput2=constrain(PWMOutput2,-1,1)*175;
//    errorTurningSpeed = constrain(errorTurningSpeed,-1,1)*min(175/KpPhi,abs(errorTurningSpeed));
//  }
// 
//  if (PWMOutput1 >0){
//    digitalWrite (VS1,HIGH);
//  }else{
//    digitalWrite(VS1,LOW);
//  }
//   if (PWMOutput1 >0){
//    digitalWrite (VS2, HIGH);
//  }else{
//    digitalWrite(VS2,LOW);
//  }
//  //digitalWrite (VS1, HIGH);
//  PWMOutput1 = abs(PWMOutput1);
//  PWMOutput2 = abs(PWMOutput2);
//  
//  analogWrite(MV1, PWMOutput1);
//  analogWrite(MV2, PWMOutput2);
//  
//  Ts=micros()-Tc;
//  Tc=micros();
//
//}
