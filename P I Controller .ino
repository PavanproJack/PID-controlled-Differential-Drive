//#include <Romi32U4.h>
//Romi32U4Encoders encoders;


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15


int weightsArray[] = {1000, 2000, 3000};
float setPoint = 1900;
int maxSpeed = 30;
int minSpeed = 21;
int right_speed = 0;
int left_speed = 0;

double Kp = 0.005;
double Ki = 0.0002;
double Kd = 3.5;
double proportional = 0.0;
double errorIntegral = 0.0;
double errorDerivative = 0.0;
float lastError = 0.0;
float mP = 0.1;


float angleDeg, angleRad;
float xProj, yProj;
float theta = 0.0;
float distancePerCount = 0.015;
int wheelDistance = 14;
float meanDistance;

float d = 0.0;
float finalRightMotor = 0.0;

void setup() {

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  moveForward();
  Serial.begin(9600);
  //delay(1500);
  //Serial.println("Calibrating the set point ....");
  /*while(1){
      motor_drive(minSpeed, minSpeed);
      moveForward();
    }*/
}
 
 
void loop(){
                                                                //Serial.println("Weighted Gain is : " + String(calcWeightedMean()));
  if(identifiedWhite()){ //getValues();
      moveForward();
      motor_drive(30, 30);
    }
    else{
                                                                //Serial.println("BLACK IDENTIFIED");
        //motor_drive(0, 0);
      
        double error = calcWeightedMean() - setPoint;
                                                                //Serial.println("Weighted Gain Error is : " + String(error));
        double controlInput = calcPidGain(error);
        
                                                                //Serial.println("Control Input is : " + String(controlInput)); 
        calcNewTurn(controlInput);                                                       
        //calc_turn(pidErr);
    }
     //delay(1000);
}

void calcNewTurn(double inputVal){
    //double inputVal = mP * inputVal;
    Serial.println("Newspeed  is : " + String(inputVal)); 
    //delay(500);
    if(inputVal < 0){
        moveLeft();
        analogWrite( L_PWM_PIN, minSpeed - inputVal);
        analogWrite( R_PWM_PIN, minSpeed - inputVal);
      }else if(inputVal > 0){
          moveRight();
          analogWrite( L_PWM_PIN, minSpeed + inputVal);
          analogWrite( R_PWM_PIN, minSpeed + inputVal);
        }
        else{
          moveForward();
          analogWrite( L_PWM_PIN, minSpeed);
          analogWrite( R_PWM_PIN, minSpeed);
        }
  }
  
void moveForward(){
  //Serial.println("Forward");
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, LOW );
  }
  void moveLeft(){
     Serial.println("Moving Left");
     digitalWrite( L_DIR_PIN, HIGH  );
     digitalWrite( R_DIR_PIN, LOW );
     //getValues();
     //delay(200);
  }
  void moveRight(){
    Serial.println("Moving Right");
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, HIGH );
     //delay(200);
  }

void motor_drive(int rs, int ls){                                        // Drive motors according to the calculated values for a turn
  analogWrite(L_PWM_PIN, ls);
  analogWrite(R_PWM_PIN, rs);
  //delay(50); // Optional
}


int calcPidGain(float currentErr){
                                                                         //Serial.println("Current Error is : " + String(currentErr));
                                                                         //Serial.println("Error integral is : " + String(errorIntegral));
                                                                         //Serial.println("Error derivative is : " + String(errorDerivative));
    proportional = currentErr;
    errorIntegral =  currentErr + errorIntegral;
    //errorDerivative =  currentErr - lastError;
    //lastError = currentErr;
                                                                         //Serial.println("errorDerivative is : " + String(errorDerivative));
  return int(proportional * Kp + errorIntegral * Ki + errorDerivative * Kd);
 }


void calc_turn(int error_value){ 
  if (error_value< (-1 * maxSpeed) ){
      error_value = -1 * maxSpeed;
   }
  if (error_value>= maxSpeed){
      error_value = maxSpeed;
   }
   // If error_value is less than zero calculate right turn speed values
  if (error_value< 0){
      left_speed = maxSpeed + error_value;
      right_speed = maxSpeed;
   }
   // Iferror_value is greater than zero calculate left turn values
  else{
      left_speed = maxSpeed; 
      right_speed = maxSpeed - error_value;
   }
   motor_drive(right_speed, left_speed);
}


float calcWeightedMean(){
   float lS =  analogRead(A4);  // left sensor towards pin 5
   float mS =  analogRead(A3); // middle sensor
   float rS =  analogRead(A2); // right sensor towards ground 
   float pos = ( weightsArray[0] * lS + weightsArray[1] * mS + weightsArray[2] * rS ) / (lS + mS + rS);
   return pos;
  }

bool identifiedWhite(){
    return (analogRead(A4) < 550) and (analogRead(A3) < 300) and (analogRead(A2) < 550) ;
  }


void getValues(){
  //Serial.println("Left: " + String(analogRead(A4)) + "\t" +  "Middle : " + "\t" + String(analogRead(A3)) + "Right: " +"\t" + String(analogRead(A2)));
  }
