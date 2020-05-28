/*
This code is submitted as a part of Robotics Systems Assessment May 2020.

This code uses open source pololu/romi-32u4-arduino-library available at https://github.com/pololu/romi-32u4-arduino-library

Robot moves forward in white space until it encounters a black line. It folows the black line 
and when it dentifies line break with high confifdence, it return home. Through out this process,
odometry calculations are carried out in loop.

Optimal performance with PID is achieved with constants Kp 0.005, Kd 1.0 and Ki 0.0002.

On white space - turn = 0
After Line break - turn = 1
After turn to 180 degrees  - turn = 2
After turn to 90 degrees   - turn = 3
After reaching home  - turn = 4

The variable 'count' counts the local white spaces while being on line and frequently set to zero after 
encountering black. At the line break, this count crosses 700 as no black can be seen further indicating
the robot has reached terminus.


*/



#include <Romi32U4.h>
#define BUZZER_PIN  6

Romi32U4Encoders encoders;

float angleDeg, angleRad;
float xProj, yProj;
float theta = 0.0;
float distancePerCount = 0.015;
int wheelDistance = 14;
float meanDistance;
float finalX = 0.0;
float finalY = 0.0;
float finalTheta = 0.0;
int turn = 0;
int16_t count_left_e;
int16_t count_right_e;


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

int weightsArray[] = {1000, 2000, 3000};
float setPoint = 1900;
int maxSpeed = 30;
int minSpeed = 30;
int right_speed = 0;
int left_speed = 0;

double Kp = 0.005;
double Ki = 0.0002;
double Kd = 1.0;
double proportional = 0.0;
double errorIntegral = 0.0;
double errorDerivative = 0.0;
float lastError = 0.0;
float mP = 0.1;



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
  
  Serial.begin(9600);

/*
While on white space move forward and undergo the distance calculations with calcOdometry() function.
*/
  while(identifiedWhite()){ 
    Serial.println("White...");
      moveForward();
      motor_drive(30, 30);
      calcOdometry(encoders.getCountsLeft(), encoders.getCountsRight(), turn);
      delay(600);
    }
}

int count = 0;

/*

*/
 
void loop(){
//Serial.println("Turn:" + String(turn) + "Count:" + String(count));

if(turn == 0){
    if(count >= 700){
        stopMe(1);    
        play_tone();
    }
    else{
        if(identifiedWhite()){
            count = count + 1;
        }else{
            double error = calcWeightedMean() - setPoint;
            double controlInput = calcPidGain(error);
            calcNewTurn(controlInput); 
            count = 0;
        }
     }
  }

  if(turn == 4){
      exit(0);  // or do nothing...as we have reached home
    }
  
     
    calcOdometry(encoders.getCountsLeft(), encoders.getCountsRight(), turn);
}



void calcOdometry(int16_t encoderLPos, int16_t encoderRPos, int turn){
  
    static int encoderRPosPrev = 0;
    static int encoderLPosPrev = 0;

    float SR = distancePerCount * (encoderRPos - encoderRPosPrev);
    float SL = distancePerCount * (encoderLPos - encoderLPosPrev);

    encoderRPosPrev = encoderRPos;
    encoderLPosPrev = encoderLPos;
    

    theta += (SR - SL) / wheelDistance;

    if(theta > 6.28)
      theta -= 6.28;
    else if(theta < -6.28)
      theta += 6.28;
   
     Serial.println("Deg : " + String(theta * 57.2958) );
        meanDistance = (SL + SR)/2;
        xProj = xProj + meanDistance*cos(theta);
        yProj = yProj + meanDistance*sin(theta);

      if(turn == 0){
           finalX = xProj;
           finalY = yProj;
           finalTheta = theta;
           /*Serial.println(
                      "Final Theta : " + String(finalTheta * 57.2958) + "\t" + 
                      "final X : " + String(xProj) + "\t" + 
                      "final Y : " + String(yProj)
                      )*/
        }
      else if(turn == 1){   // Turn 180 degrees opposite to start location
        
           float t = theta * 57.2958;
           double difference = -180 - t;
           //Serial.println("Inside Turn 1");
           Serial.println("Difference Angle is : " + String(difference));

           if(turnToAngle(difference)){
                Serial.println("Stopping");
                resetEncoders();
                encoderRPosPrev = 0;
                encoderLPosPrev = 0;
                xProj = 0;
                yProj = 0;
                theta = 0;
                stopMe(2);
                delay(2000);
            }
                     
        }else if(turn == 2){ 
            
          Serial.println("Difference X is : " + String(float(finalX - xProj)));

          int xSpeed = map(int(finalX - xProj), 0, finalX, 14, 30);
          
          if(int(finalX - xProj) <= 0){
              stopMe(3);
            }else{   
              Serial.println("Moving Forward");
                moveForward();
                motor_drive(xSpeed, xSpeed);
                delay(1700);
              }
          }else if(turn == 3){  
              Serial.println("Inside Turn 3");
              // Now turn 90 deg to travel in y direction...

              int ySpeed = map(int(finalY - yProj), 0, finalY, 14, 30);
              float t = theta * 57.2958;
              double difference = -90 - t;
              
              if(turnToAngle(difference)){
                  if(int(finalY - yProj) <= 0){
                        stopMe(4);
                    }else{
                        moveForward();
                        motor_drive(ySpeed, ySpeed);
                        delay(1700);
                      }
                }
            }
  }



 int turnToAngle(double difference){
  
            if(difference > 0){
                      moveLeft();
                      analogWrite( L_PWM_PIN, 20);
                      analogWrite( R_PWM_PIN, 20);
                      return false;
                  }else if(difference < 0){
                      moveRight();
                      //Serial.println("Moving Right");
                      analogWrite( L_PWM_PIN, 20);
                      analogWrite( R_PWM_PIN, 20);
                      return false;
                    }
                    else{
                      return true;
                      }
  }



void stopMe(int turnFlag){
      analogWrite( L_PWM_PIN, 0);
      analogWrite( R_PWM_PIN, 0);
      turn = turnFlag;
      Serial.println(
                      "Final Theta : " + String(finalTheta * 57.2958) + "\t" + 
                      "final X : " + String(finalX) + "\t" + 
                      "final Y : " + String(finalY)
                      );
    
                      delay(3000);
    }

void resetEncoders(){
    Serial.println("Resetting Encoders");
    int RC = encoders.getCountsAndResetRight();
    int LC = encoders.getCountsAndResetLeft();
    Serial.println(LC);Serial.println(RC);
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

void moveForward(){
  //Serial.println("Forward");
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, LOW );

  }
  void moveLeft(){
     //Serial.println("Moving Left");
     digitalWrite( L_DIR_PIN, HIGH  );
     digitalWrite( R_DIR_PIN, LOW );
  
     
  }
  void moveRight(){
    //Serial.println("Moving Right");
     digitalWrite( L_DIR_PIN, LOW  );
     digitalWrite( R_DIR_PIN, HIGH );

     
  }

void motor_drive(int rs, int ls){                                        // Drive motors according to the calculated values for a turn
  analogWrite(L_PWM_PIN, ls);
  analogWrite(R_PWM_PIN, rs);
  //delay(50); // Optional
}

void calcNewTurn(double inputVal){
    //double inputVal = mP * inputVal;
    //Serial.println("Newspeed  is : " + String(inputVal)); 
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

int calcPidGain(float currentErr){
  
    proportional = currentErr;
    errorIntegral =  currentErr + errorIntegral;
    errorDerivative =  currentErr - lastError;
    lastError = currentErr;
    
    return int(proportional * Kp + errorIntegral * Ki + errorDerivative * Kd);
 }


 
void play_tone()
{
for (int i=0; i<=100; i++)
{
analogWrite(BUZZER_PIN, 10);
delay(5);
analogWrite(BUZZER_PIN, 0);
}
delay(100);
for (int i=0; i<=100; i++)
{
analogWrite(BUZZER_PIN, 10);
delay(5);
analogWrite(BUZZER_PIN, 0);
}
}
