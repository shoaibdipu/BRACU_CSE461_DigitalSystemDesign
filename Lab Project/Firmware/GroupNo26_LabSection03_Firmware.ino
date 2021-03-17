/*
 * CSE461 Lab Section 03
 * Group No : 26 [As Per Response Sheet]
 * 
 * Project Type : Hands On [Practical]
 * Project Title : Maze Solver & Android Based Bluetooth Controlled Robot 
 * 
 * Group Members :
 * Shoaib Ahmed Dipu
 * Mehadi Hassan
 * Shemonto Das
 * Nayem-Ul-Islam
 */


#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t sensor_count = 8;
uint16_t sensor_values[sensor_count];

/*
 * PID : Proportional–Integral–Derivative Controller Or Three-term Controller.
 * A control loop mechanism employing feedback.
 * Automatically applies accurate and responsive correction to a control function.
 */


float Kp = 3.0; // Proportional Term : Directly controls how to take the curves.
 
float Ki = 0.5; // Integral Term : Accumulates all errors. 
                // This seeks to eliminate the residual error by adding a control effect,
                // due to the historic cumulative value of the error.
               
float Kd = 0.1; // Derivative Term : Calculates the value between the current error and the last error.

int P;
int I;
int D;

int last_error = 0;

// Initialization of max speed and base speed for the motors 
const uint8_t maxspeeda = 60;
const uint8_t maxspeedb = 60;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

// Setting up the drive motor carrier pins
int bphasex = 7;
int bphase = 8;
int benbl = 6;

int aphase = 5;
int aphasex = 4;
int aenbl = 3;

int mode = 10;

char bluetooth_char;

boolean calibration_status = false;

void setup(){
  Serial.begin(9600);
  qtr.setTypeAnalog();
  
  //Set up the sensor array pins
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensor_count);
  qtr.setEmitterPin(2); 
  
  pinMode(aphasex, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(bphasex, OUTPUT);
  pinMode(benbl, OUTPUT);

  pinMode(mode, INPUT);
  
  delay(500);
  
  pinMode(LED_BUILTIN, OUTPUT);

  //forward_brake(0, 0);
}

void calibration(){
  digitalWrite(LED_BUILTIN, HIGH);
  
  for(uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  
  digitalWrite(LED_BUILTIN, LOW);
}


void forward_brake(int posa, int posb){
  // Setting up the values for the robot to move accordingly to solve the maze autonomously
  digitalWrite(aphase, LOW);
  digitalWrite(aphasex, HIGH);
  digitalWrite(bphase, HIGH);
  digitalWrite(bphasex, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

void forward(){
  // Setting the values for the robot to move forward
  digitalWrite(aphase, LOW);
  digitalWrite(aphasex, HIGH);
  digitalWrite(bphase, HIGH);
  digitalWrite(bphasex, LOW);
  analogWrite(aenbl, 100);
  analogWrite(benbl, 100);
}
void back(){
  // Setting the values for the robot to move backward
  digitalWrite(aphase, HIGH);
  digitalWrite(aphasex, LOW);
  digitalWrite(bphase, LOW);
  digitalWrite(bphasex, HIGH);
  analogWrite(aenbl, 100);
  analogWrite(benbl, 100);
}

void left(){
  // Setting the values for the robot to move left
  digitalWrite(aphase, LOW);
  digitalWrite(aphasex, HIGH);
  digitalWrite(bphase, HIGH);
  digitalWrite(bphasex, LOW);
  analogWrite(aenbl, 0);
  analogWrite(benbl, 100);
}

void right(){
  // Setting the values for the robot to move right
  digitalWrite(aphase, LOW);
  digitalWrite(aphasex, HIGH);
  digitalWrite(bphase, HIGH);
  digitalWrite(bphasex, LOW);
  analogWrite(aenbl, 100);
  analogWrite(benbl, 0);
}

void stops(){
  // Setting the values for the robot to stop moving
  digitalWrite(aphase, HIGH);
  digitalWrite(aphasex, LOW);
  digitalWrite(bphase, LOW);
  digitalWrite(bphasex, HIGH);
  analogWrite(aenbl, 0);
  analogWrite(benbl, 0);
}

/*
 * PID : Proportional–Integral–Derivative Controller Or Three-term Controller
 * A control loop mechanism employing feedback
 */

void PID_Control(){
  uint16_t position = qtr.readLineBlack(sensor_values);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - last_error;
  
  last_error = error;
  
  int motorspeed = (P * Kp) + (I * Ki) + (D * Kd);

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if(motorspeeda > maxspeeda){ motorspeeda = maxspeeda; }
  if(motorspeedb > maxspeedb){ motorspeedb = maxspeedb; }
  if(motorspeeda < 0){ motorspeeda = 0; }
  if(motorspeedb < 0){ motorspeedb = 0; }
    
  Serial.print(motorspeed); 
  Serial.print(" "); 
  Serial.print(motorspeeda); 
  Serial.print(" "); 
  Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
}

void loop(){
  int selected_mode = digitalRead(mode);
  
  if(selected_mode == 1){
    while(calibration_status == false){ 
      calibration(); 
      calibration_status = true;
    }
    
    PID_Control();
	
    Serial.println("Mode : Autonomous Maze Solver");
    
  }else{
    Serial.println("Mode : Android Based Bluetooth Controlled");
    
    if(Serial.available()){
      bluetooth_char = Serial.read();
      Serial.println(bluetooth_char);
      
      if(bluetooth_char == 'F'){ forward();
      }else if(bluetooth_char == 'B'){ back();
      }else if(bluetooth_char == 'L'){ left();   
      }else if(bluetooth_char == 'R'){ right(); 
      }else if(bluetooth_char == 'S'){ stops(); 
      }
    }
  }
}
