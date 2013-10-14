// Code, combination of Ash and Ray's work, 2:30pm

#include <Servo.h> 

vaerera
// Speed Changes
const int M1pwm = 190; // right //190
const int M2pwm = 160; // left //160
const int dur = 20; // duration of pulse in milliseconds for all LEDs
// Pulsing variables
int k = 0;

// Digital Pin Allocations
const int pulse = 35;
const int pulBALLb = 33;
const int pulBALLr = 31; // Red LED pin number
const int frontButton[] = {26, 29, 28, 27}; 
const int TeamColour = 34;
const int RedTeamLED = 32;
const int BlueTeamLED = 30;
const int Motor1Pin1 = 23;
const int Motor1Pin2 = 25;
const int Motor2Pin1 = 22;
const int Motor2Pin2 = 24;
const int LSclock = 12;
const int LineSensorPin = 15;

// PWM Pin Allocations
const int Motor1Enable = 2;
const int Motor2Enable = 3;
Servo kicker; // allocate within setup as pin 4
Servo backdoor; // allocate within setup as pin 5
Servo frontdoor; // allocate within setup as pin 6

// Analog Pin Allocations
const int IR[] = {A0, A4, A2, A6}; // define what idex is what location [front left, front right, back left, back right]
const int RED[] = {A1, A5, A3, A7}; //define what index is what location
const int dataPin = A9; // Line Sensor Data Pin

// LED pulsing times
long LED_prevTime = 0;
int LEDstate = LOW;
int IR_on[4] = {0,0,0,0};
int IR_off[4] = {0,0,0,0};
int IR_net[4] = {0,0,0,0};
int RED_on[4] = {0,0,0,0};
int RED_off[4] = {0,0,0,0};
int RED_net[4] = {0,0,0,0};


int GoToGoal = LOW;
long SerialWritePrevTime;

int BALL = A8; // define what idex is what location
unsigned long PrevStopTime = 0;
unsigned long PrevRedTime = 0;
int RedGate = LOW;
unsigned long PrevBlueTime = 0;


unsigned long BALL_prevTime = 0;
int BALLstateB = LOW;
int BALLstateR = LOW;
int BALL_onB = 0;
int BALL_onR = 0;
int BALL_off = 0;
int Ball_Blue_Net = 0;
int Ball_RED_Net = 0;
int reading[2] = {0,0};


// touch sensors buttons

int buttonState;
int lastButtonState = LOW;

long lastDebounceTime = 0; 
long debounceDelay = 50; 
int reverse = LOW;
long lastReverseTime = 0;

int right = LOW;
long lastRightTime = 0;

// motor variables



//unsigned long Mtime;
int Mstate;
int Mtemp = 0;

int Direction;
float theta0=11.25;
float theta;             

const int kicker_rest = 175;
const int kicker_kick = 70;
const int backdoor_open = 90;
const int backdoor_closed = 160;
const int frontdoor_open = 0;
const int frontdoor_closed = 80;
unsigned long servoSTOP = 0;


// Additional Variables which have been added in

// Phototransistor Sensor Parameters
int IR_temp[4] = {0,0,0,0}; // Temporary counter, must count to 3
int IR_X[4] = {0,0,0,0}; // Says whether the robot is at a wall
int IR_thresh[4] = {250,300,250,140}; // IR threshold values
int RED_temp[4] = {0,0,0,0}; // Temporary counter, must count to 3
int RED_X[4] = {0,0,0,0};  // Says whether the robot is on the green mat
int RED_thresh[4] = {-1,200,120,175}; // Red threshold values


// Ball Parameters
int BCol = 0; // 1 = Blue, 2 = Red
int TCol = 1; // 1 = Blue, 2 = Red
int BCol_Temp=0;
const int BallBlueThresh = 600;
const int BallRedThresh = 450;
int BallInHand = 0;

// Motor Parameters
unsigned long Mtime = 0; // Motor Time
const int Mdur = 200; // Update speed for the motors

// Additional Servo Parameters
int backdoor_trigger = 1;
unsigned long backtime = 0;

// Goal Finding Parameters
int GoalStage = 0;
int WallSide = 0;

// Stall Control Parameters
int buttonPrev[4] = {0,0,0,0};
int IRnetPrev[4] = {0,0,0,0};
unsigned long StallTimeTouch = 0;
unsigned long StallTimeIR = 0;
const int SdurTouch = 500;
const int SdurIR = 200;
const int Sdur = 100;
int ScountTouch = 0;
int ScountIR = 0;
const int IRStallThresh = 20;
int Sdir = 0; // Current direction of motion of robot. 0 = stop, 1 = forward, 2 = left, 3 = right, 4 = backwards

// Line Sensor Parameter Values
int lightVal[130];
int LSj;
int del = 5;
unsigned long LStime = 0;  
const int LSdur = 500;


void setup()
{
  Serial.begin(115200);

  pinMode(LSclock, OUTPUT);
  pinMode(LineSensorPin, OUTPUT);

  kicker.attach(4);
  kicker.write(kicker_rest);
  backdoor.attach(5);
  backdoor.write(backdoor_closed);
  frontdoor.attach(6);
  frontdoor.write(frontdoor_open);
  
 
  for (k=0; k<4; k++) { 
  pinMode(IR[k],INPUT);
  pinMode(RED[k], INPUT);
  pinMode(frontButton[k], INPUT);
  }
  pinMode(pulse, OUTPUT);
  
  pinMode(BALL, INPUT);
  pinMode(pulBALLb, OUTPUT);
  pinMode(pulBALLr, OUTPUT);
  

  pinMode(TeamColour, INPUT);
  pinMode(BlueTeamLED,OUTPUT);
  pinMode(RedTeamLED,OUTPUT);
  pinMode(Motor1Pin1, OUTPUT);   
  pinMode(Motor1Pin2, OUTPUT);   
  pinMode(Motor2Pin1, OUTPUT);   
  pinMode(Motor2Pin2, OUTPUT);

  pinMode(Motor1Enable, OUTPUT);   
  pinMode(Motor2Enable, OUTPUT); 
  
  
  Mstate = 1;
 
} 
void loop(){
	// make sure the kicker is in it's resting position
	kicker.write(kicker_rest);
// change to first 20 seconds will read in team colour and then wont read.
	if (millis() < 20000) {
        if (digitalRead(TeamColour)) { TCol = 1; digitalWrite(BlueTeamLED, HIGH); digitalWrite(RedTeamLED, LOW); }
        else if (digitalRead(!TeamColour)) { TCol = 2; digitalWrite(RedTeamLED, HIGH); digitalWrite(BlueTeamLED, LOW); }
    }
        
	// Perform all sensing
	REDandIR_Sense();
	Ball_Colour(); 

  //   if ( (millis() - SerialWritePrevTime) > 200) {
		// SerialWritePrevTime = millis(); 
  //     //  IR_diagALL();
  //     //  IR_diagNet();
  //     //  RED_diagNet();
  //     //  RED_diagALL();
  //     //  Button_diag();
  //       // BallColor_diag();
  //      BallColor_diagNet();
  //         Serial.println(" ");
  //       }

	// Write motor PWM
	analogWrite(Motor1Enable, M1pwm); // DO WE NEED TO WRITE THIS CONTINUOUSLY? COULDN'T WE DO IT ONCE AT THE START? 
	analogWrite(Motor2Enable, M2pwm);

	if (BCol == TCol) {
		frontdoor.write(frontdoor_closed);
        backdoor.write(backdoor_closed);
		GoalTimeBitches();
	} else if (BallInHand == 1) { CatchAndRelease() };
	else StandardDrive();
		// No Ball Caught Condition
	}
	

}

void GoForward(){
	Sdir = 1;
  digitalWrite(Motor1Pin2, LOW); //LOW
  digitalWrite(Motor1Pin1, HIGH); //HIGH
  digitalWrite(Motor2Pin2, LOW); //LOW
  digitalWrite(Motor2Pin1, HIGH); //HIGH
}
void GoBackward(){
	Sdir = 4;
  digitalWrite(Motor1Pin1, LOW); //LOW
  digitalWrite(Motor1Pin2, HIGH); //HIGH
  digitalWrite(Motor2Pin1, LOW); //LOW
  digitalWrite(Motor2Pin2, HIGH); //HIGH
}
void GoLeft(){
	Sdir = 2;
  digitalWrite(Motor1Pin1, LOW); //LOW
  digitalWrite(Motor1Pin2, HIGH); //HIGH
  digitalWrite(Motor2Pin2, LOW); //LOW
  digitalWrite(Motor2Pin1, HIGH); //HIGH
}
void GoRight(){
	Sdir = 3;
  digitalWrite(Motor1Pin2, LOW); //LOW
  digitalWrite(Motor1Pin1, HIGH); //HIGH
  digitalWrite(Motor2Pin1, LOW); //LOW
  digitalWrite(Motor2Pin2, HIGH);  //HIGH
}
void Stop(){
	Sdir = 0;
  digitalWrite(Motor1Pin2, LOW); //LOW
  digitalWrite(Motor1Pin1, LOW); //LOW
  digitalWrite(Motor2Pin1, LOW); //LOW
  digitalWrite(Motor2Pin2, LOW); //LOW
}
// Diagnostic Algorithms
void IR_diagNet() {
    for (k=0; k<4; k++) {
  Serial.print("IR: ");
  Serial.print(IR_net[k]);
  Serial.print(" | ");
  }
}
void IR_diagALL() {
    for (k=0; k<4; k++) {
  Serial.print("IR_on: ");
  Serial.print(IR_on[k]);
  Serial.print(" | ");
  
  Serial.print("IR_off: ");
  Serial.print(IR_off[k]);
  Serial.print(" | ");
  
  Serial.print("IR_net: ");
  Serial.print(IR_net[k]);
  Serial.print(" | ");
  }
}
void RED_diagNet() {
    for (k=0; k<4; k++) {
  Serial.print("R: ");
  Serial.print(RED_net[k]);
  Serial.print(" | ");
  }
}
void RED_diagALL() {
    for (k=0; k<4; k++) {
  Serial.print("R_on: ");
  Serial.print(RED_on[k]);
  Serial.print(" | ");
  
  Serial.print("R_off: ");
  Serial.print(RED_off[k]);
  Serial.print(" | ");
  
  Serial.print("R_net: ");
  Serial.print(RED_net[k]);
  Serial.print(" || ");
  }
}
void Button_diag() {
    for (k=0; k<4; k++) {
  Serial.print("But: ");
  Serial.print(digitalRead(frontButton[k]));
  Serial.print(" | ");
  }
}
void BallColor_diag() {
  Serial.print("RedBall_on: ");
  Serial.print(BALL_onR);
  Serial.print(" | ");
  
  Serial.print("BlueBall_on: ");
  Serial.print(BALL_onB);
  Serial.print(" | ");
  
  Serial.print("off: ");
  Serial.print(BALL_off);
  Serial.print(" | ");
  
  Serial.print("RedBall_net: ");
  Serial.print(Ball_RED_Net);
  Serial.print(" | ");
  
  Serial.print("BlueBall_net: ");
  Serial.print(Ball_Blue_Net);
  Serial.print(" || ");
}
void BallColor_diagNet() {
  Serial.print("RC: ");
  Serial.print(Ball_RED_Net);
  Serial.print(" | ");
  
  Serial.print("BC: ");
  Serial.print(Ball_Blue_Net);
  Serial.print(" || ");
}
// Driving Algorithms
void CatchAndRelease(){
	// In here we open both front and back doors to let out the captured ball
	if (backdoor_trigger){
		backdoor.write(backdoor_open);
		GoForward();
		frontdoor.write(frontdoor_closed);
		backtime = millis();
		backdoor_trigger = 0;
	} // end if (backdoor_trigger)

	// Here we close the back door again, reading the robot once more
	else{ 
		if (millis() - backtime > 3000){
			backdoor.write(backdoor_closed);
			frontdoor.write(frontdoor_open);
			backdoor_trigger = 1;
			BallInHand = 0;
			BCol = 0;
			BCol_Temp = 0;
		}
	}
}
void StandardDrive(){
	if (millis() - Mtime > Mdur){ // This stops damage to the motor

		// Change direction based on IR readings
		if (IR_X[0]||IR_X[2]) GoRight();
                else if (digitalRead(frontButton[0])) {
                        GoBackward();
                        delay(400); // change delays to better code!!!!!!!!!!!!
                        GoLeft();
                        delay(250);
                      }
                      
                else if (digitalRead(frontButton[1])) {
                        GoBackward();
                        delay(400);
                        GoLeft();
                        delay(250);
                      }
                
                
		else if (IR_X[1]||IR_X[3]) GoLeft();
		
		// Change of direction based on Green Mat Readings
		else if ((RED_net[0] < RED_thresh[0])||(RED_net[2] < RED_thresh[2])) GoRight();
		else if ((RED_net[1] < RED_thresh[1])||(RED_net[3] < RED_thresh[3])) GoLeft();

		// Otherwise continue as per normal
		else GoForward();

		// Reset Motor Time Counter
		Mtime = millis();
	}  // end if (millis() - Mtime > Mdur)

}
void GoalTimeBitches(){
	switch(GoalStage){ // change the case number iterations below in wall find and wall follow or just incorporate this case into the below one.
        case 0: WallFind();
		break;
		case 1: WallFollow();
		break;
		case 2: GreenMat();
		break;
		case 3: KICK();
		break;
	}

}
void WallFind(){

	int x = -1;
	int y = -1;
	int z = -1;
	for (int k=0;k<4;k++){
		if(IR_X[k]){
			x=k;
			break;
		}
	}
	for (int k=0;k<4;k++){
		if(RED_net[k] < RED_thresh[k]){
			y=k;
			break;
		}
	}
	for (int k=2;k<4;k++)
		if (digitalRead(frontButton[k])){
			z=k;
			break;
		}

	if ((y < 0) && (x < 0)){
		if (z < 0)
			GoBackward();
		else if (z==2){
			GoForward();
			delay(500);
			GoLeft();
			delay(500);
			GoalStage = 1;
		} else {
			GoForward();
			delay(500);
			GoRight();
			delay(500);
			GoalStage = 1;
		}
	}
	else{
		if (y==2||y==3){ // Back over the green
			GoForward();
			Mtemp = 1;
			GoalStage = 2;
		}else if (y==0||y==1){
			GoBackward();
		}
		if (x==0||x==2){ // Wall on left side
			GoBackward();
			WallSide = 0;
			GoalStage = 1;
		} else {
			GoBackward();
			WallSide = 1;
			GoalStage = 1;
		}
	}
}
void WallFollow(){
	if (WallSide == 0){ // WALL ON LEFT SIDE
		if (digitalRead(frontButton[2])||digitalRead(frontButton[3])){
			GoForward();
			Mtime = millis();
			Mtemp = 2;
		}
		else{
		  if (RED_X[2]){
			GoalStage = 2;
		  }
		  else if (RED_X[3]){
			GoalStage = 2;
		  }else{
//			if (Mcount > 10){
//			  GoalStage = 0;
//			}else if (Mcount < -10){
//			  GoalStage = 0;
//			}
			if (Mtemp == 0){
			  Mtemp = 1;
			  Mtime = millis();
			  if ((IR_X[1]==0)||(IR_X[3]==0)){ // Too FAR
				GoRight();
				Mtime = millis();
//				Mcount++;
			  }else if ((IR_net[1]>(2*IR_thresh[1]))||(IR_net[3]>(2*IR_thresh[3]))){ // TOO CLOSE
				GoLeft();
				Mtime = millis();
//				Mcount--;
			  }
			  else{
//				Mcount = 0;
				GoBackward();
				Mtime = millis();
			  }
			}else if (millis() - Mtime > 200){
				if (Mtemp == 2){
					Mtime = millis();
					Mtemp = 1;
					GoRight();
				}
			  Mtemp = 0;
			}
		  }
		}
	}else{ // WALL ON RIGHT SIDE
		if (RED_X[2]){
		GoalStage = 2;
	  }
	  else if (RED_X[3]){
		GoalStage = 2;
	  }
	  else{
//		if (Mcount > 10){
//		  GoalStage = 0;
//		}
//		else if (Mcount < -10){
//		  GoalStage = 0;
//		}
		if (Mtemp == 0){
		  Mtemp = 1;
		  Mtime = millis();
		  if ((IR_X[0]==0)||(IR_X[2]==0)){ // Too FAR
			GoLeft();
//			Mcount++;
		  }
		  else if ((IR_net[0]>(2*IR_thresh[0]))||(IR_net[2]>(2*IR_thresh[2]))){ // TOO CLOSE
			GoRight();
//			Mcount--;
		  }
		  else{
//			Mcount = 0;
			GoForward();
			Mtime = millis();
		  }
		}
		else if (millis() - Mtime > 200){
		  Mtemp = 0;
		}
	  }
	}
//	if ((Mcount < -10)||(Mcount > 10)){
//		GoalStage = 0;
//	}
}
void GreenMat(){
  switch (Mtemp){
  case 0:
	// Drive fully onto the green mat
	  if (WallSide == 0){
		if (RED_X[1]){
		  Mtemp = 1;
		  GoLeft();
		  Mtime = millis();
		}
	  }else if(RED_X[0]){
		  Mtemp = 1;
		  GoRight();
		  Mtime = millis();
	  }
	break;
  case 1:
	if (millis() - Mtime > 500){ // check in labs
	  Mtemp = 2;
	  GoForward();
	  Mtime = millis();
	}
	break;
  case 2:
	if (RED_X[2] == 0){
	  if (RED_X[3] == 0){
		GoalStage = 3;
		KICK();
	  }
	}
	break;
  }
}
void KICK(){

  // Stop robot
  Stop();
  delay(100);
  backdoor.write(backdoor_open);

  frontdoor.write(frontdoor_open);

  delay(200);
  // Kick the ball
  kicker.write(kicker_kick);
  delay(300);
  kicker.write(kicker_rest);
  backdoor.write(backdoor_closed);
  BallInHand = 0;
  BCol = 0;
  GoRight();
  delay(500);

}
// Stall Control
void StallControl(){
	int x = 0;

	// Stall Based on touch sensors
	if (millis() - StallTimeTouch > SdurTouch){
		for (int k=0;k<4;k++){
			if (buttonPrev[k] == digitalRead(frontButton[k])){
				x++;
			}
			buttonPrev[k] = digitalRead(frontButton[k]);
		}
		if (x==4){
			ScountTouch++;
			if (ScountTouch > 20){
				// Potentially Stalled, so escape
					switch(Sdir){
						case 0: GoForward(); 
						Mtime = millis();
						break;
						case 1: GoBackward();
						Mtime = millis();
						break;
						case 2: GoBackward();
						Mtime = millis();
						break;
						case 3: GoForward();
						Mtime = millis();
						break;
					}
				ScountTouch = 0;
				digitalWrite(BlueTeamLED,HIGH);
				digitalWrite(RedTeamLED,HIGH);
				Mtime = millis();
			}
		}
		else{
			ScountTouch = 0;
			digitalWrite(BlueTeamLED,LOW);
			digitalWrite(RedTeamLED,LOW);
		}
		StallTimeTouch = millis();
	}

	// Stall Based on IR
	x = 0;
	if (millis() - StallTimeIR > SdurIR){
		for (int k=0;k<4;k++){
			if ((IR_X[k])&&((IRnetPrev[k] - IR_net[k] > IRStallThresh)||(-IRnetPrev[k] + IR_net[k] > IRStallThresh))){
				x++;
			}
			IRnetPrev[k] = IR_net[k];
		}
		if (x>2){
			ScountIR++;
			if (ScountIR > 20){
				// Potentially Stalled, so escape
					switch(Sdir){
						case 0: GoForward(); 
						Mtime = millis();
						break;
						case 1: GoBackward();
						Mtime = millis();
						break;
						case 2: GoBackward();
						Mtime = millis();
						break;
						case 3: GoForward();
						Mtime = millis();
						break;
					}
				ScountIR = 0;
				digitalWrite(BlueTeamLED,HIGH);
				digitalWrite(RedTeamLED,HIGH);
				Mtime = millis();
			}
		}
		else{ 
			ScountIR = 0;
			digitalWrite(BlueTeamLED,LOW);
			digitalWrite(RedTeamLED,LOW);
		}
		StallTimeIR = millis();
	}
}
// Sensors
void REDandIR_Sense() {
  
  if ( (millis() - LED_prevTime) > dur) {
    
    
    if (LEDstate == HIGH) {
      //read in all IR sensors
      for (k=0; k<4; k++) {
        IR_on[k] = analogRead(IR[k]);
        RED_on[k] = analogRead(RED[k]);
      }
      LEDstate = LOW;
    }
    else  {
      for (k=0; k<4; k++) {
        IR_off[k] = analogRead(IR[k]);
        IR_net[k] = IR_on[k] - IR_off[k];
        RED_off[k] = analogRead(RED[k]);
        RED_net[k] = RED_on[k] - RED_off[k];
      }
      LEDstate = HIGH;
    }
    LED_prevTime = millis();
  digitalWrite(pulse, LEDstate);  
  }
  

  // Analysis of readings
  for (k=0;k<4;k++){
//	  // IR Readings
//	  if (IR_net[k] > IR_thresh[k])
//		  IR_temp[k]++;
//	  else
//		  IR_temp[k]--;
//	  // Prevent temp values from getting too small
//	  if (IR_temp[k]<0)
//		  IR_temp[k] = 0;
//	  else if (IR_temp[k]>1)
//		  IR_temp[k] = 1;

	 // if (IR_temp[k] == 1)
          if (IR_net[k] > IR_thresh[k])
		  IR_X[k] = 1;
	  else 
		  IR_X[k] = 0;

	  }
	  // Green Mat Readings
//	  if (RED_net[k] < RED_thresh[k])
//		  RED_temp[k]++;
//	  else
//		  RED_temp[k]--;
	  // Prevent temp values from getting too small
//	  if (RED_temp[k]<0)
//		  RED_temp[k] = 0;
//	  else if (RED_temp[k]>1)
//		  RED_temp[k] = 1;

	  //if (RED_temp[k] == 1)
          if (RED_net[k] < RED_thresh[k])
		  RED_X[k] = 1;
	  else
		  RED_X[k] = 0;
  }
void Ball_Colour() {
  // DO WE NEED A CONDITION TO DETECT BALL COLOUR??
  
  
   if ( (millis() - BALL_prevTime) > dur) {
    BALL_prevTime = millis();
    
    if (BALLstateB == HIGH && BALLstateR == LOW) { 
      BALL_onB = analogRead(BALL);
      BALLstateB = LOW;
      BALLstateR = HIGH;
    }
    else if (BALLstateB == LOW && BALLstateR == HIGH) { 
      BALL_onR = analogRead(BALL);
      BALLstateB = LOW;
      BALLstateR = LOW;
      Ball_Blue_Net = BALL_onB - BALL_off;
      Ball_RED_Net = BALL_onR - BALL_off;
    }
    else {
      BALL_off = analogRead(BALL);    
      BALLstateB = HIGH;
      BALLstateR = LOW;
    }

  digitalWrite(pulBALLb, BALLstateB);
  digitalWrite(pulBALLr, BALLstateR);
  

  // Analysis of Ball Colour
  if (Ball_Blue_Net > BallBlueThresh)  // Captured Ball Appears Blue
	  BCol_Temp++; 
  else if (Ball_RED_Net > BallRedThresh)  // Captured Ball Appears Red
	  BCol_Temp--;
  else if (BCol_Temp > 0) // No Ball, head to reset BCol_Temp
	  BCol_Temp--;
  else if (BCol_Temp < 0) // No Ball, head to reset BCol_Temp
	  BCol_Temp++;

   }
  // Stop |BCol_Temp| exceeding 4
  if (BCol_Temp > 4) { // Ball is Blue, set BCol = 1
	  BCol_Temp = 0;
	  	BallInHand = 1;
          BCol = 1;
          /*  digitalWrite(BlueTeamLED,HIGH);
          digitalWrite(RedTeamLED,LOW);*/
  }
  else if (BCol_Temp < -4) {// Ball is Red, set BCol = 2
	  BCol_Temp = 0;
	  	BallInHand = 1;
          BCol = 2;
          /*digitalWrite(RedTeamLED,HIGH);
          digitalWrite(BlueTeamLED,LOW);*/
  }
  else{  // No Ball
	  BallInHand = 0;
          /*digitalWrite(BlueTeamLED,LOW);
          digitalWrite(RedTeamLED,LOW);*/
  }
}
void LineSensor(){
	if (millis() - LStime > LSdur){
	  // initial loop used for sensor to detect loop, second loop to read data into arduino
	  digitalWrite(LSclock, HIGH);
	  digitalWrite(LineSensorPin, LOW);
	  digitalWrite(LSclock, LOW);
	  digitalWrite(LineSensorPin, HIGH);
  
	  for(LSj=0; LSj<128; LSj++) {
		digitalWrite(LSclock, HIGH);
		digitalWrite(LineSensorPin, LOW);
		delayMicroseconds(del);
		digitalWrite(LSclock, LOW);
		delayMicroseconds(del);
		 }
  
	  // reads data into arduino whilst pulsating the LSclock.
	  digitalWrite(LSclock, HIGH);
	  digitalWrite(LineSensorPin, LOW);
	  digitalWrite(LSclock, LOW);
	  digitalWrite(LineSensorPin, HIGH);
  
	  digitalWrite(LSclock, HIGH);
	  digitalWrite(LineSensorPin, LOW);
	  lightVal[0] = analogRead(dataPin);
	  delayMicroseconds(del);
	  digitalWrite(LSclock, LOW);
	  delayMicroseconds(del);
  
	  //collects data
	  for(LSj=1; LSj<128; LSj++) {
		digitalWrite(LSclock, HIGH);
		digitalWrite(LineSensorPin, LOW);
		delayMicroseconds(del);
		lightVal[LSj] = analogRead(dataPin);
		digitalWrite(LSclock, LOW);
		delayMicroseconds(del);
	  }
	  LStime = millis();
	}
}


