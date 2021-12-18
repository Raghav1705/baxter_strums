//THIS VERSION INCLUDES THE AUDIO SENSOR INSTEAD OF POTENTIOMETER
//THIS VERSION HAS THE WIP STATE MACHINE W/ PORT EXPANDER
//THIS VERSION ALSO WAS TUNED TO WORK WITH BAXTER
#include <Adafruit_MCP23017.h>
#include <AudioAnalyzer.h>
#include <ESP32Encoder.h>

#define BTNon 15
#define BTNplay 13
#define SOL_PIN0 7
#define SOL_PIN1 6
#define SOL_PIN2 5
#define SOL_PIN3 4
#define pub_pin 3

#define motor1_channel1 8 //blue
#define motor1_channel2 9
#define motor2_channel1 10 //green
#define motor2_channel2 11
#define motor3_channel1 12 //yellow
#define motor3_channel2 13
#define motor4_channel1 14 //brown
#define motor4_channel2 15

Adafruit_MCP23017 mcp;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

Analyzer Audio = Analyzer(14, 32, 12); //Strobe pin ->14  RST pin ->32 Analog Pin ->

int nut = 0;
int fret0 = -15;
int fret1 = 610;
int fret2 = 1620;
int fret3 = 2460;
int fret4 = 3366;
int fret5 = 4194;
//int fret[4] = {fret1, fret2, fret3, fret4};
int chords[10][4] = {
  /*A*/{fret2, fret0, fret0, fret0},/*G*/{fret0, fret2, fret3, fret2},/*C*/{fret0, fret0, fret0, fret3},
  /*A*/{fret2, fret0, fret0, fret0},/*G*/{fret0, fret2, fret3, fret2},/*C*/{fret0, fret0, fret0, fret3},
  /*A*/{fret2, fret0, fret0, fret0},/*G*/{fret0, fret2, fret3, fret2},/*C*/{fret0, fret0, fret0, fret3},
  /*F*/{fret2, fret0, fret1, fret0}
};

bool actuate[10][4] = {
  /*A*/{true, false, false, false},/*G*/{false, true, true, true},/*C*/ {false, false, false, true},
  /*A*/{true, false, false, false},/*G*/{false, true, true, true},/*C*/ {false, false, false, true},
  /*A*/{true, false, false, false},/*G*/{false, true, true, true},/*C*/ {false, false, false, true},
  /*F*/{true, false, true, false}
};

int w = 0;
int x = 0;
int y = 0;
int z = 0;
//int c = 0;

int state = 0;
int FreqVal[7];//set array of values of size 7
int highthresh = 3100;
int lowthresh = 500;
bool highpass = false;


volatile bool motor1on = false;
volatile bool motor1onreverse = false;
volatile bool motor2on = false;
volatile bool motor2onreverse = false;
volatile bool motor3on = false;
volatile bool motor3onreverse = false;
volatile bool motor4on = false;
volatile bool motor4onreverse = false;
volatile int des_enc1_val = chords[w][0];
volatile int des_enc2_val = chords[x][1];
volatile int des_enc3_val = chords[y][2];
volatile int des_enc4_val = chords[z][3];


//Setup interrupt variables ----------------------------

volatile bool ONtimerDone = false;
volatile bool PLAYtimerDone = false;
volatile bool ONButtonIsPressed = false;
volatile bool PLAYButtonIsPressed = false;

//interrupts init
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() { //ON BUTTON DEBOUNCING
  portENTER_CRITICAL_ISR(&timerMux0);
  ONtimerDone = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
  timerStop(timer0);
}
void ONTimerInterruptInit() {
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTime0, true);
  timerAlarmWrite(timer0, 2000000, true);
  timerAlarmEnable(timer0);
}
void IRAM_ATTR ONbtntmr() {
  if (ONtimerDone && (state == 0 || state == 1))
  {
    ONButtonIsPressed = true;
  }
  ONTimerInterruptInit();
}

void IRAM_ATTR onTime1() { //PLAY BUTTON DEBOUNCING
  portENTER_CRITICAL_ISR(&timerMux1);
  PLAYtimerDone = true;//the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
  timerStop(timer1);
}

void PLAYTimerInterruptInit() {
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTime1, true);
  timerAlarmWrite(timer1, 2000000, true);
  timerAlarmEnable(timer1);
}
void IRAM_ATTR PLAYbtntmr() {
  if (PLAYtimerDone) 
  {
    if ((state == 0) || (state == 2)) {
      PLAYButtonIsPressed = false;
    } else if ((state == 1) || (state == 3) || (state == 4)) {
      PLAYButtonIsPressed = true;
    } else {
      PLAYButtonIsPressed = false;
    }
  }
  PLAYTimerInterruptInit();
}

//Setup ------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  pinMode(BTNon, INPUT); // configures the specified pin to behave either as an input or an output
  pinMode(BTNplay, INPUT);
  attachInterrupt(BTNon, ONbtntmr, RISING);  // set the "BTN" pin
  attachInterrupt(BTNplay, PLAYbtntmr, RISING);  // set the "BTN" pin

  Serial.begin(115200);

  Audio.Init();//Init audio module
  mcp.begin();// "Start" the mcp object
  mcp.pinMode(motor1_channel1, OUTPUT); //set motor pins
  mcp.pinMode(motor1_channel2, OUTPUT);
  mcp.pinMode(motor2_channel1, OUTPUT);
  mcp.pinMode(motor2_channel2, OUTPUT);
  mcp.pinMode(motor3_channel1, OUTPUT);
  mcp.pinMode(motor3_channel2, OUTPUT);
  mcp.pinMode(motor4_channel1, OUTPUT);
  mcp.pinMode(motor4_channel2, OUTPUT);
  mcp.digitalWrite(motor1_channel1, LOW); //sets motor to low by default
  mcp.digitalWrite(motor1_channel2, LOW);
  mcp.digitalWrite(motor2_channel1, LOW);
  mcp.digitalWrite(motor2_channel2, LOW);
  mcp.digitalWrite(motor3_channel1, LOW);
  mcp.digitalWrite(motor3_channel2, LOW);
  mcp.digitalWrite(motor4_channel1, LOW);
  mcp.digitalWrite(motor4_channel2, LOW);

  mcp.pinMode(SOL_PIN0, OUTPUT);
  mcp.pinMode(SOL_PIN1, OUTPUT);
  mcp.pinMode(SOL_PIN2, OUTPUT);
  mcp.pinMode(SOL_PIN3, OUTPUT);
  mcp.digitalWrite(SOL_PIN0, LOW); // sets the initial state of SOL as turned-off
  mcp.digitalWrite(SOL_PIN1, LOW); // sets the initial state of SOL as turned-off
  mcp.digitalWrite(SOL_PIN2, LOW); // sets the initial state of SOL as turned-off
  mcp.digitalWrite(SOL_PIN3, LOW); // sets the initial state of SOL as turned-off

  mcp.pinMode(pub_pin,OUTPUT);
  mcp.digitalWrite(pub_pin,LOW);
  
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder1.attachHalfQuad(26, 25); // Attache pins for use as encoder pins
  encoder2.attachHalfQuad(34, 39);
  encoder3.attachHalfQuad(36, 4);
  encoder4.attachHalfQuad(33, 27);
  encoder1.setCount(0);  // set starting count value after attaching
  encoder2.setCount(0);
  encoder3.setCount(0);
  encoder4.setCount(0);
  encoder1.clearCount( );
  encoder2.clearCount( );
  encoder3.clearCount( );
  encoder4.clearCount( );

  // initilize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 2000000, true); // 2000000 * 1 us = 2 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
}

//////////////////////////////////////////////////////////MAIN LOOP ///////////////////////////////////////////////--------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
  Audio.ReadFreq(FreqVal);

  switch (state) {
    case 0 : // CURRENTLY IN OFF STATE
      Serial.print("Enc1 Val: ");
      Serial.print(((int32_t)encoder1.getCount()));
      Serial.print("Enc2 Val: ");
      Serial.print(((int32_t)encoder2.getCount()));
      Serial.print("Enc3 Val: ");
      Serial.print(((int32_t)encoder3.getCount()));
      Serial.print("Enc4 Val: ");
      Serial.print(((int32_t)encoder4.getCount()));
      Serial.print(" State: 0, OFF\n");
      if (CheckOnButtonPressed()) {
        state = 1; // SWITCH TO PAUSE STATE
        mcp.digitalWrite(pub_pin,LOW);
      }
      StopActuatingService(1);
      StopActuatingService(2);
      StopActuatingService(3);
      StopActuatingService(4);
      break;

    case 1 : // CURRENTLY IN PAUSE
      Serial.print("Enc1 Val: ");
      Serial.print(((int32_t)encoder1.getCount()));
      Serial.print("Enc2 Val: ");
      Serial.print(((int32_t)encoder2.getCount()));
      Serial.print("Enc3 Val: ");
      Serial.print(((int32_t)encoder3.getCount()));
      Serial.print("Enc4 Val: ");
      Serial.print(((int32_t)encoder4.getCount()));
      Serial.print(" State: 1, PAUSE\n");
      if (CheckPlayButtonPressed()) {
        state = 3; //Chord transitioning state
        mcp.digitalWrite(pub_pin,LOW);
        NextChordService(1);
        NextChordService(2);
        NextChordService(3);
        NextChordService(4);
      }
      if (CheckOnButtonPressed()) {
        state = 2; // SWITCH TO RESETTING MOTORS
        mcp.digitalWrite(pub_pin,LOW);
        ResetMotorsService(1);
        ResetMotorsService(2);
        ResetMotorsService(3);
        ResetMotorsService(4);
      }
      break;

    case 2 : // RESETTING MOTORS STATEp
      Serial.print("Enc1 Val: ");
      Serial.print(((int32_t)encoder1.getCount()));
      Serial.print("Enc2 Val: ");
      Serial.print(((int32_t)encoder2.getCount()));
      Serial.print("Enc3 Val: ");
      Serial.print(((int32_t)encoder3.getCount()));
      Serial.print("Enc4 Val: ");
      Serial.print(((int32_t)encoder4.getCount()));
      Serial.print(" State: 2, RESETTING MOTOR\n");
      if (CheckEncAtZero(1)) {
        StopActuatingService(1);
      }
      if (CheckEncAtZero(2)) {
        StopActuatingService(2);
      }
      if (CheckEncAtZero(3)) {
        StopActuatingService(3);
      }
      if (CheckEncAtZero(4)) {
        StopActuatingService(4);
      }
      if (!motor1on && !motor1onreverse && !motor2on && !motor2onreverse && !motor3on && !motor3onreverse && !motor4on && !motor4onreverse) {
        //if for some reason this isn't working try to fix the tolerence in check enc at zero
        state = 0;
        mcp.digitalWrite(pub_pin,LOW);
        Serial.print("\n RESET MOTORS DONE -- DONE \n");
      }
      break;

    case 3 : // CHORD TRANSITIONING STATE
      Serial.print("Enc1 Val: ");
      Serial.print(((int32_t)encoder1.getCount()));
      Serial.print("Enc2 Val: ");
      Serial.print(((int32_t)encoder2.getCount()));
      Serial.print("Enc3 Val: ");
      Serial.print(((int32_t)encoder3.getCount()));
      Serial.print("Enc4 Val: ");
      Serial.print(((int32_t)encoder4.getCount()));
      Serial.print(" State: 3, TRANSITIONING\n");
      if (CheckEncAtDes(1)) {
        stopMotorResponse(1);
        
      }
      if (CheckEncAtDes(2)) {
        stopMotorResponse(2);
        
      }
      if (CheckEncAtDes(3)) {
        stopMotorResponse(3);
        
      }
      if (CheckEncAtDes(4)) {
        stopMotorResponse(4);
        
      }
      if (!motor1on && !motor1onreverse && !motor2on && !motor2onreverse && !motor3on && !motor3onreverse && !motor4on && !motor4onreverse) {
        //also may need to check tolernces with new motor
        Serial.print("\n NEXT CHORD SERVICE -- DONE \n");
        BeginChordService(1);
        BeginChordService(2);
        BeginChordService(3);
        BeginChordService(4);
        state = 4;
        
        
      }

      if (CheckPlayButtonPressed()) {
        StopActuatingService(1);
        StopActuatingService(2);
        StopActuatingService(3);
        StopActuatingService(4);
        mcp.digitalWrite(pub_pin,LOW);
        state = 1; //Chord transitioning state
      }
      break;

    case 4 : // CHORD PLAYING STATE
      mcp.digitalWrite(pub_pin,HIGH);
      Serial.print("Enc1 Val: ");
      Serial.print(((int32_t)encoder1.getCount()));
      Serial.print("Enc2 Val: ");
      Serial.print(((int32_t)encoder2.getCount()));
      Serial.print("Enc3 Val: ");
      Serial.print(((int32_t)encoder3.getCount()));
      Serial.print("Enc4 Val: ");
      Serial.print(((int32_t)encoder4.getCount()));
      Serial.print(" State: 4, CHORD PLAYING\n");
      //mcp.digitalWrite(pub_pin,LOW);
      if (CheckAudioAtThresh()) {
        state = 3;
        mcp.digitalWrite(pub_pin,LOW);
        NextChordService(1);
        NextChordService(2);
        NextChordService(3);
        NextChordService(4);
        mcp.digitalWrite(pub_pin,LOW);
      }
      if (CheckPlayButtonPressed()) {
        state = 1; //Chord transitioning state
        mcp.digitalWrite(pub_pin,LOW);
        StopActuatingService(1);
        StopActuatingService(2);
        StopActuatingService(3);
        StopActuatingService(4);
        mcp.digitalWrite(pub_pin,LOW);
      }
      break;

  }
}
///////////////// EVENT CHECKERS BASED OFF PSEUDO CODE /////////////////////

bool CheckOnButtonPressed() {

  if (ONtimerDone && ONButtonIsPressed && (state == 1 || state == 0) ) {
    ONtimerDone = false;
    ONTimerInterruptInit();
    ONButtonIsPressed = false;
    return true;
  }
  else {
    return false;
  }
}


bool CheckPlayButtonPressed() {

  if (PLAYtimerDone && PLAYButtonIsPressed) {
    PLAYButtonIsPressed = false;
    PLAYtimerDone = false;
    PLAYTimerInterruptInit();
    return true;
  }
  else {
    return false;
  }
}

bool CheckEncAtDes (int encnum) {
  //Did all Encoders reach the desired value?
  //  If MotorsAtDesValue is true
  //    Turn flag off and return true
  //  Else, return false
  //  Serial.print("Check ENCODER");
  if (encnum == 1) {
    if ((motor1on == true) && ((int32_t)encoder1.getCount() < des_enc1_val - 75) ) {
      return false;
    }
    if ((motor1onreverse == true) && ((int32_t)encoder1.getCount() > des_enc1_val + 75)) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 2) {
    if ((motor2on == true) && ((int32_t)encoder2.getCount() < des_enc2_val - 75) ) {
      return false;
    }
    if ((motor2onreverse == true) && ((int32_t)encoder2.getCount() > des_enc2_val + 75)) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 3) {
    if ((motor3on == true) && ((int32_t)encoder3.getCount() < des_enc3_val - 75) ) {
      return false;
    }
    if ((motor3onreverse == true) && ((int32_t)encoder3.getCount() > des_enc3_val + 75)) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 4) {
    if ((motor4on == true) && ((int32_t)encoder4.getCount() < des_enc4_val - 75) ) {
      return false;
    }
    if ((motor4onreverse == true) && ((int32_t)encoder4.getCount() > des_enc4_val + 75)) {
      return false;
    }
    else {
      return true;
    }
  }
}

bool CheckEncAtZero (int encnum) {
  //Did all Encoders reach the zero value?
  //  If MotorsAtDesValue is true
  //    Turn flag off and return true
  //  Else, return false
  //  Serial.print("Check ENCODER");
  if (encnum == 1) {
    if ((int32_t)encoder1.getCount() > 75) {
      return false;
    }
    if ((int32_t)encoder1.getCount() < -75) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 2) {
    if ((int32_t)encoder2.getCount() > 75) {
      return false;
    }
    if ((int32_t)encoder2.getCount() < -75) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 3) {
    if ((int32_t)encoder3.getCount() > 75) {
      return false;
    }
    if ((int32_t)encoder3.getCount() < -75) {
      return false;
    }
    else {
      return true;
    }
  }
  if (encnum == 4) {
    if ((int32_t)encoder4.getCount() > 75) {
      return false;
    }
    if ((int32_t)encoder4.getCount() < -75) {
      return false;
    }
    else {
      return true;
    }
  }
}


bool CheckAudioAtThresh () {
  //Did the Audio Sensor reach the threshold?
  //  If AudioAtThresh is true
  //    Turn flag off and return true
  //  Else, return false
  //  Serial.print("Check AUDIO THRESH");

  int audio_val = FreqVal[1];
  Serial.print("Audio value = ");
  Serial.println(audio_val);
  if (state == 4 && audio_val > highthresh)
  {
    highpass = true;
  }
  //Serial.print(audio_val);
  if (state == 4 && highpass && audio_val < lowthresh) {
    highpass = false;
    return true;
  }
  else {
    return false;
  }
}




//////////////////////  MOTOR HELPER FUNCTIONS //////////////////////////
void stopMotorResponse(int motornum) {
  if (motornum == 1) {
    mcp.digitalWrite(motor1_channel1, LOW);
    mcp.digitalWrite(motor1_channel2, LOW);
    motor1on = false;
    motor1onreverse = false;
  }
  if (motornum == 2) {
    mcp.digitalWrite(motor2_channel1, LOW);
    mcp.digitalWrite(motor2_channel2, LOW);
    motor2on = false;
    motor2onreverse = false;
  }
  if (motornum == 3) {
    mcp.digitalWrite(motor3_channel1, LOW);
    mcp.digitalWrite(motor3_channel2, LOW);
    motor3on = false;
    motor3onreverse = false;
  }
  if (motornum == 4) {
    mcp.digitalWrite(motor4_channel1, LOW);
    mcp.digitalWrite(motor4_channel2, LOW);
    motor4on = false;
    motor4onreverse = false;
  }
}

void startReverseMotorResponse(int motornum) {
  if (motornum == 1) {
    mcp.digitalWrite(motor1_channel2, LOW);
    mcp.digitalWrite(motor1_channel1, HIGH);
    motor1onreverse = true;
  }
  if (motornum == 2) {
    mcp.digitalWrite(motor2_channel2, LOW);
    mcp.digitalWrite(motor2_channel1, HIGH);
    motor2onreverse = true;
  }
  if (motornum == 3) {
    mcp.digitalWrite(motor3_channel2, LOW);
    mcp.digitalWrite(motor3_channel1, HIGH);
    motor3onreverse = true;
  }
  if (motornum == 4) {
    mcp.digitalWrite(motor4_channel2, LOW);
    mcp.digitalWrite(motor4_channel1, HIGH);
    motor4onreverse = true;
  }
}

void startMotorResponse(int motornum) {

  if (motornum == 1) {
    mcp.digitalWrite(motor1_channel1, LOW);
    mcp.digitalWrite(motor1_channel2, HIGH);
    motor1on = true;
  }
  if (motornum == 2) {
    mcp.digitalWrite(motor2_channel1, LOW);
    mcp.digitalWrite(motor2_channel2, HIGH);
    motor2on = true;
  }
  if (motornum == 3) {
    mcp.digitalWrite(motor3_channel1, LOW);
    mcp.digitalWrite(motor3_channel2, HIGH);
    motor3on = true;
  }
  if (motornum == 4) {
    mcp.digitalWrite(motor4_channel1, LOW);
    mcp.digitalWrite(motor4_channel2, HIGH);
    motor4on = true;
  }
}


/////////////// EVENT SERVICES BASED FROM PSEUDO CODE ////////////////////////

void StopActuatingService(int motornum) {
  //Turn all solenoids to LOW & Stop all Motors
  if (motornum == 1) {
    stopMotorResponse(motornum);
    mcp.digitalWrite(SOL_PIN0, LOW);
    //Serial.print("\n stop actuating service -- done \n");
  }
  if (motornum == 2) {
    stopMotorResponse(motornum);
    mcp.digitalWrite(SOL_PIN1, LOW);
    //Serial.print("\n stop actuating service -- done \n");
  }
  if (motornum == 3) {
    stopMotorResponse(motornum);
    mcp.digitalWrite(SOL_PIN2, LOW);
    //Serial.print("\n stop actuating service -- done \n");
  }
  if (motornum == 4) {
    stopMotorResponse(motornum);
    mcp.digitalWrite(SOL_PIN3, LOW);
    //Serial.print("\n stop actuating service -- done \n");
  }

}

void ResetMotorsService(int motornum) {
  //All desired encoder values set to 0
  //Turn all solenoids to LOW
  //Turn all motors on in reverse
  w = 0;
  x = 0;
  y = 0;
  z = 0;
  if (motornum == 1) {
    des_enc1_val = chords[w][motornum-1];
    mcp.digitalWrite(SOL_PIN0, LOW);

    if (((int32_t)encoder1.getCount() > 0 )) {
      startReverseMotorResponse(motornum); //MOVING BACKWARDS
    }
    if (((int32_t)encoder1.getCount() < 0 )) {
      startMotorResponse(motornum); //MOVING FORWARDS
    }
  }
  if (motornum == 2) {
    des_enc2_val = chords[x][motornum-1];
    mcp.digitalWrite(SOL_PIN1, LOW);

    if (((int32_t)encoder2.getCount() > 0 )) {
      startReverseMotorResponse(motornum); //MOVING BACKWARDS
    }
    if (((int32_t)encoder2.getCount() < 0 )) {
      startMotorResponse(motornum); //MOVING FORWARDS
    }
  }
  if (motornum == 3) {
    des_enc3_val = chords[y][motornum-1];
    mcp.digitalWrite(SOL_PIN2, LOW);

    if (((int32_t)encoder3.getCount() > 0 )) {
      startReverseMotorResponse(motornum); //MOVING BACKWARDS
    }
    if (((int32_t)encoder3.getCount() < 0 )) {
      startMotorResponse(motornum); //MOVING FORWARDS
    }
  }
  if (motornum == 4) {
    des_enc4_val = chords[z][motornum-1];
    mcp.digitalWrite(SOL_PIN3, LOW);

    if (((int32_t)encoder4.getCount() > 0 )) {
      startReverseMotorResponse(motornum); //MOVING BACKWARDS
    }
    if (((int32_t)encoder4.getCount() < 0 )) {
      startMotorResponse(motornum); //MOVING FORWARDS
    }
  }
}

void BeginChordService(int encnum) {
  //  Turn desired solenoids to HIGH
  //  Set desired encoder values to next chord positions
  //  Set desired solenoids to next chord solenoids
  if (encnum == 1) {
    if (actuate[w][encnum-1]){
      mcp.digitalWrite(SOL_PIN0, HIGH);
    }
    //change to set desired next chord position
    w++;
    if (w >= 10) {
      w = 0;
    }
    des_enc1_val = chords[w][encnum-1];
    
    Serial.print("\n begin chord service -- done \n");
  }
  if (encnum == 2) {
    if (actuate[x][encnum-1]){
      mcp.digitalWrite(SOL_PIN1, HIGH);
    }

    //change to set desired next chord position
    x++;
    if (x >= 10) {
      x = 0;
    }
    des_enc2_val = chords[x][encnum-1];    
    Serial.print("\n begin chord service -- done \n");
  }
  if (encnum == 3) {
    if (actuate[y][encnum-1]){
      mcp.digitalWrite(SOL_PIN2, HIGH);
    }

    //change to set desired next chord position
    y++;
    if (y >= 10) {
      y = 0;
    }
    des_enc3_val = chords[y][encnum-1];
    Serial.print("\n begin chord service -- done \n");
  }
  if (encnum == 4) {
    if (actuate[z][encnum-1]){
      mcp.digitalWrite(SOL_PIN3, HIGH);
    }

    //change to set desired next chord position
    z++;
    if (z >= 10) {
      z = 0;
    }
    des_enc4_val = chords[z][encnum-1];
    Serial.print("\n begin chord service -- done \n");
  }
}

void NextChordService(int encnum) {
  //  Turn all solenoids to LOW
  //  Compare current encoder positions to desired positions
  //  Actuate motors forward/reverse based on their positions
  if (encnum == 1) {
    mcp.digitalWrite(SOL_PIN0, LOW);
    if(des_enc1_val == fret0){
      //do nothing
    }
    else if (((int32_t)encoder1.getCount() < des_enc1_val )) {
      startMotorResponse(encnum); //MOVING FWD
    }
    else if (((int32_t)encoder1.getCount() > des_enc1_val )) {
      startReverseMotorResponse(encnum); //MOVING BACKWARD
    }
  }
  if (encnum == 2) {
    mcp.digitalWrite(SOL_PIN1, LOW);
    if(des_enc2_val == fret0){
      //do nothing
    }
    else if (((int32_t)encoder2.getCount() < des_enc2_val )) {
      startMotorResponse(encnum); //MOVING FWD
    }
    else if (((int32_t)encoder2.getCount() > des_enc2_val )) {
      startReverseMotorResponse(encnum); //MOVING BACKWARD
    }
  }
  if (encnum == 3) {
    mcp.digitalWrite(SOL_PIN2, LOW);
    if(des_enc3_val == fret0){
      //do nothing
    }
    else if (((int32_t)encoder3.getCount() < des_enc3_val )) {
      startMotorResponse(encnum); //MOVING FWD
    }
    else if (((int32_t)encoder3.getCount() > des_enc3_val )) {
      startReverseMotorResponse(encnum); //MOVING BACKWARD
    }
  }
  if (encnum == 4) {
    mcp.digitalWrite(SOL_PIN3, LOW);
    if(des_enc4_val == fret0){
      //do nothing
    }
    else if (((int32_t)encoder4.getCount() < des_enc4_val )) {
      startMotorResponse(encnum); //MOVING FWD
    }
    else if (((int32_t)encoder4.getCount() > des_enc4_val )) {
      startReverseMotorResponse(encnum); //MOVING BACKWARD
    }
  }
}
