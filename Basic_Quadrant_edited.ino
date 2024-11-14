
/*            Throttle script with filtering
 *  This script takes three axis, and filters the signal for an output 
 *  at 50 Hz. The clock is used to exactly time the polling, 
 *  to avoid sampling bias. A crude bandpass filter throws 
 *  away spikes that are impossible(!) to result from user 
 *  input. This will convert spikes on the control axis into 
 *  a mere stutter. Then a Kalman filter is applied to the 
 *  20 Hz signal to remove remaining gaussian noise. 
 *  All axes each get their own one dimensional Kalman 
 *  because there is no correlation that is relevant for the 
 *  filter. The constants in this script are configured for 
 *  the Saitek Pro throttle quadrant.
 *  
 *  NB!!!!!!!!!! MUST USE ItsyBitsy 5V 16Mhz - VID: 239A PID: 800E
*/

#include "Joystick.h"
#include "SimpleKalmanFilter.h"

#define OUTPUT_LIMIT_X 992
#define OUTPUT_LIMIT_Y 987
#define OUTPUT_LIMIT_Z 988
#define OUTPUT_MIN_X 679
#define OUTPUT_MIN_Y 604
#define OUTPUT_MIN_Z 474
#define OUTPUT_HZ 5
#define OUTPUT_LIMIT 450
#define BANDPASS_SPD 13 
#define MEASUREM_UNCERT 10
#define PROC_VARIANCE 2
//#define REV_THRESHOLDX 1000
//#define REV_THRESHOLDY 1000
//#define REV_THRESHOLDZ 1000
int REV_THRESHOLDX = OUTPUT_LIMIT_X + 3;
int REV_THRESHOLDY = OUTPUT_LIMIT_Y + 3;
int REV_THRESHOLDZ = OUTPUT_LIMIT_Z + 3;


/*                      Settings
* REV_THRESHOLD      Past idle notch of the throttle, activates reverse
* OUTPUT_LIMIT       Limit of value that is output for a slider. should
*                    correspond to the idle notch
* OUTPUT_HZ          Desired output frequency to the computer
* BANDPASS_SPD       Rate of change over which measurements will be
*                    discarded. 10 Means 10 times the full range per 
*                    second. Don't set the bandpass speed too low, 
*                    erratic output if it is actually achieved! It 
*                    needs to be a value where with 100% certainty it 
*                    is a potentiometer spike and not user input!
* MEASUREMENT_UNCERT Random noise to be expected from the Pot,
*                    in output resolution units
* PROC_VARIANCE      Normal accelerations of the control to be 
*                    expected, in output resoltution units per sample 
*                    in OUTPUT_HZ. Decreasing this will decrease 
*                    responsiveness
*/

int potX, valX;
int potY, valY;
int potZ, valZ;
int but1, but2, but3, but4, but5, but6;
int revX, revY, revZ;
unsigned int bandPass;
unsigned long lastPoll = 0;
//unsigned int sampleTime;

// output configuration
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 9, 0,
  true, true, true, false, false, false,
  false, false, false, false, false);

SimpleKalmanFilter kalmanFilterX(MEASUREM_UNCERT, MEASUREM_UNCERT, PROC_VARIANCE);
SimpleKalmanFilter kalmanFilterY(MEASUREM_UNCERT, MEASUREM_UNCERT, PROC_VARIANCE);
SimpleKalmanFilter kalmanFilterZ(MEASUREM_UNCERT, MEASUREM_UNCERT, PROC_VARIANCE);

void setup() {
  Joystick.begin();
  Joystick.setXAxisRange(1, 100);
  Joystick.setYAxisRange(1, 100);
  Joystick.setZAxisRange(1, 100);

  pinMode(A0, INPUT); //configure inputs for potentiometers
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(2,INPUT_PULLUP); //configure inputs for buttons, use internal pullup resistor
  pinMode(3,INPUT_PULLUP); 
  pinMode(4,INPUT_PULLUP); 
  pinMode(5,INPUT_PULLUP); 
  pinMode(6,INPUT_PULLUP); 
  pinMode(7,INPUT_PULLUP); 
    
  //initialize parameters for the filters and sampling
  //sampleTime = 1000000 / OUTPUT_HZ;
  bandPass = 30000; //just the first two seconds, no real bandpass
  bandPass = (BANDPASS_SPD * OUTPUT_LIMIT_X) / OUTPUT_HZ;
  
  //Serial.begin( 9600 ); //remove comment for debugging on the serial monitor/plotter
}

void loop() {

/*
  //enable band pass filter after 2 seconds stabilization
  if (bandPass ==  30000 && millis() > 2000) {
    bandPass = (BANDPASS_SPD * OUTPUT_LIMIT_X) / OUTPUT_HZ;
  }

  //read inputs
  while (lastPoll+sampleTime > micros()) {delayMicroseconds(4);} //clocked sample rate.
  */
  int newpotX, newpotY, newpotZ;
  newpotX = 1024-analogRead(A1); //X axis, Throttle
  newpotY = 1024-analogRead(A0); //Y axis, Prop
  newpotZ = 1024-analogRead(A2); //Z axis, Mixture

  if (newpotX < (potX - 1) || newpotX > (potX + 1)) {
    potX = newpotX;
  }
  if (newpotY < (potY - 1) || newpotY > (potY + 1)) {
    potY = newpotY;
  }  
  if (newpotZ < (potZ - 1) || newpotZ > (potZ + 1)) {
    potZ = newpotZ;
  }

  //lastPoll = micros();
  but1 = !digitalRead(2); 
  but2 = !digitalRead(3); 
  but3 = !digitalRead(4); 
  but4 = !digitalRead(5); 
  but5 = !digitalRead(6); 
  but6 = !digitalRead(7); 

  //process notch and reverse ranges
  if (potX > REV_THRESHOLDX) { //test whether lever is in reverse range
    revX = 1; //reverse on
  } else {
    revX = 0; //reverse off
  }

  if (potX > OUTPUT_LIMIT_X){ potX = OUTPUT_LIMIT_X;} //limit the output range
  
  if (potY > REV_THRESHOLDY) { //test whether lever is in reverse range
    revY = 1; //reverse on
  } else {
    revY = 0; //reverse off
  }

  if (potY > OUTPUT_LIMIT_Y){ potY = OUTPUT_LIMIT_Y;} //limit the output range

  if (potZ > REV_THRESHOLDZ) { //test whether lever is in reverse range
    revZ = 1; //reverse on
  } else {
    revZ = 0; //reverse off
  }

  if (potZ > OUTPUT_LIMIT_Z){ potZ = OUTPUT_LIMIT_Z;} //limit the output range

  //throw out too large changes (bandpass)
  if (abs(valX -potX) < bandPass) { //very crude bandpass filter
    valX= potX;                     
    } else {
    Serial.println(" !!!BANDPASS FILTER HAS TWROWN OUT AN INPUT VALUE!!!"); //debug only
  }
  
  if (abs(valY -potY) < bandPass) {
    valY= potY;                     
    } else {
    Serial.println(" !!!BANDPASS FILTER HAS TWROWN OUT AN INPUT VALUE!!!"); //debug only
  }

  if (abs(valZ -potZ) < bandPass) {
    valZ= potZ;                     
    } else {
    Serial.println(" !!!BANDPASS FILTER HAS TWROWN OUT AN INPUT VALUE!!!"); //debug only
  }

  //apply Kalman filter to remove remaining gaussian noise
  valX = kalmanFilterX.updateEstimate(valX); 
  valY = kalmanFilterY.updateEstimate(valY);
  valZ = kalmanFilterZ.updateEstimate(valZ);

  int axisValueX = exp(0.0143*(valX - OUTPUT_MIN_X))*1.2;
  int axisValueY = exp(0.0115*(valY - OUTPUT_MIN_Y))*1.3;
  int axisValueZ = exp(0.0085*(valZ - OUTPUT_MIN_Z))*1.3;
  //0,9636e0,0143x

//  //remove comment for debugging on the serial monitor/plotter
/*
  Serial.print("Pot-X: ");
  Serial.print(newpotX);
  Serial.print(" Pot-Y: ");
  Serial.print(newpotY);
  Serial.print(" Pot-Z: ");
  Serial.print(newpotZ);
  Serial.println("");

  Serial.print("    X: ");
  Serial.print(valX);
  Serial.print("    Y: ");
  Serial.print(valY);
  Serial.print("    Z: ");
  Serial.print(valZ);
  Serial.println("");

  Serial.print("AxisX: ");
  Serial.print(axisValueX);
  Serial.print(" AxisY: ");
  Serial.print(axisValueY);
  Serial.print(" AxisZ: ");
  Serial.print(axisValueZ);
  Serial.println("");
*/
//  Serial.print(" rev X: ");
//  Serial.print(revX);
//  Serial.print(" rev Y: ");
//  Serial.print(revY);
//  Serial.print(" rev Z: ");
//  Serial.print(revZ);
//  Serial.println("");

//
//  Joystick.setXAxis(valX);
//  Joystick.setYAxis(valY);
//  Joystick.setZAxis(valZ);
  Joystick.setXAxis(axisValueX);
  Joystick.setYAxis(axisValueY);
  Joystick.setZAxis(axisValueZ);
  Joystick.setButton(0, but1);
  Joystick.setButton(1, but2);
  Joystick.setButton(2, but3);
  Joystick.setButton(3, but4);
  Joystick.setButton(4, but5);
  Joystick.setButton(5, but6);
  Joystick.setButton(6, revX);
  Joystick.setButton(7, revY);
  Joystick.setButton(8, revZ);
  
  delay(10);
}
