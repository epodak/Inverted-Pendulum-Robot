/*
	Inverted Pendulum Robot Firmware

        - Collects translational position signals from rotary encoders via interrupts.
	- Collects pitch angle and angular velocity signals from UART (sent by IMU).
        - (Optional) Applies LPF to encoder data to address backlash.
        - (Optional) Applies Savitzky-Golay filter to estimate translational velocities.
        - (Optional) Applies moving-average type smoothing to estimate translational velocities.
        - Implements state feedback control to calculate output command signals to H-bridge.
        - Sends output commands to H-bridge as PWM signals.

	Created 01/13/2013
	By Brady Bolton
	Modified 02/25/2014

*/

// Libraries
#include "Arduino.h"
#include <digitalWriteFast.h>
#include <MatrixMath.h>

// Pin Assignments
// Input
#define encoderL_PinA 2
#define encoderR_PinA 3
#define encoderL_PinB 4
#define encoderR_PinB 5
#define encoderL_Index 6
#define encoderR_Index 7
#define currentL A0
#define currentR A1
// Output
#define hbridgeL_PWM 9
#define hbridgeR_PWM 10
#define hbridgeL_Dir 11
#define hbridgeR_Dir 12

// Interrupts
#define encoderL_Interrupt 0
#define encoderR_Interrupt 1

// Internal Variables
int loops = 0;
#define encoderR_IsReversed
volatile bool encoderL_BSet;
volatile bool encoderR_BSet;
double encoderL_Ticks = 0;
double encoderR_Ticks = 0;
double encoderL_Ticks_filt = 0;
double encoderL_Ticks_filt_prev = 0;
double encoderR_Ticks_filt = 0;
double encoderR_Ticks_filt_prev = 0;
double revsR = 0;
double revsL = 0;
double xL = 0;
double xR = 0;
double xC = 0;
double phi = 0;
double prev_micros = 0;
double dt;
double theta = 0;
double prev_xC = 0;
double prev_phi = 0;
double theta_dot = 0;
double xC_dot = 0;
double xC_dot_old = 0;
double phi_dot = 0;
double phi_dot_old = 0;
double xC_r = 0;
double phi_r = 0;
double xC_dot_r;
double phi_dot_r;
double xC_dot_r_prev = 0;
double phi_dot_r_prev = 0;
double vSum;
double vDiff;
double vL;
double vR;
int pwmL;
int pwmR;
static char buffer[14];
int chars;
char theta_char[6];
char theta_dot_char[6];

// Matrices (designed for dt = 8.068ms)
#define k (2) // polynomial degree
#define m (15) // # points to fit (must be odd)
int z[m];
double JtranJInvJtran[k + 1][m] = {
  {-70.5882,  -11.7647,   38.0090,   78.7330,  110.4072,  133.0317, 146.6063,  151.1312,  146.6063,  133.0317,  110.4072,   78.7330, 38.0090,  -11.7647,  -70.5882},
  {-25.0000,  -21.4286,  -17.8571,  -14.2857,  -10.7143,   -7.1429, -3.5714,         0,    3.5714,    7.1429,   10.7143,   14.2857, 17.8571,   21.4286,   25.0000},
  {7.3529,    4.2017,    1.5352,   -0.6464,   -2.3432,   -3.5553, -4.2825,   -4.5249,   -4.2825,   -3.5553,   -2.3432,   -0.6464, 1.5352,    4.2017,    7.3529}
};
double theta_sample[m];
double xC_sample[m];
double phi_sample[m];
double theta_Coeffs[m];
double xC_Coeffs[m];
double phi_Coeffs[m];
//double J[m][k + 1];
//double Jtran[k + 1][m];
//double JtranJInv[k + 1][k + 1];

//Constants
double theta_0 = -0.032638-0.001047; // -0.75 //-2.37 / 100 //+1.295
double r = 0.0508;
double circum = 2 * 3.14159265359 * r;
double d = 0.498475;
double gear_ratio = 5.9;
double cpr = 500;
double shaft_cpr = gear_ratio * cpr;
double vMax = 22.2; // 12.7 21.8

// Control parameters
//double ktheta1=12;   //12   //10   //14++++
double ktheta2 = 60; // 55 //15.5 //14.5 //16.5
double ktheta3 = 150; //250  //250  //200  //300
double ktheta4 = 25; //30 //30 //30   //30
double ktheta_aug = -55; //-30
double kphi1 = 0;
double kphi2 = 0;
double bias = 0;

//// Filter cutoff frequencies
double omega_enc = 100; //100
double alpha_enc;

// Smoothing parameters
const int numReadings = 40;
double readings[numReadings];      // the readings from the analog input
double readings2[numReadings];
int index = 0;                  // the index of the current reading
double total = 0;                  // the running total
double total2 = 0;

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180

void setup() {
  Serial.begin(115200);
  // Pin Modes
  pinMode(encoderL_PinA, INPUT);
  pinMode(encoderR_PinA, INPUT);
  pinMode(encoderL_PinB, INPUT);
  pinMode(encoderR_PinB, INPUT);
  pinMode(encoderL_Index, INPUT);
  pinMode(encoderR_Index, INPUT);
  pinMode(hbridgeL_PWM, OUTPUT);
  pinMode(hbridgeR_PWM, OUTPUT);
  pinMode(hbridgeL_Dir, OUTPUT);
  pinMode(hbridgeR_Dir, OUTPUT);
  // Initialize
  digitalWrite(encoderL_PinA, LOW);
  digitalWrite(encoderR_PinA, LOW);
  digitalWrite(encoderL_PinB, LOW);
  digitalWrite(encoderR_PinB, LOW);
  digitalWrite(encoderL_Index, LOW);
  digitalWrite(encoderR_Index, LOW);
  digitalWrite(hbridgeL_PWM, LOW);
  digitalWrite(hbridgeR_PWM, LOW);
  digitalWrite(hbridgeL_Dir, LOW);
  digitalWrite(hbridgeR_Dir, LOW);
  // Interrupt Setup
  attachInterrupt(encoderL_Interrupt, doEncoderL, RISING);
  attachInterrupt(encoderR_Interrupt, doEncoderR, RISING);

  // PWM Frequency settings (62.5kHz)
  TCCR1A = _BV (WGM10) | _BV (WGM11) | _BV (COM1A1) | _BV (COM1B1);
  TCCR1B &= ~7;  // clear prescaler
  TCCR1B |= _BV (CS10);  // prescaler of 1
  //  TCCR1A |= (1<<WGM11) | (1<<WGM10);
  //  TCCR1B |= (1<<WGM12);
  //  TCCR1B &= ~(1<<WGM13);
  //  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  //  TCCR1B = _BV(CS10);
  //  OCR1A = 0;
  //  OCR1B = 0;

  // Define z
  for (int i = 0; i < m; i++) {
    z[i] = i - (m + 1) / 2;
  }
  for (int i = 0; i < k + 1; i++) {
    for (int j = 0; j < m; j++) {
      JtranJInvJtran[i][j] = JtranJInvJtran[i][j] / 1000;
    }
  }

  // Jacobian
  //  for (int i = 1; i <= m; i++) {
  //    for (int j = 1; j <= k + 1; j++) {
  //      J[i][j] = pow(z[i],j - 1);
  //    }
  //  }
  //  Matrix.Transpose((double*)J, m, k + 1, (double*)Jtran);
  //  Matrix.Multiply((double*)Jtran, (double*)J, k + 1, m, k + 1, (double*)JtranJInv);
  //  Matrix.Invert((double*)JtranJInv, k + 1);
  //  Matrix.Multiply((double*)JtranJInv, (double*)Jtran, k + 1, k + 1, m, (double*)JtranJInvJtran);

  // initialize all the readings to 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    readings2[thisReading] = 0;
  }

}

void loop() {

  // Collect full data line from serial buffer
  while (Serial.available ()) { Serial.read (); } // clear serial buffer to avoid overflow
  do {
    chars = readline(Serial.read(), buffer, 14);
  } while (chars < 13);
  
  // Convert buffer line char array into double variables
  for (int i = 0; i < 6; i++) { theta_char[i] = buffer[i]; }
  for (int i = 0; i < 6; i++) { theta_dot_char[i] = buffer[i+7]; }
  theta = TO_RAD(atof(buffer) / 100) - theta_0;
  theta_dot = TO_RAD(atof(theta_dot_char) / 100);
  
  // Update time step
  dt = (micros() - prev_micros) / 1000000;
  prev_micros = micros();

  // Encoder LPF
//  alpha_enc = dt / (1 / omega_enc + dt);
//  encoderL_Ticks_filt = alpha_enc * encoderL_Ticks + (1 - alpha_enc) * encoderL_Ticks_filt_prev;
//  encoderR_Ticks_filt = alpha_enc * encoderR_Ticks + (1 - alpha_enc) * encoderR_Ticks_filt_prev;
//  encoderL_Ticks_filt_prev = encoderL_Ticks_filt;
//  encoderR_Ticks_filt_prev = encoderR_Ticks_filt;
  
  // Get wheel positions xC and phi
  revsL = -encoderL_Ticks / shaft_cpr;
  revsR = -encoderR_Ticks / shaft_cpr;
  xL = revsL * circum;
  xR = revsR * circum;
  xC = (xR + xL) / 2;
  phi = asin((xL - xR) / d);

  // Calculate velocities using Savitzky-Golay Filter
//  if (loops == 0) {
//    for (int i = 0; i < m; i++) {
//      theta_sample[i] = theta;
//      xC_sample[i] = xC;
//      phi_sample[i] = phi;
//    }
//  }
//  for (int i = 0; i < m - 1; i++) {
//    theta_sample[i] = theta_sample[i + 1];
//    xC_sample[i] = xC_sample[i + 1];
//    phi_sample[i] = phi_sample[i + 1];
//  }
//  theta_sample[m - 1] = theta;
//  xC_sample[m - 1] = xC;
//  phi_sample[m - 1] = phi;
//  theta_dot = 0;
//  xC_dot = 0;
//  phi_dot = 0;
//  for (int i = 1; i < k + 1; i++) {
//    theta_Coeffs[i] = 0;
//    xC_Coeffs[i] = 0;
//    phi_Coeffs[i] = 0;
//    for (int j = 0; j < m; j++) {
//      theta_Coeffs[i] += JtranJInvJtran[i][j] * theta_sample[j];
//      xC_Coeffs[i] += JtranJInvJtran[i][j] * xC_sample[j];
//      phi_Coeffs[i] += JtranJInvJtran[i][j] * phi_sample[j];
//    }
//    theta_dot += i * theta_Coeffs[i] * pow(z[m - 1], i - 1) / dt;
//    xC_dot += i * xC_Coeffs[i] * pow(z[m - 1], i - 1) / dt;
//    phi_dot += i * phi_Coeffs[i] * pow(z[m - 1], i - 1) / dt;
//  }

  // Calculate smoothed velocities
  total= total - readings[index];
  total2= total2 - readings2[index];
  readings[index] = (xC - prev_xC) / dt;
  readings2[index] = (phi - prev_phi) / dt;
  prev_xC = xC;
  prev_phi = phi;
  total= total + readings[index];
  total2= total2 + readings2[index];
  index = index + 1;
  if (index >= numReadings) {index = 0;}
  xC_dot = total / numReadings;
  phi_dot = total2 / numReadings;

  // Calculate finite difference velocities
//  xC_dot = (xC - prev_xC) / dt;
//  phi_dot = (phi - prev_phi) / dt;
//  prev_xC = xC;
//  prev_phi = phi;

  // Set reference states
  xC_dot_r = 0; //0.0254
  phi_dot_r = 0;
  xC_r += dt * (xC_dot_r + xC_dot_r_prev) / 2;
  phi_r += dt * (phi_dot_r + phi_dot_r_prev) / 2;
  xC_dot_r_prev = xC_dot_r;
  phi_dot_r_prev = phi_dot_r;

  // Apply linear state feeedback control
  vSum = ktheta2 * xC_dot + ktheta3 * theta + ktheta4 * theta_dot + ktheta_aug * (xC_r - xC);
  vDiff = kphi1 * (phi - phi_r) + kphi2 * phi_dot;
  vL = (vSum - vDiff) / 2;
  vR = (vSum + vDiff) / 2;

  // Set h-bridge direction
  if (vL >= 0) {
    digitalWriteFast(hbridgeL_Dir, LOW);
    vL = vL+bias;
  }
  else {
    digitalWriteFast(hbridgeL_Dir, HIGH);
    vL = vL-bias;
  }
  if (vR >= 0) {
    digitalWriteFast(hbridgeR_Dir, HIGH);
    vR = vR+bias;
  }
  else {
    digitalWriteFast(hbridgeR_Dir, LOW);
    vR = vR-bias;
  }

  // Limit PWM voltages to be <=|Vmax|
  if (vL < -vMax) { vL = -vMax; }
  if (vL > vMax) { vL = vMax; }
  if (vR < -vMax) { vR = -vMax; }
  if (vR > vMax) { vR = vMax; }

  // Set h-bridge PWM
  OCR1B = abs(vL) * 1023 / vMax;
  OCR1A = abs(vR) * 1023 / vMax;

  // Increment loop count
  ++loops;

  // Optional serial monitoring
//  Serial.print(dt * 1000); //each serial print costs ~0.41ms
//  Serial.print("\t");
//  Serial.print(theta*100,3);
//  Serial.print("\t");
//  Serial.print(theta_dot*100,3);
//  Serial.print("\n");

  // Optional delay
//  delay(4.5); //2.72
}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

// Left Encoder Interrupt
void doEncoderL() {
  // Test transition
  encoderL_BSet = digitalReadFast(encoderL_PinB);
  // and adjust counter + if A leads B
#ifdef encoderL_IsReversed
  encoderL_Ticks -= encoderL_BSet ? -1 : +1;
#else
  encoderL_Ticks += encoderL_BSet ? -1 : +1;
#endif
}

// Right Encoder Interrupt
void doEncoderR() {
  // Test transition
  encoderR_BSet = digitalReadFast(encoderR_PinB);
  // and adjust counter + if A leads B
#ifdef encoderR_IsReversed
  encoderR_Ticks -= encoderR_BSet ? -1 : +1;
#else
  encoderR_Ticks += encoderR_BSet ? -1 : +1;
#endif
}
