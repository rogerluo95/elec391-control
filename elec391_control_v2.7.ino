//--------------------------------------------------------------------------
//Control code V2.0 for ELEC 391 Group Project Final Demo
//Author: The University of British Columbia
//        Sichen (Roger) Luo  #45323136
//        Daniel Li           #50995133
//        Hailee Renkers      #56870132
// Revisions History:
// 17.03.16 V2.0: Include 2.0 Library, Exclude Hardware Pin Defines, Same Features
// 17.03.21 V2.1: Implementation of Virtual Wall Boundaries, with new variables for IN&OUT
// 17.03.24 V2.2: Working Gearshift: To be improved in the next version
// 17.03.27 V2.3: Adding Rocker Switch Protection for switching between modes
// 17.03.28 V2.4: Adding Velocity Sensing for unstable conditions 
// 17.03.29 V2.5: Brand New version for the Finalized PCB
// 17.03.31 V2.6: Trial with PID for N-D Region 
// 17.04.03 V2.7: Modified Interrupt for Mega
//--------------------------------------------------------------------------

//Includes
#include <math.h>
#include <Haptics.h>

//User Input Variables
int mode, mode_last;
int handle_force;

//Position Sensing Variables
int calibrated_left, calibrated_right;
int sensor_left_1, sensor_left_2;
int sensor_right_1, sensor_right_2;
int sensor_state_left, sensor_state_right;
int sensor_state_left_last, sensor_state_right_last;
long pulse_count_left, pulse_count_right;

//Position Tracking Variables
JointSpace q_in_cm, q;
JointSpace q_last, q_delta, q_prime;
WorkSpace p_in_cm, p, p_last;
WorkSpace p_v, p_v_last, p_v_last_last;

//State Tracking Variables
int path; 
int path_total = 6;
int gear, gear_next;
int path_haptics;
int path_haptics_total;

//Force Output Variables
Jacobian j;
WorkSpace force_p;
JointSpace force_q, pwm;

// --------------------------------------------------------------
// Setup function
// --------------------------------------------------------------
void setup() {

  Serial.begin(9600);
  // Setup Optical Encoder
  //attachInterrupt(2, updatePositionLeft, CHANGE);
  //attachInterrupt(3, updatePositionLeft, CHANGE);
  attachInterrupt(0, updatePositionLeft, CHANGE);
  attachInterrupt(1, updatePositionLeft, CHANGE);
  attachInterrupt(4, updatePositionRight, CHANGE);
  attachInterrupt(5, updatePositionRight, CHANGE);
  // Setup Limit & Rocker Switches
  pinMode(LIM_SWITCH_1, INPUT);
  pinMode(LIM_SWITCH_2, INPUT);
  pinMode(ROCKER_SWITCH, INPUT);
  // Setup Coils
  pinMode(SOL_L_EN, OUTPUT);
  pinMode(SOL_R_EN, OUTPUT);
  pinMode(SOL_L_1, OUTPUT);
  pinMode(SOL_L_2, OUTPUT);
  pinMode(SOL_R_1, OUTPUT);
  pinMode(SOL_R_2, OUTPUT);
  calibrated_left = FALSE;
  calibrated_right = FALSE;
  path = 1;
  path_haptics = 1;
  mode_last = digitalRead(ROCKER_SWITCH);
  while (!calibrated_left) {
    Haptics.calibrateLeft(&calibrated_left, &pulse_count_left);
  }
  while (!calibrated_right) {
    Haptics.calibrateRight(&calibrated_right, &pulse_count_right);
  }
  gear = INIT;
  gear_next = INIT;
}

// --------------------------------------------------------------
// Execution
// --------------------------------------------------------------
void loop() {
  // 0. User Inputs
  Haptics.readModeAndForce(&mode, &handle_force);
  // 1. Pulse Count from Optical Encoder
  // 2. Joint Space Position from Pulse Count
  q_in_cm = Haptics.getQInCm(&pulse_count_left, &pulse_count_right);
  q = Haptics.getQInM(q_in_cm);
  q_delta = Haptics.subQ(q, q_last);

  // 3. Work Space Position from Joint Space
  p = Haptics.dirKin(q);
  p_in_cm = Haptics.getPInCm(p);
  p_v = Haptics.getHandleVelocity(p_v_last_last,p_v_last,p_last,p);
  j = Haptics.getJacobian(q, q_delta, p);
  // Print Variables
  Haptics.PrintPositions(pulse_count_left, pulse_count_right,
                         q, q_in_cm, p_in_cm, p, p_v, j);      
                                    
  q_last = q;
  p_last = p;
  p_v_last_last = p_v_last;
  p_v_last = p_v;

  // 4. Haptics Rendering
  // Gearshift
  if(mode == 1 && mode_last == 1){
    if(path_haptics != 2){
      Haptics.doHapticsInitialization(&gear, &path_haptics, &path_haptics_total, p_in_cm, &force_p, &force_q, &pwm);
      }
    else{
       Haptics.renderGearShift(&gear, &gear_next, p_in_cm, p_v, &force_p, &force_q, &pwm);
      }
  // 5. Inverse Kinematics
  // 6. Motor Acturation
  Haptics.printForces(path, path_total, gear, gear_next,
                      force_p, force_q, pwm);
  mode_last = 1;
  }
  else if (mode == 0 && mode_last == 0) {
  Haptics.doAutonomousMotion(&path, &path_total, p_in_cm, &force_p, &force_q, &pwm);  
  Haptics.printForces(path, path_total, gear, gear_next,
                     force_p, force_q, pwm);
  mode_last = 0;
  }
  else {
    Haptics.goHome();
    setup();
    }
}

void updatePositionLeft () {
  sensor_left_1 = digitalRead(OPT_ENC_L1);
  sensor_left_2 = digitalRead(OPT_ENC_L2);
  Haptics.checkSensorStateLeft(&sensor_left_1, &sensor_left_2,
                               &sensor_state_left, &sensor_state_left_last, &pulse_count_left);
}

void updatePositionRight () {
  sensor_right_1 = digitalRead(OPT_ENC_R1);
  sensor_right_2 = digitalRead(OPT_ENC_R2);
  Haptics.checkSensorStateRight(&sensor_right_1, &sensor_right_2,
                                &sensor_state_right, &sensor_state_right_last, &pulse_count_right);
}



