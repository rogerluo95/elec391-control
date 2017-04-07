/****************************************************************************
  * Copyright (c) 2017 LTech
  * All rights reserved.
  *
  *@file      Haptics.h
  *@header file for Haptics.cpp
  *
  *@author    Roger (Sichen) Luo, University of British Columbia
  *@date      March, 2017
  *
  */
/************************ Pin Definitions *  ***********************/
#ifndef haptics__h
#define haptics__h

#define ARM 0.11

#define TRUE 1
#define FALSE 0
//ON/OFF For Handle Vibration
#define ON 1
#define OFF 0
//AT/MT Mode Switching
#define AT 3
#define MT 4
//Virtual Wall Boundary
#define INIT 10
#define N 11
#define P 12
#define R 13
#define EV 14
#define D 15
#define S 16

//Virtual Wall Definations
#define GEAR_WALL 0.5
#define SLOT_WALL 1
#define S_WALL 5.5
// X AXIS
#define CENTRAL_LEFT -1.5
#define CENTRAL_RIGHT 1.5
#define EV_LEFT -3.5
#define EV_RIGHT -2.5
#define P_LEFT 2.5
#define P_RIGHT 3.5
// Y AXIS
#define R_UP 9.7
#define R_DOWN 9.55
#define CENTRAL_UP 9.55
#define CENTRAL_DOWN 7.15
#define D_UP 7.15
#define D_DOWN 5.2

// Handle Vibration Settings
#define LOW_FREQ 1
#define MID_FREQ 2
#define HIGH_FREQ 3

#define OPT_ENC_L1 2
#define OPT_ENC_L2 3
#define OPT_ENC_R1 18
#define OPT_ENC_R2 19

#define LIM_SWITCH_1 46
#define LIM_SWITCH_2 48
#define ROCKER_SWITCH 50

#define FORCE_SENSOR A0

#define SOL_L_EN 8
#define SOL_L_1 22
#define SOL_L_2 24

#define SOL_R_EN 9
#define SOL_R_1 26
#define SOL_R_2 28

typedef struct {
  double x;
  double y;
} WorkSpace;

typedef struct {
  double q1;
  double q2;
} JointSpace;

typedef struct {
  double j11;
  double j12;
  double j21;
  double j22;
} Jacobian;

class HapticsClass {
public:
  HapticsClass();
  void calibrateLeft(int* calibrated_right, long* pulse_count_right);
  void calibrateRight(int* calibrated_right, long* pulse_count_right);
  void readModeAndForce (int *mode, int *handle_force);
  void checkSensorStateLeft(int *sensor_1, int *sensor_2,
      int *sensor_state_left, int*sensor_state_left_last, long*pulse_count_left);
  void checkSensorStateRight(int *sensor_1, int *sensor_2,
      int *sensor_state_right, int*sensor_state_right_last, long*pulse_count_right);
  void PrintPositions(long pulse_count_left, long pulse_count_right,
            JointSpace q_in_cm, JointSpace q, WorkSpace p_in_cm, WorkSpace p, WorkSpace p_v, Jacobian j);
  JointSpace getQInCm(long *pulse_count_left, long *pulse_count_right);
  JointSpace getQInM(JointSpace q);
  JointSpace getQInCmForTest(JointSpace q);
  JointSpace subQ(JointSpace a, JointSpace b);
  JointSpace getQPrime(JointSpace a, JointSpace b);
  JointSpace getQDoublePrime(JointSpace a, JointSpace b);
  WorkSpace dirKin(JointSpace q);
  WorkSpace getPInCm(WorkSpace p);
  WorkSpace subP(WorkSpace a, WorkSpace b);
  WorkSpace getHandleVelocity(WorkSpace p_v_last_last, WorkSpace p_v_last, WorkSpace p_last, WorkSpace p);
  Jacobian getJacobian(JointSpace q, JointSpace q_delta, WorkSpace p);
  JointSpace invKin(WorkSpace force, Jacobian j);
  JointSpace invKinPosition(WorkSpace p);

  void fullUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void fullDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void fullLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void fullRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void slightUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void slightDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void slightLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void slightRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void microUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void microDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void microLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void microRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void fullStop(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm);
  void renderGearShift(int *gear, int *gear_next, WorkSpace p, WorkSpace p_v, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm);

  void applyForce(JointSpace pwm);
  void moveLeftMotorLeft(int pwm, int duration,int a, int b);
  void moveLeftMotorRight(int pwm, int duration, int a, int b);
  void moveRightMotorLeft(int pwm, int duration, int a, int b);
  void moveRightMotorRight(int pwm, int duration, int a, int b);
  void moveUp(int pwm, int duration, int frequency, int a, int b);
  void moveDown(int pwm, int duration, int frequency, int a, int b);
  void moveLeft(int pwm, int duration, int frequency, int a, int b);
  void moveRight(int pwm, int duration, int frequency, int a, int b);
  void initializeGearPosition(int pwm, int duration, int frequency, int a, int b);
  void doAutonomousMotion(int *path, int *path_total, WorkSpace p, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm);
  void doHapticsInitialization(int *gear, int *path_haptics, int*path_haptics_total, WorkSpace p, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm);
  void goHome(void);

  void stopMotors(void);
  JointSpace getPWM(JointSpace force_q);
  void driveMotorLeft(double duty_cycle_left, bool left_1, bool left_2);
  void driveMotorRight(double duty_cycle_right, bool right_1, bool right_2);

  void printForces(int path, int path_total, int handle_state, int handle_state_next, WorkSpace force_p,
            JointSpace force_q, JointSpace pwm);

private:
};

extern HapticsClass Haptics;

#endif
