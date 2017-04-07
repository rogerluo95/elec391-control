
/****************************************************************************
  * Copyright (c) 2017 LTech
  * All rights reserved.
  *@file      Haptics.cpp
  *@          Arduino Library for ELEC 391
  *@author    Roger (Sichen) Luo, The University of British Columbia
  *@date      March, 2017

  *History:
  *1.0 Testing Library Functionalities
  */

/******************************** Inclusions *********************************/
#include <Arduino.h>
#include <Haptics.h>
#include <math.h>

/*********************************************************************************************************
** Function name:           HapticClass
** Descriptions:            C++ Wrapper
*********************************************************************************************************/

HapticsClass::HapticsClass(){
}

void HapticsClass::calibrateLeft(int* calibrated_left, long* pulse_count_left){
  int limit_sw1_pressed = digitalRead(LIM_SWITCH_1);

  if (limit_sw1_pressed == 1){
    *pulse_count_left = 0;
    *calibrated_left = TRUE;
    Serial.println("1,");
  }
  else{
    *calibrated_left = FALSE;
    Serial.println("0,");
  }
}

void HapticsClass::calibrateRight(int* calibrated_right, long* pulse_count_right){
  int limit_sw2_pressed = digitalRead(LIM_SWITCH_2);

  if (limit_sw2_pressed == 0){
    *pulse_count_right = 0;
    *calibrated_right = TRUE;
    Serial.println("2,");
    delay(3000);
  }
  else{
    *calibrated_right = FALSE;
    Serial.println("1,");
  }
}

void HapticsClass::readModeAndForce (int *mode, int *handle_force){
  *mode = digitalRead(ROCKER_SWITCH);
  if (*mode == 0){
    Serial.print("3,");
  }
  else if (*mode == 1){
    Serial.print("4,");
  }
  *handle_force = analogRead(FORCE_SENSOR);
  Serial.print(*handle_force);
  Serial.print(",");
}

void HapticsClass::checkSensorStateLeft(int *sensor_1, int *sensor_2,
    int *sensor_state_left, int*sensor_state_left_last, long*pulse_count_left){
      if (*sensor_1 == 1 && *sensor_2 == 1){
        *sensor_state_left = 0;
        if (*sensor_state_left_last == 1){
          *pulse_count_left = *pulse_count_left -1;
        }
        if (*sensor_state_left_last == 3){
          *pulse_count_left = *pulse_count_left +1;
        }
        *sensor_state_left_last = 0;
      }
      if (*sensor_1 == 0 && *sensor_2 == 1){
        *sensor_state_left = 1;
        if (*sensor_state_left_last == 2){
          *pulse_count_left = *pulse_count_left -1;
        }
        if (*sensor_state_left_last == 0){
          *pulse_count_left = *pulse_count_left +1;
        }
        *sensor_state_left_last = 1;
      }

      if (*sensor_1 == 0 && *sensor_2 == 0){
        *sensor_state_left = 2;
        if (*sensor_state_left_last == 3){
          *pulse_count_left = *pulse_count_left -1;
        }
        if (*sensor_state_left_last == 1){
          *pulse_count_left = *pulse_count_left +1;
        }
        *sensor_state_left_last = 2;
      }

      if (*sensor_1 == 1 && *sensor_2 == 0){
        *sensor_state_left = 3;
        if (*sensor_state_left_last == 0){
          *pulse_count_left = *pulse_count_left -1;
        }
        if (*sensor_state_left_last == 2){
          *pulse_count_left = *pulse_count_left +1;
        }
        *sensor_state_left_last = 3;
      }
}

void HapticsClass::checkSensorStateRight(int *sensor_1, int *sensor_2,
    int *sensor_state_right, int*sensor_state_right_last, long*pulse_count_right){
      if (*sensor_1 == 1 && *sensor_2 == 1){
        *sensor_state_right = 0;
        if (*sensor_state_right_last == 1){
          *pulse_count_right = *pulse_count_right -1;
        }
        if (*sensor_state_right_last == 3){
          *pulse_count_right = *pulse_count_right +1;
        }
        *sensor_state_right_last = 0;
      }
      if (*sensor_1 == 0 && *sensor_2 == 1){
        *sensor_state_right = 1;
        if (*sensor_state_right_last == 2){
          *pulse_count_right = *pulse_count_right -1;
        }
        if (*sensor_state_right_last == 0){
          *pulse_count_right = *pulse_count_right +1;
        }
        *sensor_state_right_last = 1;
      }

      if (*sensor_1 == 0 && *sensor_2 == 0){
        *sensor_state_right = 2;
        if (*sensor_state_right_last == 3){
          *pulse_count_right = *pulse_count_right -1;
        }
        if (*sensor_state_right_last == 1){
          *pulse_count_right = *pulse_count_right +1;
        }
        *sensor_state_right_last = 2;
      }

      if (*sensor_1 == 1 && *sensor_2 == 0){
        *sensor_state_right = 3;
        if (*sensor_state_right_last == 0){
          *pulse_count_right = *pulse_count_right -1;
        }
        if (*sensor_state_right_last == 2){
          *pulse_count_right = *pulse_count_right +1;
        }
        *sensor_state_right_last = 3;
      }
}

JointSpace HapticsClass::getQInCm(long *pulse_count_left, long *pulse_count_right){
  JointSpace result;
  result.q1 = (*pulse_count_left+1)/9.4 - 10;
  result.q2 = *pulse_count_right/9.4 + 10;
  return result;
}

JointSpace HapticsClass::getQInM(JointSpace q){
  JointSpace result;
  result.q1 = q.q1/100;
  result.q2 = q.q2/100;
  return result;
}

JointSpace HapticsClass::getQInCmForTest(JointSpace q){
  JointSpace result;
  result.q1 = q.q1*100;
  result.q2 = q.q2*100;
  return result;
}

void HapticsClass::PrintPositions(long pulse_count_left, long pulse_count_right,
                                  JointSpace q_in_cm, JointSpace q, WorkSpace p_in_cm, WorkSpace p, WorkSpace p_v, Jacobian j){
    Serial.print(pulse_count_left); Serial.print(",");
    Serial.print(pulse_count_right); Serial.print(",");
    Serial.print(q_in_cm.q1); Serial.print(",");
    Serial.print(q_in_cm.q2); Serial.print(",");
    Serial.print(q.q1); Serial.print(",");
    Serial.print(q.q2); Serial.print(",");
    Serial.print(p_in_cm.x); Serial.print(",");
    Serial.print(p_in_cm.y); Serial.print(",");
    Serial.print(p.x); Serial.print(",");
    Serial.print(p.y); Serial.print(",");
    Serial.print(p_v.x); Serial.print(",");
    Serial.print(p_v.y); Serial.print(",");
    Serial.print(j.j11); Serial.print(",");
    Serial.print(j.j12); Serial.print(",");
    Serial.print(j.j21); Serial.print(",");
    Serial.print(j.j22); Serial.print(",");
}

JointSpace HapticsClass::subQ(JointSpace a, JointSpace b){
  JointSpace result;
  result.q1 = a.q1 - b.q1;
  result.q2 = a.q2 - b.q2;
  return result;
}

JointSpace HapticsClass::getQPrime(JointSpace a, JointSpace b){
  JointSpace result;
  result.q1 = a.q1 + b.q1;
  result.q2 = a.q2;
  return result;
}

JointSpace HapticsClass::getQDoublePrime(JointSpace a, JointSpace b){
  JointSpace result;
  result.q1 = a.q1;
  result.q2 = a.q2 + b.q2;
  return result;
}

WorkSpace HapticsClass::dirKin(JointSpace q){
  WorkSpace result;

  result.x = (q.q1 + q.q2)/2;
  double diff_q = abs (q.q1 - q.q2) /2;
  result.y = sqrt (ARM * ARM - diff_q * diff_q);
  return result;
}

WorkSpace HapticsClass::getPInCm(WorkSpace p){
  WorkSpace result;
  result.x = p.x * 100;
  result.y = p.y * 100;
  return result;
}

WorkSpace HapticsClass::subP(WorkSpace a, WorkSpace b){
  WorkSpace result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  return result;
}

WorkSpace HapticsClass::getHandleVelocity(WorkSpace p_v_last_last, WorkSpace p_v_last, WorkSpace p_last, WorkSpace p){
  WorkSpace p_v;

  p_v.x = -(0.45*0.45)*(p_v_last_last.x) + 2*0.45*(p_v_last.x) + (1-0.45)*(1-0.45)*(p.x-p_last.x)/0.001;
  p_v.y = -(0.45*0.45)*(p_v_last_last.y) + 2*0.45*(p_v_last.y) + (1-0.45)*(1-0.45)*(p.y-p_last.y)/0.001;
  return p_v;
}

Jacobian HapticsClass::getJacobian(JointSpace q, JointSpace q_delta, WorkSpace p){
  Jacobian result;
  JointSpace q_prime;
  JointSpace q_double_prime;
  WorkSpace p_prime;
  WorkSpace p_double_prime;
  WorkSpace p_delta_prime;
  WorkSpace p_delta_double_prime;

  q_prime = getQPrime(q, q_delta);
  q_double_prime = getQDoublePrime(q, q_delta);
  p_prime = dirKin(q_prime);
  p_delta_prime = subP(p_prime,p);
  p_double_prime = dirKin(q_double_prime);
  p_delta_double_prime = subP(p_double_prime,p);

  result.j11 = p_delta_prime.x / q_delta.q1;
  result.j12 = p_delta_double_prime.x / q_delta.q2;
  result.j21 = p_delta_prime.y / q_delta.q1;
  result.j22 = p_delta_double_prime.y / q_delta.q2;
  return result;
}

void HapticsClass::renderGearShift(int *gear, int *gear_next, WorkSpace p, WorkSpace p_v, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm){
  int current_gear;
  if(*gear == N){
    if (p.y <= CENTRAL_DOWN){
      if(p.y < D_DOWN){
        //fullStop(force_p, force_q, pwm);
        slightDown(force_p,force_q,pwm);
        *gear = D;
        *gear_next = D;
      }
      else {
        if (p_v.y <= -2.0){
          if(p_v.y <= -8.0){
            fullStop(force_p,force_q,pwm);
          }
          else{
            //fullUp(force_p,force_q,pwm);
            moveDown(255,100,50,5,20);
          }
        }
        else{
          slightUp(force_p,force_q,pwm);
        }
        *gear = N;
        *gear_next = D;
      }
    }
    else if (p.y >= CENTRAL_UP){
      if(p.y > CENTRAL_UP + 0.5){
        fullStop(force_p, force_q, pwm);
        *gear = R;
        *gear_next = R;
      }
      else {
        if(p_v.y >= 4.0){
          fullStop(force_p, force_q, pwm);
        }
        else{
          moveUp(100,50,5,5,18);
        }
        *gear = N;
        *gear_next = R;
      }
    }
    else if (p.x <= CENTRAL_LEFT){
      if(p.x <= EV_RIGHT){
        fullRight(force_p,force_q,pwm);
        delay(20);
        *gear = EV;
        *gear_next = EV;
      }
      else{
        if (p_v.x <=-2.0){
          if (p_v.x <=-10.0){
            fullStop(force_p,force_q,pwm);
          }
          else{
            moveLeft(255,100,50,5,25);
          }
        }
        else{
          moveLeft(255,100,50,5,25);
        }
        *gear = N;
        *gear_next = EV;
      }
    }
    else if (p.x >= CENTRAL_RIGHT){
      if(p.x > P_LEFT){
        fullLeft(force_p,force_q,pwm);
        delay(20);
        *gear = P;
        *gear_next = P;
      }
      else {
        if (p_v.x >=2.0){
          if(p_v.x >=10.0){
            fullStop(force_p,force_q,pwm);
          }
          else{
            moveRight(255,100,50,25,5);
          }
        }
        else{
          moveRight(255,100,50,25,5);
        }
        *gear = N;
        *gear_next = P;
      }
    }
    else {
      if(*gear_next == D){
        microDown(force_p, force_q, pwm);
        //slightDown(force_p,force_q,pwm);
        *gear = N;
        *gear_next = N;
      }
      else if (*gear_next == EV){
        microLeft(force_p, force_q, pwm);
        *gear = N;
        *gear_next = N;
      }
      else if (*gear_next == P){
        microRight(force_p, force_q, pwm);
        *gear = N;
        *gear_next = N;
      }
      else{
        fullStop(force_p, force_q, pwm);
        *gear = N;
        *gear_next = N;
      }
    }
  }
  if (*gear == D){
    if (p.y >= D_DOWN){
      if(p.y > CENTRAL_DOWN){
        //fullStop(force_p, force_q, pwm);
        slightUp(force_p,force_q,pwm);
        *gear = N;
        *gear_next = N;
      }
      else {
        fullDown(force_p,force_q,pwm);
        //moveUp(255,100,50,20,5);
        *gear = D;
        *gear_next = N;
      }
    }
    else {
      fullStop(force_p, force_q, pwm);
      *gear = D;
      *gear_next = D;
    }
  }
  if(*gear == EV){
    if (p.x >= EV_RIGHT){
      if(p.x > CENTRAL_LEFT){
        microRight(force_p,force_q,pwm);
        *gear = N;
        *gear_next = N;
      }
      else {
        moveRight(255,100,50,25,5);
        *gear = EV;
        *gear_next = N;
      }
    }
    else if (p.y <= CENTRAL_DOWN){
      if(p.y < D_UP){
        //fullStop(force_p, force_q, pwm);
        slightDown(force_p,force_q,pwm);
        *gear = D;
        *gear_next = D;
      }
      else {
        fullUp(force_p,force_q,pwm);
        *gear = EV;
        *gear_next = D;
      }
    }
    else {
      fullStop(force_p, force_q, pwm);
      *gear = EV;
      *gear_next = EV;
    }
  }
  if(*gear == P){
    if (p.x <= P_LEFT){
      if(p.x < CENTRAL_RIGHT){
        microLeft(force_p,force_q,pwm);
        *gear = N;
        *gear_next = N;
      }
      else {
        moveLeft(255,100,50,5,25);
        *gear = P;
        *gear_next = N;
      }
    }
    else if (p.y <= CENTRAL_DOWN){
      if(p.y < D_UP){
        //fullStop(force_p, force_q, pwm);
        slightDown(force_p,force_q,pwm);
        *gear = D;
        *gear_next = D;
      }
      else {
        fullUp(force_p,force_q,pwm);
        *gear = P;
        *gear_next = D;
      }
    }
    else {
      fullStop(force_p, force_q, pwm);
      *gear = P;
      *gear_next = P;
    }
  }
  if (*gear_next == R){
    *gear = R;
  }
  if (*gear == R){
    if (p.y <= CENTRAL_UP){
      microUp(force_p, force_q, pwm);
    }
    else{
      moveDown(160,100,30,5,18);
    }
    if (p.y <= CENTRAL_DOWN){
      if(p.y < D_DOWN){
        slightDown(force_p,force_q,pwm);
        *gear = D;
        *gear_next = D;
      }
      else {
        moveDown(255,100,50,5,20);
        *gear = N;
        *gear_next = D;
      }
    }
    else if (p.x <= CENTRAL_LEFT){
      if(p.x <= EV_RIGHT){
        slightLeft(force_p,force_q,pwm);
        *gear = EV;
        *gear_next = EV;
      }
      else{
        moveLeft(255,100,50,5,25);
        *gear = N;
        *gear_next = EV;
      }
    }
    else if (p.x >= CENTRAL_RIGHT){
      if(p.x > P_LEFT){
        slightRight(force_p,force_q,pwm);
        *gear = P;
        *gear_next = P;
      }
      else {
        moveLeft(255,100,50,5,25);
        *gear = N;
        *gear_next = P;
      }
    }
  }
}

JointSpace HapticsClass::invKin(WorkSpace force, Jacobian j){
  JointSpace result;
  result.q1 = j.j11 * force.x + j.j21 * force.y;
  result.q2 = j.j12 * force.x + j.j22 * force.y;
  return result;
}

JointSpace HapticsClass::invKinPosition(WorkSpace p){
  JointSpace result;
  double delta_q = sqrt(ARM*ARM - p.y * p.y);
  result.q1 = p.x - delta_q;
  result.q2 = p.x + delta_q;
  return result;
}

void HapticsClass::fullUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = 1;
  force_q->q1 = 1;
  force_q->q2 =  -1;
  pwm->q1 = 230;
  pwm->q2 = -230;
  applyForce(*pwm);
}

void HapticsClass::fullDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = -1;
  force_q->q1 = -1;
  force_q->q2 =  1;
  pwm->q1 = -230;
  pwm->q2 = 230;
  applyForce(*pwm);
}

void HapticsClass::fullLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = -2;
  force_p->y = 0;
  force_q->q1 = -1;
  force_q->q2 =  -1;
  pwm->q1 = -200;
  pwm->q2 = -200;
  applyForce(*pwm);
}

void HapticsClass::fullRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 2;
  force_p->y = 0;
  force_q->q1 = 1;
  force_q->q2 = 1;
  pwm->q1 = 200;
  pwm->q2 = 200;
  applyForce(*pwm);
}

void HapticsClass::slightUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = 1;
  force_q->q1 = 0.5;
  force_q->q2 =  -0.5;
  pwm->q1 = 100;
  pwm->q2 = -100;
  applyForce(*pwm);
}

void HapticsClass::slightDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = -1;
  force_q->q1 = -0.5;
  force_q->q2 =  0.5;
  pwm->q1 = -100;
  pwm->q2 = 100;
  applyForce(*pwm);
}

void HapticsClass::slightLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = -1;
  force_p->y = 0;
  force_q->q1 = -0.5;
  force_q->q2 =  -0.5;
  pwm->q1 = -100;
  pwm->q2 = -100;
  applyForce(*pwm);
}

void HapticsClass::slightRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 1;
  force_p->y = 0;
  force_q->q1 = 0.5;
  force_q->q2 = 0.5;
  pwm->q1 = 100;
  pwm->q2 = 100;
  applyForce(*pwm);
}

void HapticsClass::microUp(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = 0.25;
  force_q->q1 = 0.5;
  force_q->q2 =  -0.5;
  pwm->q1 = 20;
  pwm->q2 = -20;
  applyForce(*pwm);
}

void HapticsClass::microDown(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = -0.5;
  force_q->q1 = -0.25;
  force_q->q2 =  0.25;
  pwm->q1 = -30;
  pwm->q2 = 30;
  applyForce(*pwm);
}

void HapticsClass::microLeft(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = -0.5;
  force_p->y = 0;
  force_q->q1 = -0.25;
  force_q->q2 =  -0.25;
  pwm->q1 = -20;
  pwm->q2 = -20;
  applyForce(*pwm);
}

void HapticsClass::microRight(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0.5;
  force_p->y = 0;
  force_q->q1 = 0.25;
  force_q->q2 = 0.25;
  pwm->q1 = 20;
  pwm->q2 = 20;
  applyForce(*pwm);
}

void HapticsClass::fullStop(WorkSpace *force_p, JointSpace *force_q, JointSpace *pwm){
  force_p->x = 0;
  force_p->y = 0;
  force_q->q1 = 0;
  force_q->q2 = 0;
  pwm->q1 = 0;
  pwm->q2 = 0;
  applyForce(*pwm);
}

JointSpace HapticsClass::getPWM(JointSpace force_q){
  JointSpace result;
  result.q1 = force_q.q1;
  result.q2 = force_q.q2;
  return result;
}

void HapticsClass::applyForce(JointSpace pwm){
  if (pwm.q1 < 0){
    if (pwm.q1 < -250){
      driveMotorLeft(255, LOW, HIGH);
    }
    else{
      driveMotorLeft(-pwm.q1, LOW, HIGH);
    }
    }
    else if (pwm.q1 >= 0){
      if(pwm.q1 >= 0){
        if (pwm.q1 < 250){
          driveMotorLeft(pwm.q1, HIGH, LOW);
        }
        else{
          driveMotorLeft(255, HIGH, LOW);
        }
      }
    }
    if (pwm.q2 < 0){
      if (pwm.q2 < -250){
        driveMotorRight(255, HIGH, LOW);
      }
      else{
        driveMotorRight(-pwm.q2, HIGH, LOW);
      }
      }
      else if (pwm.q2 >= 0){
          if (pwm.q2 < 250){
            driveMotorRight(pwm.q2, LOW, HIGH);
          }
          else{
            driveMotorRight(255, LOW, HIGH);
          }
      }
  }

void HapticsClass::moveLeftMotorLeft(int pwm, int duration, int a, int b){
    int start_time = millis();
    int end_time = start_time;
    while((end_time - start_time)<= duration){
      //Right
      driveMotorLeft(pwm, HIGH, LOW);
      delay(a);
      //Left
      driveMotorLeft(pwm, LOW, HIGH);
      delay(b);
      end_time = millis();
    }
  }

void HapticsClass::moveLeftMotorRight(int pwm, int duration, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    //Right
    driveMotorLeft(pwm, HIGH, LOW);
    delay(a);
    //Left
    driveMotorLeft(pwm, LOW, HIGH);
    delay(b);
    end_time = millis();
  }
}

void HapticsClass::moveRightMotorLeft(int pwm, int duration, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    //Right
    driveMotorRight(pwm, HIGH, LOW);
    delay(a);
    //Left
    driveMotorRight(pwm, LOW, HIGH);
    delay(b);
    end_time = millis();
  }
}

void HapticsClass::moveRightMotorRight(int pwm, int duration, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    //Right
    driveMotorRight(pwm, HIGH, LOW);
    delay(a);
    //Left
    driveMotorRight(pwm, LOW, HIGH);
    delay(b);
    end_time = millis();
  }
}

//a = 10, b = 5
void HapticsClass::moveUp(int pwm, int duration, int frequency, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){

    moveLeftMotorRight(pwm,frequency,a,b);
    stopMotors();
    moveRightMotorLeft(pwm, frequency,a,b);
    stopMotors();
    end_time = millis();
  }
}

//a = 5, b = 10
void HapticsClass::moveDown(int pwm, int duration, int frequency, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    moveLeftMotorLeft(pwm,frequency,a,b);
    stopMotors();
    moveRightMotorRight(pwm, frequency,a,b);
    stopMotors();
    end_time = millis();
  }
}

//a = 5, b = 10
void HapticsClass::moveLeft(int pwm, int duration, int frequency, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    moveLeftMotorLeft(pwm,frequency,a,b);
    stopMotors();
    moveRightMotorLeft(pwm, frequency,b,a);
    stopMotors();
    end_time = millis();
  }
}

// a = 10, b = 5
void HapticsClass::moveRight(int pwm, int duration, int frequency, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    moveLeftMotorRight(pwm,frequency,a,b);
    stopMotors();
    moveRightMotorRight(pwm, frequency,b,a);
    stopMotors();
    end_time = millis();
  }
}

void HapticsClass::initializeGearPosition(int pwm, int duration, int frequency, int a, int b){
  int start_time = millis();
  int end_time = start_time;
  while((end_time - start_time)<= duration){
    moveLeftMotorRight(pwm,frequency,a,b);
    stopMotors();
    moveRightMotorRight(pwm, frequency,0,0);
    stopMotors();
    end_time = millis();
  }
}

void HapticsClass::doAutonomousMotion(int *path, int *path_total, WorkSpace p, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm){
  if(*path == 1){
    if ((p.x == 0) && (p.y>= 9.2 - 0.1)){
      *path = 2;
      delay(2000);
  }
  else{
    if (p.y < 9.2-0.1){
    moveUp(160,300,10,10,5);
  }
    if (p.y > 9.2+0.1){
    moveDown(160,300,10,5,10);
  }
    if (p.x< 0){
    moveRight(160,10,10,10,5);
  }
    if(p.x > 0){
    moveLeft(160,10,10,5,10);
  }
  *path = 1;
  }
  }
  if(*path == 2){
    if ((p.x <= 0.1 || p.x >= -0.1) && (p.y<= 8.5 + 0.15)){
      *path = 3;
      delay(2000);
  }
  else{
    if (p.y < 8.5-0.1){
    moveUp(160,100,10,10,5);
  }
    if (p.y > 8.5+0.1){
    moveDown(160,100,10,5,10);
  }
    if (p.x< 0){
    moveRight(160,10,10,10,5);
  }
    if(p.x > 0){
    moveLeft(160,10,10,5,10);
  }
  *path = 2;
  }
  }
  if(*path == 3){
    if ((p.x <= -2.252) && (p.y<= 8.5 + 0.15)){
      *path = 4;
      delay(2000);
  }
  else{
    if (p.y < 8.5-0.1){
    moveUp(160,10,10,10,5);
  }
    if (p.y > 8.5+0.1){
    moveDown(160,10,10,5,10);
  }
    if (p.x< -2.6){
    moveRight(160,500,10,10,5);
  }
    if(p.x > -2.4){
    moveLeft(160,500,10,5,10);
  }
  *path = 3;
  }
  }
  if(*path == 4){
    if ((p.x >= 2.25) && (p.y<= 8.5 + 0.15)){
      *path = 5;
      delay(2000);
  }
  else{
    if (p.y < 8.5-0.1){
    moveUp(160,10,10,10,5);
  }
    if (p.y > 8.5+0.1){
    moveDown(160,10,10,5,10);
  }
    if (p.x< 2.4){
    moveRight(160,500,10,10,5);
  }
    if(p.x > 2.6){
    moveLeft(160,500,10,5,10);
  }
  *path = 4;
  }
  }
  if(*path == 5){
    if ((p.x == 0) && (p.y<= 8.5 + 0.3)){
      *path = 6;
      delay(2000);
  }
  else{
    if (p.y < 8.5-0.1){
    moveUp(160,100,10,10,5);
  }
    if (p.y > 8.5+0.1){
    moveDown(160,100,10,5,10);
  }
    if (p.x< 0){
    moveRight(160,100,10,10,5);
  }
    if(p.x > 0){
    moveLeft(160,100,10,5,10);
  }
  *path = 5;
  }
  }
  if(*path == 6){
    if ((p.x <= 0.11 || p.x >= -0.11) && (p.y<= 6.2)){
      *path = 1;
      delay(2000);
  }
  else{
    if (p.y < 6.1){
    moveUp(160,100,10,10,5);
  }
    if (p.y > 6.2){
    moveDown(160,100,10,5,10);
  }
    if (p.x< 0){
    moveRight(160,20,10,10,5);
  }
    if(p.x > 0){
    moveLeft(160,20,10,5,10);
  }
  *path = 6;
  }
  }
}

void HapticsClass::doHapticsInitialization(int *gear, int *path_haptics, int*path_haptics_total, WorkSpace p, WorkSpace *force_p, JointSpace *force_q, JointSpace*pwm){
  if(*path_haptics == 1){
    if (p.x >= 3.3){
      *gear = P;
      *path_haptics = 2;
      delay(2000);
  }
  else{
    if (p.x< 3.3){
    initializeGearPosition(200,500,10,10,5);
  }
  *gear = INIT;
  *path_haptics = 1;
  }
  }
}

void HapticsClass::goHome(){
  moveDown(200,3000,10,5,10);
  stopMotors();
}

void HapticsClass::stopMotors(void){
  driveMotorLeft(0,HIGH,LOW);
  driveMotorRight(0,HIGH,LOW);
}

// LOW-HIGH for Moving Left & HIGH-LOW for Moving Right
void HapticsClass::driveMotorLeft(double duty_cycle_left, bool left_1, bool left_2){
  analogWrite(SOL_L_EN, duty_cycle_left);
  digitalWrite(SOL_L_1, left_1);
  digitalWrite(SOL_L_2, left_2);
}

// HIGH-LOW for Moving Left & LOW-HIGH for Moving Right
void HapticsClass::driveMotorRight(double duty_cycle_right, bool right_1, bool right_2){
  analogWrite(SOL_R_EN, duty_cycle_right);
  digitalWrite(SOL_R_1, right_1);
  digitalWrite(SOL_R_2, right_2);
}

void HapticsClass::printForces(int path,int path_total ,int gear, int gear_next, WorkSpace force_p, JointSpace force_q, JointSpace pwm){
    Serial.print(path); Serial.print(",");
    Serial.print(path_total); Serial.print(",");
    Serial.print(gear); Serial.print(",");
    Serial.print(gear_next); Serial.print(",");
    Serial.print(force_p.x); Serial.print(",");
    Serial.print(force_p.y); Serial.print(",");
    Serial.print(force_q.q1); Serial.print(",");
    Serial.print(force_q.q2); Serial.print(",");
    Serial.print(pwm.q1); Serial.print(",");
    Serial.print(pwm.q2); Serial.println(",");
}

HapticsClass Haptics;
