#include <Robot_L298P.h> 


#define Kp 0.03 // 0.03
#define Kd 0.3 // 0.3

#define MAX_R_SPEED 100
#define MAX_L_SPEED 100 //95

#define K_R_SPEED 1
#define K_L_SPEED 1 //0.95

#define parrot_cm 157
#define parrot_angle 25

long int last_enc_A = 0,last_enc_B = 0;
long int motor_on_speed_L = 0,motor_on_speed_R = 0;
double motor_speed_A = 2.0,motor_speed_B = 2.0;
unsigned long int motor_time = 0;
#define MOTOR_DT 10


void setup() {
  Serial.begin (9600); 
  Robot.setup(); 
  Robot.reverse_motor_A();
  Robot.reverse_enc_B();
}

unsigned long int ttt = 0;

void loop() {
//  enc_forward(100);
//  enc_forward(-100);
//  enc_forward(30);
//  enc_left(90);
  if (millis()-ttt>3000) {
    motor_speed_A *= -1;
    motor_speed_B *= -1;
    //motor_speed_B += 0.1;
    ttt = millis();
  }
  update_motor();
}


void forward() {
  
}

void update_motor() {
  unsigned long int dt = millis()-motor_time;
  if (dt>MOTOR_DT) {
    double real_speed_A = double(Robot.enc_A - last_enc_A)/dt;
    last_enc_A = Robot.enc_A;
    long int eA = (motor_speed_A - real_speed_A)*100;
    double real_speed_B = double(Robot.enc_B - last_enc_B)/dt;
    last_enc_B = Robot.enc_B;
    long int eB = (motor_speed_B - real_speed_B)*100;
//    Serial.print(real_speed_A);
//    Serial.print(" ");
//    Serial.println(real_speed_B);
    PID_motor_speed(eA,eB);
    motor_time = millis();
  }
}

long int _Pr,_Dr,_PIDr,_eR_old;
long int _Pl,_Dl,_PIDl,_eL_old;

void PID_motor_speed(long int eL, long int eR) {

  _Pr = eR;
  _Dr = eR - _eR_old;
  _eR_old = eR;
  _PIDr = _Pr*Kp + _Dr*Kd;
  motor_on_speed_R += _PIDr;
  motor_on_speed_R = constrain(motor_on_speed_R,-MAX_R_SPEED,MAX_R_SPEED)*K_R_SPEED;
  if (motor_on_speed_R!=0) motor_on_speed_R -= motor_on_speed_R/abs(motor_on_speed_R);
//  Serial.print(eR);
//  Serial.print(" ");
//  Serial.print(_PIDr);
//  Serial.print(" ");
  //Serial.println(motor_on_speed_R);
  _Pl = eL;
  _Dl = eL - _eL_old;
  _eL_old = eL;
  _PIDl = _Pl*Kp + _Dl*Kd;
  motor_on_speed_L += _PIDl;
  motor_on_speed_L = constrain(motor_on_speed_L,-MAX_R_SPEED,MAX_R_SPEED)*K_L_SPEED;
  if (motor_on_speed_L!=0) motor_on_speed_L -= motor_on_speed_L/abs(motor_on_speed_L);
  Robot.motors(motor_on_speed_L,motor_on_speed_R);
}
