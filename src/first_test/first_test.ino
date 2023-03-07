#include <Robot_L298P.h> 


#define Kp 0.17//0.2
#define Kd 35//50 //10
#define Ki 0 //0.02
#define Kip 0.95
#define MAX_E 110
#define RAZGON_dt 4

#define MAX_R_SPEED 100
#define MAX_L_SPEED 95 //90

#define K_R_SPEED 1
#define K_L_SPEED 0.95

#define parrot_cm 157
#define parrot_angle 25

void setup() {
  Serial.begin (9600); 
  Robot.setup(); 
  Robot.reverse_motor_A();
  Robot.reverse_enc_B();
  /*
  Robot.reverse_motor_A();
  Robot.reverse_motor_B();
  Robot.reverse_enc_A();
  Robot.reverse_enc_B();
  Robot.motor_A(100);
  Robot.motor_B(-100);
  Robot.motors(1,2);
  Serial.println(Robot.enc_A);
  Serial.println(Robot.enc_B);
  */
  //run_enc(2000,2000);
  
  /*for (int i = 0; i<4; i++) {
    enc_forward(30);
    enc_left(90);
  }*/
}

void loop() {
  enc_forward(100);
  enc_forward(-100);
}

void enc_forward(int cm) {
  run_enc(parrot_cm*cm,parrot_cm*cm);
}

void enc_left(int angle) {
  run_enc(-parrot_angle*angle,parrot_angle*angle);
}

void run_enc(long int L, long int R) {
  Robot.enc_A = 0;
  Robot.enc_B = 0;
  long int eR = 0, eR_old = 0;
  long int eL = 0, eL_old = 0;
  unsigned long int t = millis(), t_razgon = millis();
  long int Ir = 0,Pr,Dr,PIDr;
  long int Il = 0,Pl,Dl,PIDl;
  float razgon = 0.01;
  while (millis() - t < 1000) {
    if ( abs (eL) > MAX_E || abs (eR) > MAX_E) t = millis ();
    
    //Serial.print(Robot.enc_B); Serial.print(" "); Serial.println(Robot.enc_A);
    
    eR = R - Robot.enc_B;
    Pr = eR;
    Ir = eR + Ir*Kip;
    Dr = eR - eR_old;
    eR_old = eR;
    PIDr = Pr*Kp + Ir*Ki + Dr*Kd;
    PIDr *= razgon;
    PIDr = constrain(PIDr,-MAX_R_SPEED,MAX_R_SPEED)*K_R_SPEED;

    eL = L - Robot.enc_A;
    Pl = eL;
    Il = eL + Il*Kip;
    Dl = eL - eL_old;
    eL_old = eL;
    PIDl = Pl*Kp + Il*Ki + Dl*Kd;
    PIDl *= razgon;
    PIDl = constrain(PIDl,-MAX_L_SPEED,MAX_L_SPEED)*K_L_SPEED;
    
    Robot.motors(PIDl, PIDr);

    if (millis() > t_razgon + RAZGON_dt && razgon<1.0) {
      t_razgon = millis();
      razgon += 0.01;
    }
  } 
  Robot.motors(0,0);
  Serial.print(Robot.enc_B); Serial.print(" "); Serial.println(Robot.enc_A);
}
