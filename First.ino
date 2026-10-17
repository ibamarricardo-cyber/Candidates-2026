#include <Adafruit_TCS34725.h>
#include <NewPing.h>
#include <Wire.h>

//motores
#define fr_in_1 22 //derecha
#define fr_in_2 23
#define fl_in_1 26
#define fl_in_2 27
#define rr_in_1 24 //derecha
#define rr_in_2 25
#define rl_in_1 28
#define rl_in_2 29

//motores, velocidad (PWM)
#define hl 6 //puente h left inside
#define hr 7 //puente h right inside
#define hl2 11 //puente h left outside
#define hr2 12 //puente h rignt outside

int speed_hl = 100; 
int speed_hr = 100; 
int speed_turn = 60; 

//HC-SR04
#define TRIG_F 49
#define ECHO_F 48

#define TRIG_R 51
#define ECHO_R 50

#define TRIG_L 53
#define ECHO_L 52

#define MAXD 300 // Distancia máxima en cm

int dis_max = 20; 
int disf, disr, disl;

NewPing up(TRIG_F, ECHO_F, MAXD);
NewPing r(TRIG_R, ECHO_R, MAXD);
NewPing l(TRIG_L, ECHO_L, MAXD);

//TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

//LED RGB (PWM)
#define led_r 8
#define led_g 9
#define led_b 10

int min_light = 50;

//(SDA SCL)
#define mp 0x70 //valor multiplexor

//gyroscope
#define gyro 0x68 //valor gyroscope
#define gyro_ch 1 //canal 0 (SD0/SC0)

float ang_vel_gyro = 0; //velocidad angular
float error_gyro = 0;
float angle = 0; // grados movimiento

int ang_90 = 85; 
int ang_u = 175; 

//times
int t_HC = 30; //tiempo entre ultrasónico
int t_stop = 50;
int t_loop = 50; //tiempo al final del loop
int fer = 800;

void setup(){

  //motores
  pinMode(fr_in_1, OUTPUT);
  pinMode(fr_in_2, OUTPUT);
  pinMode(fl_in_1, OUTPUT);
  pinMode(fl_in_2, OUTPUT);
  pinMode(rr_in_1, OUTPUT);
  pinMode(rr_in_2, OUTPUT);
  pinMode(rl_in_1, OUTPUT);
  pinMode(rl_in_2, OUTPUT);
  pinMode(hl, OUTPUT);
  pinMode(hr, OUTPUT);
  pinMode(hl2, OUTPUT);
  pinMode(hr2, OUTPUT);

  //LED RGB
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);
  pinMode(led_b, OUTPUT);

  stop();

  Wire.begin();

  if(!tcs.begin()){
    while(1);
  }

  channel(gyro_ch);
  Wire.beginTransmission(gyro);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();


  Wire.beginTransmission(gyro);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  delay(3000);

  gyro_error();
}

void loop(){

  colorUpdate();
  distanceSensor();

  bool freeR = (disr > dis_max);
  bool freeF = (disf > dis_max);
  bool freeL = (disl > dis_max);

  if (freeR){
    turnAngle(ang_90, true); //turn right
    delay(t_stop);
  }

  else if (freeF){
    //no gira, sigue derecho
  }

  else if (freeL){
    turnAngle(ang_90, false); //turn left
    delay(t_stop);
  }

  else {
    turnAngle(ang_u, true);
    delay(t_stop);
  }

  forward();
  delay(fer);
  stop();
  delay(t_stop);

  delay(t_loop);
}

void distanceSensor(){

  disf = up.ping_cm();
  delay(t_HC);
  disr = r.ping_cm();
  delay(t_HC);
  disl = l.ping_cm();
  delay(t_HC);

  if (disf == 0){
    disf = MAXD;
  }
  if (disr == 0){
    disr = MAXD;
  }
  if (disl == 0){
    disl = MAXD;
  }

}

void colorUpdate(){
  uint16_t r, g, b, c;
  int red = 0;
  int green = 0;
  int blue = 0;

  tcs.getRawData(&r, &g, &b, &c);

  if (c > min_light){
    red = float(r)*255/c;
    green = float(g)*255/c;
    blue = float(b)*255/c;
  }

  analogWrite(led_r, red);
  analogWrite(led_g, green);
  analogWrite(led_b, blue);
}

void channel(int ch){

  Wire.beginTransmission(mp);
  Wire.write(ch); //numro ed canal
  Wire.endTransmission();

}

void gyroUpdate(){

  channel(gyro_ch); 
  Wire.beginTransmission(gyro);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(gyro, 2);

  int high = Wire.read();
  int low = Wire.read();
  int16_t raw = (high << 8) | low;

  ang_vel_gyro = raw / 65.5 - error_gyro; //convertir a grados por segundo
}

void gyro_error(){
  float total = 0;
  error_gyro = 0;

  for (int i = 0; i < 200; i++){
    gyroUpdate();
    total = total + ang_vel_gyro;
    delay(5);
  }
  error_gyro = total / 200;
}

void turnAngle(int degrees, bool toRight){
  angle = 0;

  if (toRight){
    turnRight();
  }
  else {
    turnLeft();
  }

  while (fabs(angle) < degrees){
    gyroUpdate();
    angle = angle + ang_vel_gyro * 0.01;
    delay(10);
  }

  stop();
}

void movement(bool fr_1, bool fr_2, bool fl_1, bool fl_2, bool rr_1, bool rr_2, bool rl_1, bool rl_2, int speed_left, int speed_right){
  digitalWrite(fr_in_1 , fr_1);
  digitalWrite(fr_in_2 , fr_2);
  digitalWrite(fl_in_1 , fl_1);
  digitalWrite(fl_in_2 , fl_2);
  digitalWrite(rr_in_1 , rr_1);
  digitalWrite(rr_in_2 , rr_2);
  digitalWrite(rl_in_1 , rl_1);
  digitalWrite(rl_in_2 , rl_2);
  analogWrite(hl, speed_left);
  analogWrite(hr, speed_right);
  analogWrite(hl2, speed_left);
  analogWrite(hr2, speed_right);
}

void stop(){
  movement(LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, 0, 0);
}

void forward(){
  movement(HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH, speed_hl, speed_hr);
}

void left(){
  movement(HIGH, LOW, HIGH, LOW, LOW, HIGH, LOW, HIGH, speed_hl, speed_hr);
}

void right(){
  movement(LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW, speed_hl, speed_hr);
}

void backwards(){
  movement(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW, speed_hl, speed_hr);
}

void turnRight(){
  movement(HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, speed_turn, speed_turn);  
}

void turnLeft(){
  movement(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, speed_turn, speed_turn);
}