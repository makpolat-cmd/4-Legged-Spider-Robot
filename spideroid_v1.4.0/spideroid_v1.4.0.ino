#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_TERMINAL_MODULE
#include <DabbleESP32.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include<Wire.h>

#include "SCServo.h"
SCServo motor;
Adafruit_MPU6050 mpu;
//

#define RXD2 16
#define TXD2 17
int mot[12] = {1 , 11, 24, 8, 18, 28, 5, 15, 25, 3, 13, 23};//motor id's
int pos[12] = {0 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float motor_angle [14][12] = { //angles for 12 motors from 15 patterns. Taken from MATLAB.
  {0.785, 0.216, -0.580, -0.785, 0.216, -0.580, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  {0.785, 1.526, -1.303, -0.785, 0.216, -0.580, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  { -0.032, 0.861, -0.432, -0.785, 0.216, -0.580, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  { -0.032, -0.065, 0.226, -0.785, 0.216, -0.580, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, -0.032, -0.065, 0.226, -0.785, 0.216, -0.580},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, -0.032, 0.861, -0.432, -0.785, 0.216, -0.580},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, 0.785, 1.526, -1.303, -0.785, 0.216, -0.580},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, 0.785, 0.216, -0.580, -0.785, 0.216, -0.580},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, 0.785, 0.216, -0.580, -0.785, 1.526, -1.303},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, 0.785, 0.216, -0.580, 0.032, 0.861, -0.432},
  {0.295, 0.184, -0.404, -0.295, 0.184, -0.404, 0.785, 0.216, -0.580, 0.032, -0.065, 0.226},
  {0.785, 0.216, -0.580, 0.032, -0.065, 0.226, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  {0.785, 0.216, -0.580, 0.032, 0.593, -0.309, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
  {0.785, 0.216, -0.580, -0.785, 1.050, -1.128, 0.295, 0.184, -0.404, -0.295, 0.184, -0.404},
};

float motor_angle_right [28][12] = {//
  {0.785, 0.216, -0.580, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.785, 0.633, -0.854, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
  {0.785, 1.050, -1.128, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.383, 0.850, -0.768, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
  {0.000, 0.650, -0.409, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.000, 0.325, -0.151, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
  {0.000, -0.009, 0.107, -0.785, 0.216, -0.580, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.161, 0.090, -0.159, -0.554, 0.203, -0.503, 0.554, 0.203, -0.503, -0.161, 0.090, -0.160},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, 0.000, -0.009, 0.107},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, 0.000, 0.321, -0.151},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, 0.000, 0.650, -0.409},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, -0.393, 0.850, -0.769},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, -0.785, 1.050, -1.128},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, -0.785, 0.633, -0.854},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.216, -0.580, -0.785, 0.216, -0.580},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 0.633, -0.854, -0.785, 0.216, -0.580},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.785, 1.050, -1.128, -0.785, 0.216, -0.580},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.393, 0.850, -0.769, -0.785, 0.216, -0.580},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.000, 0.650, -0.409, -0.785, 0.216, -0.580},//base
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.000, 0.321, -0.161, -0.785, 0.216, -0.580},//
  {0.322, 0.189, -0.426, -0.322, 0.189, -0.426, 0.000, -0.009, 0.107, -0.785, 0.216, -0.580},//base
  {0.554, 0.203, -0.503, -0.161, 0.099, -0.160, 0.161, 0.090, -0.160, -0.554, 0.203, -0.503},//
  {0.785, 0.216, -0.580, 0.000, -0.009, 0.107, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.785, 0.216, -0.580, 0.000, 0.321, -0.151, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
  {0.785, 0.216, -0.580, 0.000, 0.650, -0.409, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.785, 0.216, -0.580, -0.393, 0.850, -0.769, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
  {0.785, 0.216, -0.580, -0.785, 1.050, -1.128, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//base
  {0.785, 0.216, -0.580, -0.785, 0.633, -0.854, 0.322, 0.189, -0.426, -0.322, 0.189, -0.426},//
};

float dancing [39][12] = {//
  { -0.278, 0.033, 0.014, -0.588, 0.138, -0.864, 0.588, 0.138, -0.864, 0.278, 0.033, 0.014},
  { -0.332, 0.072, -0.079, -0.549, 0.079, -0.904, 0.601, 0.183, -0.808, 0.223, -0.005, 0.098},
  { -0.384, 0.111, -0.178, -0.473, 0.009, -0.927, 0.595, 0.209, -0.737, 0.166, -0.038, 0.169},
  { -0.433, 0.148, -0.281, -0.351, -0.058, -0.937, 0.576, 0.219, -0.653, 0.109, -0.063, 0.222},
  { -0.479, 0.179, -0.385, -0.179, -0.107, -0.938, 0.547, 0.214, -0.561, 0.051, -0.079, 0.255},
  { -0.520, 0.203, -0.487, 0.026, -0.122, -0.938, 0.510, 0.198, -0.462, -0.007, -0.084, 0.264},
  { -0.555, 0.217, -0.584, 0.226, -0.097, -0.938, 0.468, 0.172, -0.359, -0.065, -0.076, 0.249},
  { -0.582, 0.218, -0.675, 0.386, -0.042, -0.935, 0.421, 0.139, -0.255, -0.123, -0.058, 0.211},
  { -0.598, 0.204, -0.756, 0.496, 0.027, -0.923, 0.371, 0.102, -0.153, -0.181, -0.030, 0.153},
  { -0.600, 0.173, -0.823, 0.561, 0.095, -0.896, 0.319, 0.062, -0.055, -0.237, 0.004, 0.078},
  { -0.581, 0.125, -0.876, 0.593, 0.151, -0.852, 0.265, 0.023, 0.036, -0.292, 0.043, -0.008},
  { -0.534, 0.062, -0.911, 0.601, 0.191, -0.791, 0.209, -0.014, 0.117, -0.345, 0.082, -0.103},
  { -0.447, -0.008, -0.931, 0.592, 0.213, -0.717, 0.152, -0.045, 0.184, -0.396, 0.121, -0.204},
  { -0.312, -0.073, -0.938, 0.569, 0.219, -0.631, 0.094, -0.068, 0.232, -0.445, 0.156, -0.307},
  { -0.129, -0.114, -0.938, 0.538, 0.211, -0.536, 0.036, -0.082, 0.259, -0.489, 0.186, -0.411},
  {0.078, -0.119, -0.938, 0.500, 0.192, -0.436, -0.022, -0.083, 0.263, -0.529, 0.207, -0.512},
  {0.271, -0.086, -0.938, 0.456, 0.164, -0.333, -0.080, -0.073, 0.241, -0.562, 0.218, -0.608},
  {0.418, -0.026, -0.934, 0.409, 0.130, -0.230, -0.138, -0.052, 0.198, -0.587, 0.216, -0.696},
  {0.516, 0.045, -0.918, 0.358, 0.092, -0.128, -0.195, -0.022, 0.135, -0.600, 0.198, -0.774},
  {0.572, 0.110, -0.886, 0.305, 0.052, -0.031, -0.251, 0.014, 0.058, -0.597, 0.163, -0.838},
  {0.597, 0.163, -0.838, 0.251, 0.014, 0.058, -0.305, 0.052, -0.031, -0.572, 0.110, -0.886},
  {0.600, 0.198, -0.774, 0.195, -0.022, 0.135, -0.358, 0.092, -0.128, -0.516, 0.045, -0.918},
  {0.587, 0.216, -0.696, 0.138, -0.052, 0.198, -0.409, 0.130, -0.230, -0.418, -0.026, -0.934},
  {0.562, 0.218, -0.608, 0.080, -0.073, 0.241, -0.456, 0.164, -0.333, -0.271, -0.086, -0.938},
  {0.529, 0.207, -0.512, 0.022, -0.083, 0.263, -0.500, 0.192, -0.436, -0.078, -0.119, -0.938},
  {0.489, 0.186, -0.411, -0.036, -0.082, 0.259, -0.538, 0.211, -0.536, 0.129, -0.114, -0.938},
  {0.445, 0.156, -0.307, -0.094, -0.068, 0.232, -0.569, 0.219, -0.631, 0.312, -0.073, -0.938},
  {0.396, 0.121, -0.204, -0.152, -0.045, 0.184, -0.592, 0.213, -0.717, 0.447, -0.008, -0.931},
  {0.345, 0.082, -0.103, -0.209, -0.014, 0.117, -0.601, 0.191, -0.791, 0.534, 0.062, -0.911},
  {0.292, 0.043, -0.008, -0.265, 0.023, 0.036, -0.593, 0.151, -0.852, 0.581, 0.125, -0.876},
  {0.237, 0.004, 0.078, -0.319, 0.062, -0.055, -0.561, 0.095, -0.896, 0.600, 0.173, -0.823},
  {0.181, -0.030, 0.153, -0.371, 0.102, -0.153, -0.496, 0.027, -0.923, 0.598, 0.204, -0.756},
  {0.123, -0.058, 0.211, -0.421, 0.139, -0.255, -0.386, -0.042, -0.935, 0.582, 0.218, -0.675},
  {0.065, -0.076, 0.249, -0.468, 0.172, -0.359, -0.226, -0.097, -0.938, 0.555, 0.217, -0.584},
  {0.007, -0.084, 0.264, -0.510, 0.198, -0.462, -0.026, -0.122, -0.938, 0.520, 0.203, -0.487},
  { -0.051, -0.079, 0.255, -0.547, 0.214, -0.561, 0.179, -0.107, -0.938, 0.479, 0.179, -0.385},
  { -0.109, -0.063, 0.222, -0.576, 0.219, -0.653, 0.351, -0.058, -0.937, 0.433, 0.148, -0.281},
  { -0.166, -0.038, 0.169, -0.595, 0.209, -0.737, 0.473, 0.009, -0.927, 0.384, 0.111, -0.178},
  { -0.223, -0.005, 0.098, -0.601, 0.183, -0.808, 0.549, 0.079, -0.904, 0.332, 0.072, -0.079},
};

int offset_array [12] = {36, -18, 0, 114, 30, 0, 52, 38, 46, 69, -39, 0};
int stance = 0, forward = 0, down = 0, left = 0, right = 0, dance = 0, reading = 0;
//float incoming_Data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char incoming_Data;

void setup() {
  init_MPU6050();

  Serial.begin(115200);     delay(250);  // serial channel 0 is for communication with computer
  Serial2.begin(1000000, SERIAL_8N1, RXD2 , TXD2); delay(250);
  Dabble.begin("Spideroid");// Bluetooth baslatma

  Serial.println("Başlangic.");
  //Scale islemi
  scale_processing();

  //offset ekleme
  offset_processing();
  Serial.println("Offsetler ayarlandi.");

  origin_position();
}

void loop() {
  Dabble.processInput();

  if (GamePad.isCirclePressed()) { //start pozisyonuna almak icin "O" tusuna bas
    origin_position();
    stance = 0;
  }

  if (GamePad.isUpPressed() || (forward != 0)) { // duz ilerlemek icin "up"
    //-------------Duz Ilerleme-----------
    forward = 1;
    turn_forward();
  }
  if (GamePad.isLeftPressed() || (left != 0)) {
    //-------------Sola Donme-------------
    left = 1;
    turn_left();
  }
  if (GamePad.isRightPressed() || (right != 0)) {
    //-------------Saga Donme-------------
    right = 1;
    turn_right();
  }
  if (GamePad.isDownPressed() || (down != 0)) {
    //-------------Geri gitme-------------
    down = 1;
    turn_down();
  }

  if (GamePad.isTrianglePressed() || (dance != 0)) {
    dance = 1;
    lets_dance();
  }

  if (GamePad.isStartPressed()) {
    reading = 1;
    read_simulation();
  }

  if (GamePad.isSquarePressed()) {// torku kapatmak icin "kare" tusuna bas
    motor.EnableTorque(0xfe, 0); delay(50);
  }
  //------taking data from MPU6050------
  gyro();
}



void scale_processing() {
  for (int i = 0; i < 14; i++) {
    for (int j = 0; j < 12; j++) {

      if (j == 0 || j == 3) //duz ilerlemesi icin
        motor_angle[i][j] *= 1.12;

      motor_angle [i][j] /= 3.141593; // Acilar (-pi)-(pi) araligindan (-1) - (1) araligina cekildi.
      motor_angle [i][j] *= 838;      // Motorlarin uygulamadaki araligi ile carpildi ve aralık (-838) - (838) yapildi.
      motor_angle [i][j] += 512;      // Motorların uygulamadaki orta noktaları 512 oldugu icin orta nokta 0'dan 512'ye cekildi.
      Serial.print(motor_angle [i][j]); Serial.print(" ");
    }
    Serial.println();
  }
  for (int i = 0; i < 28; i++) {
    for (int j = 0; j < 12; j++) {
      motor_angle_right [i][j] /= 3.141593; // Acilar (-pi)-(pi) araligindan (-1) - (1) araligina cekildi.
      motor_angle_right [i][j] *= 838;      // Motorlarin uygulamadaki araligi ile carpildi ve aralık (-838) - (838) yapildi.
      motor_angle_right [i][j] += 512;      // Motorların uygulamadaki orta noktaları 512 oldugu icin orta nokta 0'dan 512'ye cekildi.
    }
  }
  for (int i = 0; i < 39; i++) {
    for (int j = 0; j < 12; j++) {
      dancing [i][j] /= 3.141593;
      dancing [i][j] *= 838;
      dancing [i][j] += 512;
    }
  }
}
void offset_processing() {
  for (int i = 0; i < 14; i++) {
    for (int j = 0; j < 12; j++) {
      motor_angle [i][j] += offset_array[j];
    }
  }
  for (int i = 0; i < 28; i++) {
    for (int j = 0; j < 12; j++) {
      motor_angle_right [i][j] += offset_array[j];
    }
  }
  for (int i = 0; i < 39; i++) {
    for (int j = 0; j < 12; j++) {
      dancing [i][j] += offset_array[j];
    }
  }
}
void origin_position() {
  motor.EnableTorque(0xfe, 0); delay(50); // firstly disable all torque of motors
  for (int i = 0; i < 12; i++) {// changing positions to middle point
    motor.WritePos(mot[i], motor_angle [0][i], 150); delay(50);
  }
  Serial.println("Motora yazildi.");
  //-----------------------------------------------------------------
  // enable torque and moving to middle point
  Serial.println("Positions are initiated.");
  for (int i = 0; i < 12; i = i + 3) {//Coxa
    motor.EnableTorque(mot[i], 1); delay(100);
  }
  for (int i = 1; i < 12; i = i + 3) {//Femur
    motor.EnableTorque(mot[i], 1); delay(100);
  }
  for (int i = 2; i < 12; i = i + 3) {//Tibia
    motor.EnableTorque(mot[i], 1); delay(100);
  }
  Serial.println("Tork enable.");
  //-----------------------------------------------------------------
  //taking all positions
  Serial.print("All positions:");
  for (int i = 0; i < 12; i++) {
    pos[i] = motor.ReadPos(mot[i]); //delay(500);
    Serial.print(pos[i]); Serial.print(" ");
  }
  Serial.println("");
  //-----------------------------------------------------------------
}
int ia, hold_door;
float simulation_angles[12];
char hold[4];
void read_simulation() {
  int i, j;

  while (reading) {
    Dabble.processInput();
    i = 0;
    j = 0;
    hold_door = ((int)(Serial.read())) - 48;

    if ( hold_door == 126) {
      while (Serial.available()) {
        incoming_Data = Serial.read();
        ia = (int)incoming_Data - 48;
        if (ia > 100)
          ia -= 128;

        if (ia == -4) {
          hold[i] = '\0';
          i = 0;
          simulation_angles[j] = atoi(hold);
          j++;
        }
        else {
          hold[i] = (char)(ia + 48);
          i++;
        }
        if (j == 12) {
          j = 0;
          //Terminal.println("12 stance");
          for (int k = 0; k < 12; k++) {
            //Terminal.println(simulation_angles[k]);
            simulation_angles[k] /= 127;
            simulation_angles[k] -= 1;

            simulation_angles[k] *= 838;      // Motorlarin uygulamadaki araligi ile carpildi ve aralık (-838) - (838) yapildi.
            simulation_angles[k] += 512;
            simulation_angles[k] += offset_array[k];
            motor.WritePos(mot[k], (int)(simulation_angles[k]), 150);
            //Terminal.println(simulation_angles[k]);
            simulation_angles[k] = 0;
          }
          break;
        }
      }
    }
    else {
      Terminal.print(hold_door);
    }
    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "X" tusuna bas
      reading = 0;
      break;
    }
  }
}

void lets_dance() {
  while (dance) {
    //gyro();
    Serial.println("dance");
    Dabble.processInput();

    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "X" tusuna bas
      dance = 0;
      stance = 0;
      break;
    }
    // moving to all pattern
    for (int i = 0; i < 12; i++) {
      motor.WritePos(mot[i], dancing [stance][i], 150);
    }

    if (GamePad.isLeftPressed()) { // Turn Left
      forward = 1;
      dance = 0;
      stance = 0;
      break;
    }
    if (GamePad.isLeftPressed()) { // Turn Left
      left = 1;
      dance = 0;
      stance = 0;
      break;
    }
    if (GamePad.isRightPressed()) { // Turn Right
      right = 1;
      dance = 0;
      stance = 0;
      break;
    }
    if (GamePad.isDownPressed()) { // Turn Back
      down = 1;
      dance = 0;
      stance = 0;
      break;
    }
    stance++;


    if (stance >= 39)
      stance = 0;

    delay(75); // delay time 100ms altindayken bacaklar temiz bir sekilde adim atamiyor
    //bu sebepten 100 uzerinde tutulmasi daha saglikli
  }
}

void turn_forward() {
  while (forward) {
    Serial.println("forward");
    Dabble.processInput();

    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "X" tusuna bas
      forward = 0;
      stance = 0;
      break;
    }
    // moving to all pattern
    for (int i = 0; i < 12; i++) {
      motor.WritePos(mot[i], motor_angle [stance][i], 150);
    }

    if (GamePad.isLeftPressed()) { // Turn Left
      left = 1;
      forward = 0;
      stance = 0;
      break;
    }
    if (GamePad.isRightPressed()) { // Turn Right
      right = 1;
      forward = 0;
      stance = 0;
      break;
    }
    if (GamePad.isDownPressed()) { // Turn Back
      down = 1;
      forward = 0;
      break;
    }

    stance++;

    if (stance >= 14)
      stance = 0;

    delay(100); // delay time 100ms altindayken bacaklar temiz bir sekilde adim atamiyor
    //bu sebepten 100 uzerinde tutulmasi daha saglikli
  }
}

void turn_down() {
  while (down) {
    //if (stance == 15)
    //gyro();
    Serial.println("down");
    Dabble.processInput();
    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "X" tusuna bas
      down = 0;
      stance = 0;
      break;
    }
    if (GamePad.isUpPressed()) { // Forward
      forward = 1;
      down = 0;
      break;
    }
    if (GamePad.isLeftPressed()) { // Turn Left
      left = 1;
      down = 0;
      stance = 0;
      break;
    }
    if (GamePad.isRightPressed()) { // Turn Left
      right = 1;
      down = 0;
      stance = 0;
      break;
    }
    for (int i = 0; i < 12 ; i++) {
      motor.WritePos(mot[i], motor_angle [stance][i], 150);
    }
    if (stance <= 0)
      stance = 14;

    stance--;

    delay(100);
  }
}

void turn_left() {
  while (left) {
    //if (stance == 15)
    //gyro();
    Serial.println("left");
    Dabble.processInput();
    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "up" tusuna bas
      left = 0;
      stance = 0;
      break;
    }
    if (GamePad.isUpPressed()) { // Forward
      forward = 1;
      left = 0;
      stance = 0;
      break;
    }
    if (GamePad.isRightPressed()) { // Turn Right
      right = 1;
      left = 0;
      break;
    }
    if (GamePad.isDownPressed()) { // Go back
      down = 1;
      left = 0;
      stance = 0;
      break;
    }

    for (int i = 0; i < 12; i++) {
      motor.WritePos(mot[i], motor_angle_right [stance][i], 150);
    }

    if (stance <= 0)
      stance = 28;

    stance--;

    delay(100);
  }
}

void turn_right() {
  while (right) {
    if (stance == 15)
      gyro();
    Serial.println("right");
    Dabble.processInput();
    if (GamePad.isCrossPressed()) {//robotu durdurmak icin "X" tusuna bas
      right = 0;
      stance = 0;
      break;
    }
    if (GamePad.isUpPressed()) { // Forward
      forward = 1;
      right = 0;
      stance = 0;
      break;
    }
    if (GamePad.isLeftPressed()) { // Turn Left
      left = 1;
      right = 0;
      break;
    }
    if (GamePad.isDownPressed()) { // Go Back
      down = 1;
      right = 0;
      stance = 0;
      break;
    }
    for (int i = 0; i < 12; i++) {
      motor.WritePos(mot[i], motor_angle_right [stance][i], 150);
    }
    stance++;

    if (stance >= 28)
      stance = 0;

    delay(100);
  }
}


void init_MPU6050() {
  while (!Serial)
    delay(5); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(5);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
}

void gyro() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Terminal.print(a.acceleration.x);


  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");


  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

}
