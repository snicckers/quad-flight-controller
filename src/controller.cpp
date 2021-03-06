#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino.h>

#pragma region Globals

#define ACTIVATED HIGH
unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;
double throttle = 1020;

//--- Simple Moving Average Globals ------------------------------------------*/
const int sma_samples = 15;
int a_x_readings[sma_samples];
int a_y_readings[sma_samples];
int a_z_readings[sma_samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- MPU Globals ------------------------------------------------------------*/
int16_t sensor_data[7];

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient; // see datasheet
float roll, pitch, yaw;
long g_drift[3];
float q_0 = 1.0f;
float q_1 = 0.0f;
float q_2 = 0.0f;
float q_3 = 0.0f;
float correction_gain = 0.1f;

/*--- Controller Globals -----------------------------------------------------*/
float pid_roll, pid_pitch, pid_yaw;
float pwm_1, pwm_2, pwm_3, pwm_4;
float error_roll, error_pitch, error_yaw;
float error_roll_previous, error_pitch_previous, error_yaw_previous;
float roll_previous, pitch_previous, yaw_previous;

float roll_pid_p = 0, roll_pid_i = 0, roll_pid_d = 0;
float roll_k_p = 1.2f;
float roll_k_i = 0.0f;
float roll_k_d = 0.8f;
int roll_max = 450;

float pitch_pid_p = 0, pitch_pid_i = 0, pitch_pid_d = 0;
float pitch_k_p = roll_k_p;
float pitch_k_i = roll_k_i;
float pitch_k_d = roll_k_d;
int pitch_max = roll_max;

float yaw_pid_p = 0, yaw_pid_i = 0, yaw_pid_d = 0;
float yaw_k_p = 0.05f;
float yaw_k_i = 0.002f;
float yaw_k_d = 0.0f;
int yaw_max = 100;

float desired_roll = 0.0f, desired_pitch = 0.0f, desired_yaw = 0.0f;

/*--- Radio Globals ----------------------------------------------------------*/
// I/O:
const int rec_input_1_pin = 22; // roll
const int rec_input_2_pin = 23; // pitch
const int rec_input_3_pin = 24; // throttle
const int rec_input_4_pin = 25; // yaw

// Radio Reciever
volatile int rec_input_ch_1, rec_input_ch_1_timer, rec_last_ch_1;
volatile int rec_input_ch_2, rec_input_ch_2_timer, rec_last_ch_2;
volatile int rec_input_ch_3, rec_input_ch_3_timer, rec_last_ch_3;
volatile int rec_input_ch_4, rec_input_ch_4_timer, rec_last_ch_4;

/*--- Motors -----------------------------------------------------------------*/
Servo motor_fl, motor_fr, motor_rl, motor_rr; // 1, 2, 3, 4
int motor_fr_out = 2; // front right
int motor_fl_out = 3; // front light
int motor_rr_out = 4; // read right
int motor_rl_out = 5; // read left

#pragma endregion

/*--- Debugging --------------------------------------------------------------*/
void debugging(){
  int mode = 1;

  // Serial Print has a significant impact on performance. Only use it once every n scans.
  if (elapsed_time - last_time_print > 1){
    if(mode == 1){  // Used to test controller tuning
      Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

      Serial.print(" - pwm FR: ");
      Serial.print(pwm_1);

      Serial.print(" - pwm FL: ");
      Serial.print(pwm_2);

      Serial.print(" - pwm RR: ");
      Serial.print(pwm_3);

      Serial.print(" - pwm RL: ");
      Serial.print(pwm_4);

      Serial.print(" - throttle: ");
      Serial.print(throttle);


      Serial.print("\n");
    }

    if(mode == 2){  //  Used to test imu
      Serial.print(pitch);
      Serial.print(" ");
      Serial.print(roll);
      Serial.print("\n");
    }

    if(mode == 3){  //  Used to test imu
      Serial.print((float)sensor_data[0]);
      Serial.print(" ");
      Serial.print((float)sensor_data[1]);
      Serial.print(" ");
      Serial.print((float)sensor_data[2]);
      Serial.print(" ");
      Serial.print((float)sensor_data[3]);
      Serial.print(" ");
      Serial.print((float)sensor_data[4]);
      Serial.print(" ");
      Serial.print((float)sensor_data[5]);
      Serial.print(" ");
      Serial.print((float)sensor_data[6]);
      Serial.print("\n");
    }

    if(mode == 4){
      Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

      Serial.print(" - Pitch: ");
      Serial.print(yaw);

      Serial.print("\n");
    }

    if(mode == 5){
      Serial.print(rec_input_ch_1);
      Serial.print(" ");
      Serial.print(rec_input_ch_2);
      Serial.print(" ");
      Serial.print(rec_input_ch_3);
      Serial.print(" ");
      Serial.print(rec_input_ch_4);
      Serial.print(" ");
      Serial.print(desired_pitch);
      Serial.print(" ");
      Serial.print(desired_roll);
      Serial.print("\n");
    }

    if(mode == 6){
      Serial.print(pid_pitch);
      Serial.print(" ");
      Serial.print(pid_roll);
      Serial.print(" ");
      Serial.print(pid_yaw);
      Serial.print(" ");
      Serial.print(" ");
      Serial.print(" ");
      Serial.print(desired_pitch);
      Serial.print(" ");
      Serial.print(desired_roll);
      Serial.print("\n");
    }

    if (elapsed_time - last_time_print > 20000){
      last_time_print = micros();
    }
  }
}

// I don't have an oscilloscope so here you go:
void debug_loopTime(){
  if (elapsed_time - last_time_print > 100000){
    Serial.print(micros() - elapsed_time);
    Serial.print("\n");
    last_time_print = micros();
  }
}

/*--- SETUP MPU --------------------------------------------------------------*/
void setup_mpu(){
  // Activate the MPU-6050
  // 0x68 = Registry address of mpu6050
  // 0x6B = Send starting register
  // 0x00 = Tell the MPU not to be asleep
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); // Wakes up the MPU6050
  Wire.endTransmission();
  // Configure the accelerometer (+/-8g)
  // 0x1C = Registry address of accelerometer
  // 0x10 = Full scale range of accelerometer (data sheet)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Configure the gyro (500dps full scale
  // 0x1B = Registry address of gyroscope
  //        0x08 = 500 degree / sec Range of the gyro in degree/sec (data sheet)
  //        0x10 = 1000 degree / sec range
  // 0x12 = 2000 degree / sec range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();
}

/*--- READ MPU  --------------------------------------------------------------*/
void read_mpu(){
  /* Access the accellerometer register and requst
  14 bits. Assign each high and low bit to a variable. */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while(Wire.available() < 14){}; // Wait for all of the bits to be recieved:
  // Assign values to each element of the array:
  for (int i = 0; i <= 7; i++) sensor_data[i] = (Wire.read()<<8|Wire.read());

}

/*--- DATA PROCESSING --------------------------------------------------------*/
// Simple moving average filter. This method smoothes out noisey accelerometer data. It isn't too expensive. Be careful when setting the number of sma_samples: A large sma_samples will lead to a large time-delay, too few sma_samples will lead to a negligible smoothing effect.
void accel_data_processing(){  //Simple moving average filter
  a_read_total[0] -= a_x_readings[a_read_index];
  a_read_total[1] -= a_y_readings[a_read_index];
  a_read_total[2] -= a_z_readings[a_read_index];
  a_x_readings[a_read_index] = (sensor_data)[0];
  a_y_readings[a_read_index] = (sensor_data)[1];
  a_z_readings[a_read_index] = (sensor_data)[2];
  a_read_total[0] += a_x_readings[a_read_index];
  a_read_total[1] += a_y_readings[a_read_index];
  a_read_total[2] += a_z_readings[a_read_index];
  a_read_index += 1;
  if (a_read_index >= sma_samples){
    a_read_index = 0;
  }
  a_read_ave[0] = a_read_total[0] / sma_samples;
  a_read_ave[1] = a_read_total[1] / sma_samples;
  a_read_ave[2] = a_read_total[2] / sma_samples;
}

// Remove the average gyroscope drift (recorded in the calibration method) from gyro data, and correct for temperature variance error.
void gyro_data_processing(){
  // Offset drift from gyroscope data:
  (sensor_data)[4] -= g_drift[0];
  (sensor_data)[5] -= g_drift[1];
  (sensor_data)[6] -= g_drift[2];

  // Gyroscope Temperature Offset Calibation
  float mpu_t = ((float)sensor_data[3] + 12421.0f) / 340.0f;
  //float mpu_t = 25.0f;
  float lsb_scale_offset = 0.328 * (mpu_t - 25.0f);
  lsb_coefficient = (1.0f / (32.8f - lsb_scale_offset));
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// Cheapest/fastest inverse square root I could find (99.94% accurate to 1 / sqrt(x))
// Source: http://www.dbfinteractive.com/forum/index.php?topic=6269.0
float invSqrt(float x){
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

// Calculate attitude during runtime.
void calculate_attitude(){
  /*--- Madgwick Filter ------------------------------------------------------*/
  float normalize;

  //Import and normalize accelerometer data
  float a_x = a_read_ave[0];
  float a_y = a_read_ave[1];
  float a_z = a_read_ave[2];
  normalize = invSqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  a_x *= normalize; a_y *= normalize; a_z *= normalize;

  float g_x = sensor_data[4] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_y = sensor_data[5] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_z = sensor_data[6] * (lsb_coefficient) * (1.0) * degrees_to_rad;

  // Rotation matrix q_dot = 0.5 angular velocity rotation maxtrix * q
  // Reference: Inertial Navigation Systems with Geodetic Applications, Christopher Jekeli, Eq. 1.76 - 4x4 Skew Matrix
  float qDot_0 = 0.5f*(-q_1*g_x - q_2*g_y - q_3*g_z);
  float qDot_1 = 0.5f*( q_0*g_x + q_2*g_z - q_3*g_y);
  float qDot_2 = 0.5f*( q_0*g_y + q_2*g_x - q_1*g_z);
  float qDot_3 = 0.5f*( q_0*g_z + q_1*g_y - q_2*g_x);

  /* References:
      1. https://nitinjsanket.github.io/tutorials/attitudeest/madgwick - (primary)
      2. Estimation of IMU and MARG orientation using a gradient descent algorithm (Sebastian O.H. Madgwick, Andrew J.L. Harrison, Ravi Vaidyanathan) - (supplementary) */

  // Setup for gradient descent algorithm: precalculate any values that occur more than once. Doing this saves the processer 30 multiplication operations.
  float q2_0 = q_0 * q_0; //a2
  float q2_1 = q_1 * q_1; //b2
  float q2_2 = q_2 * q_2; //c2
  float q2_3 = q_3 * q_3; //d2

  float _4q_0 = 4.0f * q_0; //4a
  float _4q_1 = 4.0f * q_1; //4b
  float _4q_2 = 4.0f * q_2; //4c
  float _4q_3 = 4.0f * q_3; //4d

  float _2q_0 = 2.0f * q_0; //2a
  float _2q_1 = 2.0f * q_1; //2b
  float _2q_2 = 2.0f * q_2; //2c
  float _2q_3 = 2.0f * q_3; //2d

  float _8q_1 = 8.0f * q_1; //8b
  float _8q_2 = 8.0f * q_2; //8c

  // Gradient Descent algorithm
  float delF_0 = _4q_0 * q2_2 + _4q_0 * q2_1 + _2q_2 * a_x - _2q_1 * a_y;
  float delF_1 = _8q_1*q2_1 + _4q_1*q2_3 + _4q_1*q2_0 - _4q_1 + _8q_1*q2_2 - _2q_3*a_x - _2q_0*a_y + _4q_1*a_z;
  float delF_2 = _8q_2*q2_2 - _4q_2 + _4q_2*q2_3 + _4q_2*q2_0 + _8q_2*q2_1 + _2q_0*a_x - _2q_3*a_y + _4q_2*a_z;
  float delF_3 = _4q_3*q2_2 + _4q_3*q2_1 - _2q_1*a_x - _2q_2*a_y;

  normalize = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 *= normalize; q_1 *= normalize; q_2 *= normalize; q_3 *= normalize;

  //Change correction_gain for more or less influence of accelerometer on gyro rates.
  qDot_0 -= correction_gain * delF_0;
  qDot_1 -= correction_gain * delF_1;
  qDot_2 -= correction_gain * delF_2;
  qDot_3 -= correction_gain * delF_3;
  q_0 += qDot_0 * sample_time;
  q_1 += qDot_1 * sample_time;
  q_2 += qDot_2 * sample_time;
  q_3 += qDot_3 * sample_time;

  normalize = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 *= normalize; q_1 *= normalize; q_2 *= normalize; q_3 *= normalize;

  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees + 0.4f;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees + 1.2f;
  yaw = atan2f(2*(q_0*q_3 + q_1*q_2), 1.0f - 2.0f*(q_2*q_2 + q_3*q_3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* KEEP IMU STATIONARY DURING STARTUP */

  /*--- Simple Moving Average Setup ---*/
  for (int i = 0; i < sma_samples; i++){
    a_x_readings[i] = 0;
    a_y_readings[i] = 0;
    a_z_readings[i] = 0;
  }

  /*--- Calibrate gyroscope data and initial attitude: ---*/
  int cal_count = 500;
  Serial.print("\nCalibrating \n");
  for (int i = 0; i < cal_count; i ++){
    sample_time = (micros() - elapsed_time) / 1000000.0f;
    elapsed_time = micros();

    // Print the loading bar blips n times
    if(i % 50 == 0) { Serial.print("-"); }

    // Collect data from MPU
    read_mpu();

    g_drift[0] += sensor_data[4];
    g_drift[1] += sensor_data[5];
    g_drift[2] += sensor_data[6];

    delay(3);
  }

  // Average drift / offset of the raw gyroscope data:
  g_drift[0] /= cal_count;
  g_drift[1] /= cal_count;
  g_drift[2] /= cal_count;
}

/*--- CONTROLLER -------------------------------------------------------------*/
void flight_controller(){

  //----------------------------------------------------------- PITCH CONTROLLER
  error_pitch = desired_pitch - pitch;

  // Proportional:
  pitch_pid_p = pitch_k_p * error_pitch;

  // Integral:
  int thresh = 8;
  if (error_pitch < thresh && error_pitch > -thresh) {
    pitch_pid_i = pitch_pid_i * (pitch_k_i * error_pitch);
  }
  if (error_pitch > thresh && error_pitch < -thresh){
    pitch_pid_i = 0;
  }

  // Derivative:
  // Take the derivative of the process variable (PITCH) instead of the error
  // Taking derivative of the error results in "Derivative Kick".
  // https://www.youtube.com/watch?v=KErYuh4VDtI
  pitch_pid_d = (-1.0f) * pitch_k_d * ((pitch - pitch_previous) / sample_time);

  // Altogether:
  pid_pitch = pitch_pid_p + pitch_pid_i + pitch_pid_d;

  /* Clamp the maximum & minimum pid values*/
  if (pid_pitch < -pitch_max) pid_pitch = -pitch_max;
  if (pid_pitch > pitch_max) pid_pitch = pitch_max;

  //------------------------------------------------------------ ROLL CONTROLLER
  error_roll = desired_roll - roll;

  // Proportional:
  roll_pid_p = roll_k_p * error_roll;

  // Integral:
  //thresh = 8;
  if (error_roll < thresh && error_roll > -thresh) {
    roll_pid_i = roll_pid_i * (roll_k_i * error_roll);
  }
  if (error_roll > thresh && error_roll < -thresh){
    roll_pid_i = 0;
  }

  // Derivative:
  roll_pid_d = (-1.0f) * roll_k_d * ((roll - roll_previous) / sample_time);

  // Altogether:
  pid_roll = roll_pid_p + roll_pid_i + roll_pid_d;
  //pid_yaw = nicksMap(pid_yaw, 0.0f, 90.0f, -1000, 1000);

  /* Clamp the maximum & minimum pid values*/
  if (pid_roll < -roll_max) pid_roll = -roll_max;
  if (pid_roll > roll_max) pid_roll = roll_max;

  //------------------------------------------------------------- YAW CONTROLLER
  error_yaw = desired_yaw - yaw;

  // Proportional:
  yaw_pid_p = yaw_k_p * error_yaw;

  // Integral:
  //thresh = 8;
  if (error_yaw < thresh && error_yaw > -thresh) {
    yaw_pid_i = yaw_pid_i * (yaw_k_i * error_yaw);
  }
  if (error_yaw > thresh && error_yaw < -thresh){
    yaw_pid_i = 0;
  }

  // Derivative:
  yaw_pid_d = (-1.0f) * yaw_k_d * ((yaw - yaw_previous) / sample_time);

  // Altogether:
  pid_yaw = yaw_pid_p + yaw_pid_i + yaw_pid_d;
  //pid_yaw = nicksMap(pid_yaw, 0.0f, 90.0f, -1000, 1000);

  /* Clamp the maximum & minimum pid values*/
  if (pid_yaw < -yaw_max) pid_yaw = -yaw_max;
  if (pid_yaw > yaw_max) pid_yaw = yaw_max;

  //----------------------------------------------------- MOTOR MIXING ALGORITHM

  pwm_4 = throttle + pid_pitch + pid_roll + pid_yaw; // Rear Left       pwm_4
  pwm_2 = throttle - pid_pitch + pid_roll - pid_yaw; // Front Left      pwm_2
  pwm_3 = throttle + pid_pitch - pid_roll - pid_yaw; // Rear Right      pwm_3
  pwm_1 = throttle - pid_pitch - pid_roll + pid_yaw; // Front Right     pwm_1

  int upper_bound = 1850;
  int lower_bound = 1010;

  // Clamp PWM Values:
  if (pwm_1 >= upper_bound) pwm_1 = upper_bound;
  if (pwm_1 <= lower_bound) pwm_1 = lower_bound;

  if (pwm_2 >= upper_bound) pwm_2 = upper_bound;
  if (pwm_2 <= lower_bound) pwm_2 = lower_bound;

  if (pwm_3 >= upper_bound) pwm_3 = upper_bound;
  if (pwm_3 <= lower_bound) pwm_3 = lower_bound;

  if (pwm_4 >= upper_bound) pwm_4 = upper_bound;
  if (pwm_4 <= lower_bound) pwm_4 = lower_bound;

  //------------------------------------------------------------ WRITE TO MOTORS
  motor_fr.write(pwm_1);  // pwm_1
  motor_fl.write(pwm_2);  // pwm_2
  motor_rr.write(pwm_3);  // pwm_3
  motor_rl.write(pwm_4);  // pwm_4

  //-------------------------------------------------- SET VALUES FOR NEXT CYCLE
  error_pitch_previous = error_pitch;
  pitch_previous = pitch;
  error_roll_previous = error_roll;
  roll_previous = roll;
  error_yaw_previous = error_yaw;
  yaw_previous = yaw;
}

void clamp_deltaMotorOutput(){
  // What am I clamping
  // * radio signal?
  // * pid control signal?
  // * motor output signal? Yes clamp this one
  /* If the change in motor speed output is too high in too short a prediod of time, turn off all motors.
     This is a safety feature only, to prevent you from taking a chunk of flesh out of your arm/foot again.
     
     What am I gonna need?
     * global array to record the last ~20 output samples
     * calculate the average value for the sample set
     * if the average value exceeds a HH setpoint, emergency stop */



}

/*--- ISRs -------------------------------------------------------------------*/
float nicksMap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void radio_reciever_input() {
  // Channel 1: Roll
  if (rec_last_ch_1 == 0 && digitalRead(rec_input_1_pin)){
    rec_last_ch_1 = 1;
    rec_input_ch_1_timer = micros();
  }
  else if (rec_last_ch_1 == 1 && !digitalRead(rec_input_1_pin)){
    rec_last_ch_1 = 0;
    rec_input_ch_1 = micros() - rec_input_ch_1_timer;
  }

  // Channel 2: Pitch
  if (rec_last_ch_2 == 0 && digitalRead(rec_input_2_pin)){
    rec_last_ch_2 = 1;
    rec_input_ch_2_timer = micros();
  }
  else if (rec_last_ch_2 == 1 && !digitalRead(rec_input_2_pin)){
    rec_last_ch_2 = 0;
    rec_input_ch_2 = micros() - rec_input_ch_2_timer;
  }

  // Channel 3: Throttle
  if (rec_last_ch_3 == 0 && digitalRead(rec_input_3_pin)){
    rec_last_ch_3 = 1;
    rec_input_ch_3_timer = micros();
  }
  else if (rec_last_ch_3 == 1 && !digitalRead(rec_input_3_pin)){
    rec_last_ch_3 = 0;
    rec_input_ch_3 = micros() - rec_input_ch_3_timer;
  }

  // Channel 4: Yaw
  if (rec_last_ch_4 == 0 && digitalRead(rec_input_4_pin)){
    rec_last_ch_4 = 1;
    rec_input_ch_4_timer = micros();
  }
  else if (rec_last_ch_4 == 1 && !digitalRead(rec_input_4_pin)){
    rec_last_ch_4 = 0;
    rec_input_ch_4 = micros() - rec_input_ch_4_timer;
  }

  float rotate_range = 15.0f;
  throttle = rec_input_ch_1;
  desired_roll = nicksMap(rec_input_ch_3, 1000, 2000, -rotate_range, rotate_range);
  desired_pitch = nicksMap(rec_input_ch_2, 1000, 2000, -rotate_range, rotate_range);
  // desired_yaw = nicksMap(rec_input_ch_4, 1000, 2000, )
}

void radio_setup(){
  pinMode(rec_input_1_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_1_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_2_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_2_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_3_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_3_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_4_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_4_pin), radio_reciever_input, CHANGE);
}

void setup_motors(){
  motor_fl.attach(motor_fl_out);
  motor_fr.attach(motor_fr_out);
  motor_rl.attach(motor_rl_out);
  motor_rr.attach(motor_rr_out);

  motor_fl.write(1000);
  motor_fr.write(1000);
  motor_rl.write(1000);
  motor_rr.write(1000);
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  //pinMode(7, INPUT);
  Serial.begin(115200);
  Wire.begin();
  setup_motors();
  radio_setup();
  setup_mpu();
  calibrate_imu();
}

/*--- MAIN -------------------------------------------------------------------*/
void loop(){
  sample_time = (micros() - elapsed_time) / 1000000.0f;
  elapsed_time = micros();

  //IMU
  read_mpu();
  gyro_data_processing();
  accel_data_processing();
  calculate_attitude();
  flight_controller();

  // DEBUGGING
  debugging();
  //debug_loopTime();

  // REFRESH RATE
  while (micros() - elapsed_time < 4000){};

  // if (micros() - elapsed_time > 2530){
  //   Serial.print("\n ------- ERROR: SCAN TIME EXCEEDED ------- \n");
  //   while (1);
  // }
}
