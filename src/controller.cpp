#include <Wire.h>
#include <stdlib.h>

#define ACTIVATED HIGH
unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;
/*--- Propeller Servos -------------------------------------------------------*/

double throttle = 1100;
int button_state = 0;
int previous_time_pressed;
bool start_motors = false;

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

/*--- PID Globals ------------------------------------------------------------*/
float pid, pwm_right, pwm_left, error, previous_error, previous_pitch;
float pid_p = 0, pid_i = 0, pid_d = 0;
float k_p = 3.78f; //1.4
float k_i = 0.01f; //1.82
float k_d = 0.70f;  //1.04
float desired_angle = 0.0;

// Radio Reciever Pins:
const int rec_input_1_pin = 22;
const int rec_input_2_pin = 23;
const int rec_input_3_pin = 24;
const int rec_input_4_pin = 25;

// Radio Reciever
volatile int rec_input_ch_1, rec_input_ch_1_timer, rec_last_ch_1;
volatile int rec_input_ch_2, rec_input_ch_2_timer, rec_last_ch_2;
volatile int rec_input_ch_3, rec_input_ch_3_timer, rec_last_ch_3;
volatile int rec_input_ch_4, rec_input_ch_4_timer, rec_last_ch_4;

/*--- Debugging --------------------------------------------------------------*/
void debugging(){
  int mode = 5;

  // Serial Print has a significant impact on performance. Only use it once every n scans.
  if (elapsed_time - last_time_print > 20000){
    if(mode == 1){  // Used to test controller tuning
      Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

      Serial.print(" - Pitch: ");
      Serial.print(yaw);

      Serial.print(" - pwm left: ");
      Serial.print(pwm_left);

      Serial.print(" - pwm right: ");
      Serial.print(pwm_right);

      Serial.print(" - PID: ");
      Serial.print(pid);

      Serial.print(" - Run Motors?: ");
      Serial.print(start_motors);

      Serial.print(" - k_p: ");
      Serial.print(k_p);
      Serial.print(" - k_i: ");
      Serial.print(k_i);
      Serial.print(" - k_d: ");
      Serial.print(k_d);

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

// Remove the average gyroscope drift / offset (recorded in the calibration method) from the gyroscope data that is recorded during each scan.
void gyro_data_processing(){
  (sensor_data)[4] -= g_drift[0];
  (sensor_data)[5] -= g_drift[1];
  (sensor_data)[6] -= g_drift[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// Cheapest/fastest inverse square root I could find (99.94% accurate to 1 / sqrt(x))
// Source: http://www.dbfinteractive.com/forum/index.php?topic=6269.0
float invSqrt( float number ){
    union {
        float f;
        uint32_t i;
    } conv;

    float x2;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    conv.f  = number;
    conv.i  = 0x5f3759df - ( conv.i >> 1 );
    conv.f  = conv.f * ( threehalfs - ( x2 * conv.f * conv.f ) );
    return conv.f;
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

  float mpu_t = ((float)sensor_data[3] + 12421.0f) / 340.0f;
  float lsb_scale_offset = 0.328 * (mpu_t - 25.0f);
  lsb_coefficient = (1.0f / (32.8f - lsb_scale_offset));

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

  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees + 0.0f;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees + 2.0f;
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
  error = desired_angle - pitch;

  // PROPORTIONAL COMPONENET
  pid_p = k_p * error;

  // INTEGRAL COMPONENT
  int k_i_thresh = 8;
  if (error < k_i_thresh && error > -k_i_thresh) {
    pid_i = pid_i * (k_i * error);
  }
  if (error > k_i_thresh && error < -k_i_thresh){
    pid_i = 0;
  }

  /* DERIVATIVE COMPONENT*/
  // Take the derivative of the process variable (ROLL) instead of the error
  // Taking derivative of the error results in "Derivative Kick".
  // https://www.youtube.com/watch?v=KErYuh4VDtI
  pid_d = (-1.0f) * k_d * ((pitch - previous_pitch) / sample_time);

  // pid_d = k_d * ((error - previous_error) / sample_time);
  /* Sum the the components to find the total pid value. */
  pid = pid_p + pid_i + pid_d;

  /* Clamp the maximum & minimum pid values*/
  if (pid < -1000) pid = -1000;
  if (pid > 1000) pid = 1000;

  /* Calculate PWM width. */
  pwm_right = throttle - pid;
  pwm_left = throttle + pid;

  /* clamp PWM values. */
  //----------Right---------//
  if (pwm_right < 1000) pwm_right = 1000;
  if (pwm_right > 2000) pwm_right = 2000;

  //----------Left---------//
  if (pwm_left < 1000) pwm_left = 1000;
  if (pwm_left > 2000) pwm_left = 2000;

  if (start_motors == true){
    // MOTORS ON
  } else{
    // MOTORS OFF
  }

  previous_error = error;
  previous_pitch = pitch;
}

// void change_setpoint(){
//   if (receiver_input_channel_1 != 0){
//     desired_angle = map(receiver_input_channel_1, 1000, 2000, -30, 30);
//   }
// }

/*--- ISRs -------------------------------------------------------------------*/
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

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  pinMode(7, INPUT);
  Serial.begin(9600);
  Wire.begin();
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

  // DEBUGGING
  debugging();

  // REFRESH RATE
  int scan_time = micros() - elapsed_time;
  while (scan_time < 3800){};
  // if (scan_time > 3850){
  //   Serial.print("\n ------- ERROR: SCAN TIME EXCEEDED ------- \n");
  //   while (1);
  // }
}
