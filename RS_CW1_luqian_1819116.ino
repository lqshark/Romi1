#include "motors.h"
#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"

#define LOOP_DELAY 100
#define pi 3.14

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
int speed_max = 100;
int state;
double last_count_e0 = 0;
double last_count_e1 = 0;
double distance_x;
double distance_y;
double theta_now;
double theta_demand;
double count_per_wheel_demand;
double distance_x_record = 0;
double distance_y_record = 0;
double theta_cal = 0;

float sensor_left;
float sensor_mid;
float sensor_right;

Motor leftMotor; //left motor
Motor rightMotor;

Line_Sensor line_sensor_left(20);
Line_Sensor line_sensor_mid(21);
Line_Sensor line_sensor_right(22);

float target_speed = 30; //Target encoder count/time_delta
float output_left = 0;
float output_right = 0;
float count_e0_record = 0;
float count_e1_record = 0;

#define BAUD_RATE = 9600;

void setupMotorPins()
{
  // Set our motor driver pins as outputs.
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite(L_DIR_PIN, LOW);
  digitalWrite(R_DIR_PIN, LOW);
}

// put your setup code here, to run once:
void setup()
{

  //Assign motor pins and set direction

  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.
  pinMode(6, OUTPUT);
  setupTimer3();

  setupEncoder0();
  setupEncoder1();
  state = 0;
  last_count_e0 = 0;
  last_count_e1 = 0;
  distance_x = 0;
  distance_y = 0;
  theta_now = 0;
  theta_demand = 0;

  leftMotor.setMaxSpeed(speed_max);
  leftMotor.setPins(L_PWM_PIN, L_DIR_PIN);
  leftMotor.setDebug(false);
  rightMotor.setMaxSpeed(speed_max);
  rightMotor.setPins(R_PWM_PIN, R_DIR_PIN);
  rightMotor.setDebug(false);

  delay(1000);
  play_tone(2, 1000);
  line_sensor_left.calibrate();
  line_sensor_mid.calibrate();
  line_sensor_right.calibrate();
  delay(1000);
  play_tone(2, 1000);
  delay(2000);

  // Initialise the Serial communication
  // so that we can inspect the values of
  // our encoder using the Monitor.
  Serial.begin(9600);

  //You may want to uncomment these lines to help with the tuning process
  //leftPose.set_show_response(true); // This will print the ratio of demand to measurement. When the system is at the target, this will be 1.
  //leftPose.setDebug(true); //  This will print lots of information  - best seen via the Serial monitor
}

void loop()
{

  sensor_left = line_sensor_left.read_calibrated();
  sensor_mid = line_sensor_mid.read_calibrated();
  sensor_right = line_sensor_right.read_calibrated();

  //  Serial.print(distance_x);
  //  Serial.print(",");
  //  Serial.print(distance_y);
  //  Serial.print(",");
  Serial.print(theta_now);
  Serial.print(",");
  Serial.println(theta_demand);

  switch (state)
  {

  case 0:
  {
    StraightLine();
    float sum_sensor = sensor_left + sensor_mid + sensor_right;
    if (sum_sensor > (800 - line_sensor_left.average - line_sensor_mid.average - line_sensor_right.average))
    {
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
      delay(1000);
      play_tone(2, 1000);
      state += 1;
    }
  }
  break;
  case 1:
  {

    linefollowing();
    if (sensor_mid < (500 - line_sensor_mid.average) && sensor_left < (400 - line_sensor_left.average) && sensor_right < (400 - line_sensor_right.average))
    {
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
      delay(1000);
      play_tone(2, 1000);
      theta_cal = atan(distance_y / distance_x);

      count_e0_record = count_e0;
      count_e1_record = count_e1;
      state += 1;
    }
  }
  break;
  case 2:
  {
    double coeff = 1.015;
    theta_demand = coeff * (pi + theta_cal);
    //  count_per_wheel_demand = (theta_error*coeff)/(2*pi)*2*pi*7.5*65.5/2;
    turn_direction();
    //      count_error = count_e0-count_e0_record-count_per_wheel_demand;
    double theta_error = theta_demand - theta_now;
    if (theta_error < (pi / 18000))
    {
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
      delay(1000);
      play_tone(2, 1000);
      count_e0_record = count_e0;
      count_e1_record = count_e1;
      distance_x_record = distance_x;
      distance_y_record = distance_y;
      state += 1;
    }
  }
  break;
  case 3:
  {
    back_StraightLine();

    if (distance_x < 75)
    {
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
      delay(1000);
      play_tone(2, 1000);
      state += 1;
    }
  }
  break;
  case 4:
  {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
    delay(1000000);
    play_tone(2, 1000);
  }
  break;
  }
}
void setupTimer3()
{

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0; // set entire TCCR3A register to 0
  TCCR3B = 0; // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS32);

  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  OCR3A = 625;

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}
void play_tone(int volume, int duration)
{
  analogWrite(6, volume);
  delay(duration);
  analogWrite(6, 0);
  delay(duration);
}
void StraightLine()
{
  float p_straightline = 0.5;
  float count_avr = (count_e0 + count_e1) / 2;
  float output_left = target_speed + p_straightline * (count_avr - count_e0);
  float output_right = target_speed + p_straightline * (count_avr - count_e1);
  leftMotor.setSpeed(output_left);
  rightMotor.setSpeed(output_right);

  delay(100);
}
void linefollowing()
{

  float p_sensor = 0.35; // 0.35 is ok

  float sensor_mean = (sensor_left + sensor_right) / 2;
  float sensor_left_diff = p_sensor * (sensor_mean - sensor_left);
  float sensor_right_diff = p_sensor * (sensor_mean - sensor_right);

  output_left = target_speed + sensor_left_diff - 5;
  output_right = target_speed + sensor_right_diff - 5;
  leftMotor.setSpeed(output_left);
  rightMotor.setSpeed(output_right);

  //Serial.print(output_left);
  //Serial.print(",");
  //  Serial.println(output_right);
  delay(10);
}
//double location_now()
//{
//  theta_now = (count_e1-count_e0)/2/8*pi/180;
//
//  distance_x += ((count_e0 - last_count_e0)+(count_e1 - last_count_e1))/2*cos(theta_now);
//  distance_y += ((count_e0 - last_count_e0)+(count_e1 - last_count_e1))/2*sin(theta_now);
//
//
//  last_count_e0 = count_e0;
//  last_count_e1 = count_e1;
////  Serial.print(distance_x);
////  Serial.print(",");
////  Serial.println(distance_y);
//  delay(10);
//
//}
ISR(TIMER3_COMPA_vect)
{

  theta_now = (count_e1 - count_e0) / (2 * 65.5 * 7.3);
  distance_x += ((count_e0 - last_count_e0) + (count_e1 - last_count_e1)) / 2 * cos(theta_now);
  distance_y += ((count_e0 - last_count_e0) + (count_e1 - last_count_e1)) / 2 * sin(theta_now);

  last_count_e0 = count_e0;
  last_count_e1 = count_e1;
}
void turn_direction()
{

  output_left = -target_speed;
  output_right = target_speed;
  leftMotor.setSpeed(output_left);
  rightMotor.setSpeed(output_right);
}
void back_StraightLine()
{
  float p_back_straightline = 3;
  float output_left = target_speed - p_back_straightline / pi * 180 * (theta_demand - theta_now);
  float output_right = target_speed + p_back_straightline / pi * 180 * (theta_demand - theta_now);
  //  float count_avr = (count_e0+count_e1-count_e0_record-count_e1_record)/2;
  //    float output_left = target_speed + p_back_straightline*(count_avr-(count_e0-count_e0_record));
  //  float output_right = target_speed + p_back_straightline*(count_avr-(count_e1-count_e1_record));
  leftMotor.setSpeed(output_left);
  rightMotor.setSpeed(output_right);

  delay(100);
}
