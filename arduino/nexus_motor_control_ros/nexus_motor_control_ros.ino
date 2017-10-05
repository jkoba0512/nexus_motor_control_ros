//#define SERIAL_DEBUG

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

// constant and variables for control cycle
const int T_S = 10000;        // sampling period for control (usec)
long int startTime, endTime;  // start and end time of control process

// count of encoder pulse
int angleRate_CountPerTs1 = 0;  // count of encoder pulse per sampling period
int angleRate_CountPerTs2 = 0;  // use them for control algorithm
volatile int encCount_Instant1 = 0;  // instant count of encoder pulse
volatile int encCount_Instant2 = 0;  // count up and down them at interrupt

// variables for motor commands
// positive for CW (255 for 12V)
// negative for CCW (-255 for -12V)
int motorCommand1 = 0;
int motorCommand2 = 0;

// desired angle rate (rad/s)
float dAngleRate_RadPerS1 = 0.0;
float dAngleRate_RadPerS2 = 0.0;

// parameters from ROS parameter server
float Kp_v_1, Ki_v_1, Kp_p_1;      // PI gains for velocity loop and position loop
float Kp_v_2, Ki_v_2, Kp_p_2;
float GEAR_RATIO_1, GEAR_RATIO_2;  // gear ratio
float RES_ENC_1, RES_ENC_2;        // resolution of encoder

// initialize ROS
ros::NodeHandle nh;

// publisher for encoder count
std_msgs::Float32 enc_count;
ros::Publisher enc_count_pub("encorder_count", &enc_count);

// publisher for motor command
std_msgs::Int16 motor_command;
ros::Publisher motor_command_pub("motor_command", &motor_command);

// subscriber for desired angle rate
void messageCb(const std_msgs::Float32& _dAngleRate_RadPerS1) {
  dAngleRate_RadPerS1 = _dAngleRate_RadPerS1.data;
}
ros::Subscriber<std_msgs::Float32> desired_angle_rate_sub("desired_angle_rate_radpers", &messageCb);

// setup
void setup() {
  // initialize for motor drive (PWM freq. and pin mode)
  initMotorDrive();

  // initilize ROS
  nh.initNode();
  nh.advertise(enc_count_pub);
  nh.advertise(motor_command_pub);
  nh.subscribe(desired_angle_rate_sub);
  
  while (!nh.connected()) {
    nh.spinOnce();
    delay(10);
  }
  
  // get parameters
  if (!nh.getParam("/nexus_motor_control/gains_1/Kp_v", &Kp_v_1, 1)) Kp_v_1 = 0.0;  // PI gains
  if (!nh.getParam("/nexus_motor_control/gains_1/Ki_v", &Ki_v_1, 1)) Ki_v_1 = 0.0;
  if (!nh.getParam("/nexus_motor_control/gains_1/Kp_p", &Kp_p_1, 1)) Kp_p_1 = 0.0;
  if (!nh.getParam("/nexus_motor_control/gains_2/Kp_v", &Kp_v_2, 1)) Kp_v_2 = 0.0;
  if (!nh.getParam("/nexus_motor_control/gains_2/Ki_v", &Ki_v_2, 1)) Ki_v_2 = 0.0;
  if (!nh.getParam("/nexus_motor_control/gains_1/Kp_p", &Kp_p_1, 1)) Kp_p_2 = 0.0;
  if (!nh.getParam("/nexus_motor_control/gear_ratio_1", &GEAR_RATIO_1, 1)) GEAR_RATIO_1 = 64.0;  // gear ratio
  if (!nh.getParam("/nexus_motor_control/gear_ratio_2", &GEAR_RATIO_2, 1)) GEAR_RATIO_2 = 64.0;
  if (!nh.getParam("/nexus_motor_control/res_enc_1", &RES_ENC_1, 1)) RES_ENC_1 = 12.0;  // resulution of encoder
  if (!nh.getParam("/nexus_motor_control/res_enc_2", &RES_ENC_2, 1)) RES_ENC_2 = 12.0;

  // initialize serial com for debug
#if defined(SERIAL_DEBUG)
  Serial.begin(115200);
#endif  // SERIAL_DEBUG

}

// loop
void loop() {
  startTime = micros();  // save the start time of control process

  // update and clear the counts of encoder pulse
  angleRate_CountPerTs1 = encCount_Instant1;  // update
  angleRate_CountPerTs2 = encCount_Instant2;
  encCount_Instant1 = 0;  // clear
  encCount_Instant2 = 0;

  // update the voltages for motors at regular intervals (INPORTANT)
  setMotorCommand1(limitMotorCommand(motorCommand1));
  setMotorCommand2(limitMotorCommand(motorCommand2));

#if defined(SERIAL_DEBUG)
  modeDirectMotorCommandInput(motorCommand1, motorCommand2);
#endif  // SERIAL_DEBUG

#if !defined(SERIAL_DEBUG)
  calPIControl(motorCommand1, motorCommand2, dAngleRate_RadPerS1, dAngleRate_RadPerS2);
#endif  // SERIAL_DEBUG

  enc_count.data = (float)angleRate_CountPerTs1;
  enc_count_pub.publish(&enc_count);
  motor_command.data = motorCommand1;
  motor_command_pub.publish(&motor_command);
  
  nh.spinOnce();
  
  endTime = micros();  // save the end time of control process
  delayMicroseconds(max(0, startTime + T_S - endTime));  // keep control cycle T_S usec
}


