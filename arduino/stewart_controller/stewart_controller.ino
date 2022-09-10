#include <OpenCR_ROS.h>

#include <ros.h>

#include <delta_2/ServoAngles6DoFStamped.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.

ros::NodeHandle nh;

void posCb( const delta_2::ServoAngles6DoFStamped& servo_msg){
  nh.loginfo("Program info");
}

ros::Subscriber<delta_2::ServoAngles6DoFStamped> sub("/servo_setpoint/positions", &posCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
