/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN  -1
#define DXL_IDRF 1
#define DXL_IDRB 2
#define DXL_IDLF 3
#define DXL_IDLB 4
#define MAX_PWM 885
#define MAX_VEL 61.0

const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int32_t DXL_L_VEL;
int32_t DXL_R_VEL;
 
ros::NodeHandle  nh;
geometry_msgs::Twist data_msg;
ros::Publisher data_pub("/data_motor", &data_msg);

void toggle_led_Callback( const std_msgs::Empty& msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN)); 
  /*
  dxl.setGoalPWM(DXL_IDLF, digitalRead(LED_BUILTIN)*100);//885
  dxl.setGoalPWM(DXL_IDLB, digitalRead(LED_BUILTIN)*100);
  dxl.setGoalPWM(DXL_IDRF, -digitalRead(LED_BUILTIN)*100);
  dxl.setGoalPWM(DXL_IDRB, -digitalRead(LED_BUILTIN)*100);
  */
  if (digitalRead(LED_BUILTIN) == 1){
    dxl.ledOn(DXL_IDLF);
    dxl.ledOn(DXL_IDLB);
    dxl.ledOn(DXL_IDRF);
    dxl.ledOn(DXL_IDRB);
  }
  else{
    dxl.ledOff(DXL_IDLF);
    dxl.ledOff(DXL_IDLB);
    dxl.ledOff(DXL_IDRF);
    dxl.ledOff(DXL_IDRB);
  }
}

void cmd_vel_Callback( const geometry_msgs::Twist& msg){
  float VL = msg.linear.x*MAX_PWM - msg.angular.z*MAX_PWM;
  float VR = msg.linear.x*MAX_PWM + msg.angular.z*MAX_PWM;
  if(VL > MAX_PWM){
    VL=MAX_PWM;
  }
  else if(VL < -MAX_PWM){
    VL=-MAX_PWM;
  }
  if(VR > MAX_PWM){
    VR=MAX_PWM;
  }
  else if(VR < -MAX_PWM){
    VR=-MAX_PWM;
  }
  DXL_L_VEL = VL;
  DXL_R_VEL = -VR;
}

void vel_motor_Callback( const geometry_msgs::Twist& msg){
  float VL = msg.linear.x;
  float VR = msg.linear.y;
  if(VL > MAX_VEL){
    VL=MAX_VEL;
  }
  else if(VL < -MAX_VEL){
    VL=-MAX_VEL;
  }
  if(VR > MAX_VEL){
    VR=MAX_VEL;
  }
  else if(VR < -MAX_VEL){
    VR=-MAX_VEL;
  }

  DXL_L_VEL = VL;
  DXL_R_VEL = VR;
}

ros::Subscriber<std_msgs::Empty> toggle_led_sub("toggle_led", &toggle_led_Callback );
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/turtle1/cmd_vel", &cmd_vel_Callback );//cmd_vel
ros::Subscriber<geometry_msgs::Twist> vel_motor_sub("/vel_motor", &vel_motor_Callback );


void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(toggle_led_sub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(vel_motor_sub);
  nh.advertise(data_pub);
  
  dxl.begin(1000000);
  //dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.torqueOff(DXL_IDRF);
  //dxl.setOperatingMode(DXL_IDRF,  OP_PWM);
  dxl.setOperatingMode(DXL_IDRF,  OP_VELOCITY);
  dxl.torqueOn(DXL_IDRF);
  dxl.torqueOff(DXL_IDRB);
  //dxl.setOperatingMode(DXL_IDRB,  OP_PWM);
  dxl.setOperatingMode(DXL_IDRB,  OP_VELOCITY);
  dxl.torqueOn(DXL_IDRB);
  dxl.torqueOff(DXL_IDLF);
  //dxl.setOperatingMode(DXL_IDLF,  OP_PWM);
  dxl.setOperatingMode(DXL_IDLF,  OP_VELOCITY);
  dxl.torqueOn(DXL_IDLF);
  dxl.torqueOff(DXL_IDLB);
  //dxl.setOperatingMode(DXL_IDLB,  OP_PWM);
  dxl.setOperatingMode(DXL_IDLB,  OP_VELOCITY);
  dxl.torqueOn(DXL_IDLB);
}

void loop()
{  
  nh.spinOnce();
  
  dxl.setGoalVelocity(DXL_IDLF, DXL_L_VEL, UNIT_RPM);
  dxl.setGoalVelocity(DXL_IDLB, DXL_L_VEL, UNIT_RPM);
  dxl.setGoalVelocity(DXL_IDRF, -DXL_R_VEL, UNIT_RPM);
  dxl.setGoalVelocity(DXL_IDRB, -DXL_R_VEL, UNIT_RPM);
  
  data_msg.linear.x = (dxl.getPresentVelocity(DXL_IDLF, UNIT_RPM)+dxl.getPresentVelocity(DXL_IDLB, UNIT_RPM))/2; //Left speed
  data_msg.linear.y = ((dxl.readControlTableItem(PRESENT_LOAD, DXL_IDLF)+dxl.readControlTableItem(PRESENT_LOAD, DXL_IDLB)))*1.4/2; //Left current
  data_msg.linear.z = (dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_IDLF)+dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_IDLB))/2; //Left temp
  data_msg.angular.x = (dxl.getPresentVelocity(DXL_IDRF, UNIT_RPM)+dxl.getPresentVelocity(DXL_IDRB, UNIT_RPM))/2;
  data_msg.angular.y = ((dxl.readControlTableItem(PRESENT_LOAD, DXL_IDRF)+dxl.readControlTableItem(PRESENT_LOAD, DXL_IDRB)))*1.4/2; //Rigth current
  data_msg.angular.z = (dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_IDRF)+dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_IDRB))/2; //Rigth temp
  data_pub.publish( &data_msg );
  
  delay(20);
}
