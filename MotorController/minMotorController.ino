/*
 * Description: Side-by-side robotic wheelchair project. Using to control motor speed.
 * Date: started on 25 Sept 2015
 * Vinh Nguyen and Tony Kuo project
 * Mega 2560
 */
 
#include <stdlib.h>
#include <ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;


//motor pins
int m1_dir_pin = 3;
int m1_pwm_pin = 12;
int m2_dir_pin = 8;
int m2_pwm_pin = 7;
int led = 13;

/* Global variable */
//motor parameter, 0 - forward, 1 - backward, 2 - not moving.
int m1_dir_new = 2;
int m2_dir_new = 2;
float m1_pwm_new = 0;
float m2_pwm_new = 0;
//previous motor parameter (or current pwm)
int m1_dir = 0;
int m2_dir = 0;
float m1_pwm = 0;
float m2_pwm = 0;

// Fixed increment for motor 
int step = 40;//quick experiment 5 Nov 2015, on faster response in stopping when turning

// For timing in Arduino
unsigned long now;
//20 hz
unsigned const long dur = 50; //20 hz = 0.05 second = 50 ms

// For flashing LED, used unsigned int to keep integer positve, and overflow to 0 from 65536 (max)
unsigned int count = 0;

void messageCb2(const std_msgs::String& cmd);
ros::Subscriber<std_msgs::String> sub("motor_control", &messageCb2 );

void setup()
{ 
  pinMode(m1_dir_pin, OUTPUT);
  pinMode(m1_pwm_pin, OUTPUT);
  pinMode(m2_dir_pin, OUTPUT);
  pinMode(m2_pwm_pin, OUTPUT);
  pinMode(led,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  // Rate is 20 hz
  nh.spinOnce();
  changeMotorDirectionsAndSpeeds();
  count = count + 1;
  
  if (count %10000 == 0)            // flash every 4 count
    digitalWrite(led,HIGH);
  else if (count % 10000 == 5000)   // if count%20 == 1, willturn the LED off too fast, it's not visible enough
    digitalWrite(led,LOW);
    //  delay(1);
}

/* messageCb2 
 * probably the one that receive PWM command
 */
void messageCb2(const std_msgs::String& cmd)
{
  
  String receivedCmd = cmd.data;
  //received string should be in the following format
  //d1Xp1XXXd2Xp2XXX OR d1Xl1XXXl2Xp2XXX  
  // 0: forward, 1: backward
  // p1 and p2 = 0 ~ 255: for PWM
  //l1 and l2 = 0~ 999 for length (distance)

  // Extracting info from the received command
  String dir1 = receivedCmd.substring(2,3);
  String cmd_type = receivedCmd.substring(3,4);
  String cmd1 = receivedCmd.substring(5,8);     //pwm or distance
  //String pwm1 = receivedCmd.substring(5,8);
  String dir2 = receivedCmd.substring(10,11);
  String cmd2 = receivedCmd.substring(13,16);
  //String pwm2 = receivedCmd.substring(13,16);
  float pwm1 = 0;
  float pwm2 = 0;
  float dis1 = 0;
  float dis2 = 0;
  
  char d1[2];
  dir1.toCharArray(d1,2);
  m1_dir_new = atoi(d1);

  char d2[2];
  dir2.toCharArray(d2,2);
  m2_dir_new = atoi(d2);
  
  if ((m1_dir_new != 0)&&(m1_dir_new != 1)) {
    //Invalid motor direction!
    return;
  }
  else if ((m2_dir_new != 0)&& (m2_dir_new != 1)) {
    //Invalid motor direction!
    return;
  }
  

  // Convert pwm from string to int.
  char p1[4];
  float cmd1_number;
  cmd1.toCharArray(p1,4);
  cmd1_number = atoi(p1);
  float cmd2_number;
  cmd2.toCharArray(p1,4);
  cmd2_number = atoi(p1);
  
  if (cmd_type == "p"){
    //speed command
    pwm1 = cmd1_number;
    pwm2 = cmd2_number;
    if (pwm1 > 255) pwm1 = 255;
    if (pwm1 < 0 ) pwm1 = 0;
    if (pwm2 > 255) pwm2 = 255;
    if (pwm2 < 0 ) pwm2 = 0;
      
    //apply changes to motors 
    //copy to global variables
    m1_pwm_new = pwm1;
    m2_pwm_new = pwm2;
    //angeMotorDirectionsAndSpeeds();
  }
}

void setPWMPin()
{
    // ASSUMING m1 and m2 directions are already set to the right values
    // Capping target PWM values, within 0 - 255
    if (m1_pwm_new < 0) m1_pwm_new = 0;
    if (m2_pwm_new < 0) m2_pwm_new = 0;
    if (m1_pwm_new > 255) m1_pwm_new = 255;
    if (m2_pwm_new > 255) m2_pwm_new = 255;
    
    int m1_diff = m1_pwm_new - m1_pwm;
    int m2_diff = m2_pwm_new - m2_pwm;
    // IF no change, don't do anything
    // ADJUSTING new pwms to reach the target PWM values
    // PSEUDO CODE: // this is more for command from joystick.
    // IF diff > step, 
    //  don't adjust in one go. 
    //  adjust step by step
    if (m1_diff > step)
        m1_pwm_new = m1_pwm + step;
    else if (m1_diff < (-step))
        m1_pwm_new = m1_pwm - step;
    
    if (m2_diff > step)
        m2_pwm_new = m2_pwm + step;
    else if (m2_diff < (-step))
        m2_pwm_new = m2_pwm - step;
    
    // ELSE just adjust the difference.
    analogWrite(m1_pwm_pin,m1_pwm_new);
    analogWrite(m2_pwm_pin,m2_pwm_new);
    m1_pwm = m1_pwm_new;
    m2_pwm = m2_pwm_new;
}
void setDirectionPin()
{
    if (m1_dir_new == 1){
        digitalWrite(m1_dir_pin, HIGH);
    }else if (m1_dir_new == 0){
        digitalWrite(m1_dir_pin, LOW);
    }
    //write the m2_dir_pinection. no change
    if (m2_dir_new == 1){
        digitalWrite(m2_dir_pin, HIGH);
    }else if (m2_dir_new == 0){
        digitalWrite(m2_dir_pin, LOW);
    }
    m1_dir = m1_dir_new;
    m2_dir = m2_dir_new;
}
void changeMotorDirectionsAndSpeeds()
{
  /* The logic of this method: 
   * If the direction of the new command change wheel's direction
   *    modify the pwm command to gracefully change direction
   * Otherwise, 
   *    execute the command 
   */
    // If m1 and m2 are going to move in the same direction
    if ( (m1_dir == m1_dir_new) && (m2_dir == m2_dir_new))
    {
      setPWMPin();    
      // apply change to achieve the target using m1 & m2 new_pwm
    }

    //if both motors change direction, stop both motors first
    //both motors do not change the direction at this s
    else if((m1_dir_new != m1_dir) && (m2_dir_new != m2_dir)){
        // IF motors are stationary
        if ( (m1_pwm == 0) && (m2_pwm == 0) )
        {
            // Change wheel directions using m1 & m2 new_dir
            setDirectionPin();
            // apply change to achieve the target using m1 & m2 new_pwm
            setPWMPin();
        }
        // ELSE Stop the motors in the current direction first.
        else
        {
            // Change direction back
            m1_dir_new = m1_dir;
            m2_dir_new = m2_dir;
            // Try to reduce pwm to zero, adjust target pwm for m1 & m2 
            m1_pwm_new = m1_pwm - step;
            m2_pwm_new = m2_pwm - step;
            setPWMPin();
        }
    }
  //if only m1 changes direction, stop m1 first
  //both motors do not change the direction at this s
  else if(m1_dir_new != m1_dir){
        // IF motors are stationary
        if ( m1_pwm == 0 )
        {
            // Change wheel directions using m1 & m2 new_dir
            setDirectionPin();
            // apply change to achieve the target using m1 & m2 new_pwm
            setPWMPin();
        }
        // ELSE Stop the motors in the current direction first.
        else
        {
            // Change direction back
            m1_dir_new = m1_dir;
            // Try to reduce pwm to zero, adjust target pwm for m1 
            m1_pwm_new = m1_pwm - step;
            setPWMPin();
        }
  }
  //if only m2 changes direction, stop m2 first
  //both motors do not change the direction at this s
  else if(m2_dir_new != m2_dir){
        // IF motors are stationary
        if ( m2_pwm == 0 )
        {
            // Change wheel directions using m1 & m2 new_dir
            setDirectionPin();
            setPWMPin();
        }
        // ELSE Stop the motors in the current direction first.
        else
        {
            // Change direction back
            m2_dir_new = m2_dir;
            // Try to reduce pwm to zero, adjust target pwm for m2
            m2_pwm_new = m2_pwm - step;
            setPWMPin();
        }
  }
}
