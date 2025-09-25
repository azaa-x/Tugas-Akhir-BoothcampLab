#include <ros.h>
#include <std_msgs/Int32.h> 

// Pin untuk LED hijau dan merah
int GREEN_LED = 2; 
int RED_LED = 3;

// Inisialisasi ROS NodeHandle
ros::NodeHandle nh;

void massageCb(const std_msgs::Int32 &msg)
{
  if (msg.data == 1)
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
  else if (msg.data == 0)
  {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
}

// Subscriber untuk topik "led_status"
ros::Subscriber<std_msgs::Int32> sub("led_status", massageCb);

void setup()
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  nh.initNode();
  nh.subscribe(sub);
  nh.getHardware()->setBaud(57600);

}

void loop()
{
  nh.spinOnce();
  delay(1); 
}   
