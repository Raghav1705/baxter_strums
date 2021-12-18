#include <ros.h>
#include <std_msgs/UInt16.h>

#define IN 4 //to know if ukulele finished transitioning

ros::NodeHandle node_handle;

std_msgs::UInt16 strum_msg;

ros::Publisher strum_publisher("strum_time", &strum_msg);

//volatile int curr = 0;
//volatile int prev = -1;

void setup()
{
  pinMode(IN, INPUT);
  digitalWrite(IN, LOW);
  node_handle.initNode();
  node_handle.advertise(strum_publisher);
  strum_msg.data = 0;
}

void loop()
{ 
//  curr = digitalRead(IN);
  if (digitalRead(IN) == HIGH) {
    strum_msg.data = strum_msg.data + 1;
    strum_publisher.publish( &strum_msg );
//    strum_msg.data = 1;
//    strum_publisher.publish( &strum_msg );
//    strum_msg.data = 0;
//    strum_publisher.publish( &strum_msg );
//    delay(3000);
//  }
//  if (curr == HIGH && curr != prev) {
//    strum_msg.data = 1;
//    strum_publisher.publish( &strum_msg );
  } else {
    strum_msg.data = 0;
    strum_publisher.publish( &strum_msg );
  }
//  prev = curr;

  node_handle.spinOnce();
  delay(100);
}
