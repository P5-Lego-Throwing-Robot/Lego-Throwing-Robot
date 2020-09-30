
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 8
#define GRIPPER0 13
#define GRIPPER1 12

ros::NodeHandle node_handle;

std_msgs::UInt16 button_msg;
//std_msgs::UInt16 led_msg;

int current_state0 = 1;
int current_state1 = 0;


int button_state = 0;
int previous_button_state = 1;

long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

void subscriberCallback(const std_msgs::UInt16& button_msg) {

  if (button_msg.data  == 1 ) {
    if (current_state0 == 1) {
      current_state0 = 0;
      current_state1 = 1;
    }
    else {
      current_state0 = 1;
      current_state1 = 0;

    }

  }
  digitalWrite(GRIPPER0, current_state0);
  digitalWrite(GRIPPER1, current_state1);

}


ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt16> open_gripper_subscriber("button_press", &subscriberCallback);

void setup()
{
  pinMode(GRIPPER0, OUTPUT);
  pinMode(GRIPPER1, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  node_handle.initNode();
  node_handle.advertise(button_publisher);
  node_handle.subscribe(open_gripper_subscriber);
}

void loop()
{

  if (digitalRead(BUTTON) == LOW  && previous_button_state == 1 && millis() - time > debounce) {
    button_msg.data = 1;
    previous_button_state = 0;
    button_publisher.publish( &button_msg );
    
  } 
  else if (digitalRead (BUTTON) == HIGH) previous_button_state = 1;
  
  node_handle.spinOnce();

  delay(100);
}
