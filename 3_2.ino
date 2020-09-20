/*
  :Version 1.0
  :Author: Denis Shustov
  :Email: denis.shustov.1984@gmail.com
  :License: BSD
  :Date: 18/09/2020
  :Src version: https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/
*/

#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
 #include <std_msgs/ByteMultiArray.h>
 
#define SONAR_NUM 3          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long _timerStart = 0;

int LOOPING = 40; //Loop for every 40 milliseconds.

uint8_t oldSensorReading[3];    //Store last valid value of the sensors.

std_msgs::Float32MultiArray msg;
ros::Publisher pub_sonar1("rangeSonar1", &msg);
 

NewPing sonar[SONAR_NUM] = {
  NewPing(3, 2, MAX_DISTANCE), //L Trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE), //C
  NewPing(7, 6, MAX_DISTANCE)  //R
};


ros::NodeHandle nh;


void messageCb( const std_msgs::ByteMultiArray& toggle_msg)
{
  if(toggle_msg.data[0] == 1){
    digitalWrite(8, HIGH);   
  else
    digitalWrite(8, LOW);   

  if(toggle_msg.data[1] == 1)
    digitalWrite(9, HIGH);   
  else
    digitalWrite(9, LOW);   
   
  if(toggle_msg.data[2] == 1)
    digitalWrite(10, HIGH);   
  else
    digitalWrite(10, LOW);   
}

ros::Subscriber<std_msgs::ByteMultiArray> sub("funAndBrushes", &messageCb );

//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}

// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//Return the last valid value from the sensor.
void oneSensorCycle() {
    msg.data[0]  =returnLastValidRead(0, cm[0]);
    msg.data[1] = returnLastValidRead(1, cm[1]);
    msg.data[2] = returnLastValidRead(2, cm[2]);
}

//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  
  nh.initNode();
  nh.advertise(pub_sonar1);

  nh.subscribe(sub);
  
  msg.data = (float*)malloc(sizeof(float) * 3);
  msg.data_length = 3;    
}

void loop() {
  if ((millis() - _timerStart) > LOOPING) {
    sensorCycle();
    oneSensorCycle();

    pub_sonar1.publish(&msg);     
    _timerStart = millis();
  }
  nh.spinOnce();
}


/*
rostopic pub /funAndBrushes std_msgs/ByteMultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 0
  data_offset: 0
data:
- 0
- 1
- 0" 
*/
