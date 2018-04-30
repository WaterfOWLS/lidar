#include <ros.h>
#include <Wire.h>
#include <Servo.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/time.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

Servo myservo;

int pos = 0;         // Position of the servo (degress, [0, 180])
int distance = 0;    // Distance measured
ros::NodeHandle  nh;
sensor_msgs::LaserScan scan;
ros::Publisher pub_range( "scan", &scan);
float ranges[12];
char frameid[] = "laser_link";
int num_readings = 100;
int laser_frequency = 40;

void setup()
{
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  delay(1000);
  nh.advertise(pub_range);
  scan.header.frame_id = "laser_link";
  scan.header.stamp = nh.now();
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1.0 / laser_frequency) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges_length = 1;
  scan.ranges = ranges;
  //Serial.begin(250000); // Initialize serial connection to display distance readings
  myservo.attach(9);
  Wire.begin(); // join i2c bus
  
}

//uint32_t last_print = 0;
uint16_t Distance[16] = {0};
//long range_time = 0;
//int r =0;'
//long count = 0;
//int32_t i=0;
int k=1;

void loop()
{ Serial.print("\npublishing begin ");
  if(pos==180){
    k=1;}
  else if(pos==0){
    k=0;}
  if(k==0){
    pos++;
  }
  else{
    pos--;
  }
    //Distance[0] = myLidarLite.distance(0);
    distance = lidarGetRange();
    Serial.print("publishing success ");
    serialPrintRange(pos, distance);
    //Distance[0]=distance;
    scan.ranges[0] = (float)distance / 100;
    scan.header.frame_id = "laser_link";
    scan.header.stamp = nh.now();
    scan.angle_increment=pos;
    pub_range.publish(&scan);
    nh.spinOnce();
    myservo.write(pos);
    
    //delay(1);
   
}

int lidarGetRange(void)
{
  int val = -1;
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting
  delay(10); // Wait 20ms for transmit
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting
  delay(10); // Wait 20ms for transmit
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read(); // receive high byte (overwrites previous reading)
    val = val << 8; // shift high byte to be high 8 bits
    val |= Wire.read(); // receive low byte as lower 8 bits
  }
  
  return val;
}

void serialPrintRange(int pos, int distance)
{
    Serial.print("Position (deg): ");
    Serial.print(pos);
    Serial.print("\t\tDistance (cm): ");
    Serial.println(distance);
}



