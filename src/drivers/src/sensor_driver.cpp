/*
* Driver Node for arduino sensors: (Accelerometer, Gyroscope, Compass, GPS)
* Author: Vincent Yuan
* Hardware: Arduino Uno, ADXL335 
* Specs(ADXL335): www.sparkfun.com/datasheets/Components/SMD/adxl335.pdf
*/

#include "sensor_driver.hpp"

int main (int argc, char** argv){
	ros::init(argc,argv, ROS_NODE_NAME);
	ros::NodeHandle nh; 
	ros::Rate loop_rate(ROS_LOOP_RATE);
  ros::Publisher sensor_imu_publisher = nh.advertise<sensor_msgs::Imu>(SENSOR_OUTPUT_TOPIC,20);
  //Attempts at opening the Serial Port
  unsigned int count = 0;
  while( !link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))&& count < 9){
    count++;
  }
  cout << endl;
  if (link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))){
    cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl;
    }
  else{ 
    cout << "[1]Check Permissions" << endl << "[2]Check USB" << endl;
    return 1;
      }
  
  //Temporary Function for Setting covariance values 
  for (int i = 0; i < 9; i++){
  IMU.linear_acceleration_covariance[i] = 0; 
  IMU.angular_velocity_covariance[i]=0;
  IMU.orientation_covariance[i]=0;
  }
  
  int x = 0;
  while(ros::ok()){
    //ROS Loop - All procedures repeated are done here
    char buff1[32];
    char buff2[32];
    data_request('A');//request dat
    loop_rate.sleep(); // Wait for messages to arrive 
    link_port.readData(32,buff1); // Read messsages
    data_request('G');
    loop_rate.sleep();
    link_port.readData(32,buff2);
    if(Serial_Store(buff2,3) && Serial_Store(buff1,0))
        sensor_imu_publisher.publish(IMU);
          //if the messages read are good, publish them, else do nothing 
  }
  
  ROS_ERROR("Sensor Input Node Terminated"); 
}

std::string to_string(int i){
  ostringstream out;
  out << i;
  return out.str();
}

void IMU_write(int c, double val){
  //Takes in value val and writes to IMU.msg depending on situation 
   switch (c){
  case 1: 
    val *= ARDUINO_ANALOG_PU; 
    //conversion from PU (1 to 1024 int) to voltage 
    val /= ADXL_V_G_PU; //conversion from voltage to gs 
    val *= G_TO_ACCEL;
    //conversion from g to m/s^2 as specified in std_msgs/IMU.h 
    IMU.linear_acceleration.x = val;
    break;
  case 2:
    val *= ARDUINO_ANALOG_PU;
    val /= ADXL_V_G_PU; 
    val *= G_TO_ACCEL; 
    IMU.linear_acceleration.y = val;
    break; 
  case 3: 
    val *= ARDUINO_ANALOG_PU;
    val /= ADXL_V_G_PU; 
    val *= G_TO_ACCEL;
    IMU.linear_acceleration.z = val;
    break;
  case 4: 
    IMU.angular_velocity.x = val; 
    break;
  case 5:  
    IMU.angular_velocity.y = val;
    break; 
  case 6:  
    IMU.angular_velocity.z = val; 
    break; 
  }
}

bool Serial_Store(char *buffer,int sensor){
  //Parses the buffer characters and stores them into the respective IMU position 
  //When strtok is passed a NULL pointer, it checks the previous succeful truncation location
  //sensor = 0 for accelerometer, sensor = 3 for gyro, sensor = 3 undetermined
//  if (sensor > 1) return false;
  char *c;  //temp char that stores the char to convert to integer
  c = strtok(buffer,",:");
  while (c != NULL){
    if (c[0] == 'X'){ //Looks for linear_accel_x 
      c = strtok(NULL,",:");
      IMU_write((1+sensor),atoi(c));     
    }
    else if (c[0] == 'Y'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      IMU_write((2+sensor),atoi(c));
    }
    else if (c[0] == 'Z'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      IMU_write((3+sensor),atoi(c));
      return true;
    }
   
      c = strtok(NULL,",:");
  }
  return true;
}

void data_request(char c){
  //clears input buffer and sends confirmation byte to prepare acceptance of message in serial
  link_port.clearBuffer();
  stringstream ss; 
  ss << c; 
  link_port.writeData(ss.str(),1); 
}


