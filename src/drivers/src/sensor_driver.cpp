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
  ros::Publisher sensor_imu_publisher = nh.advertise<sensor_msgs::Imu>(IMU_TOPIC, 5);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC,20);
  //Attempts at opening the Serial Port
  /*if(!connect_device("SENS"))
        return 1;//Error signal
  else
        cout << "Connected to Accelerometer, Gyroscope" << endl;

  */
  ros::NodeHandle private_nh("~");
  string port = "/dev/ttyACM0";
  private_nh.param("port", port);
  if(!link_port.connect(BAUD_RATE,port)){
cout << "Unable to connect to a device on " << port << endl 
        << "Did you remember to set the correct port as a param? You should go do that" << endl;
    return 1;
}
  //Temporary Function for Setting covariance values 
  for (int i = 0; i < 9; i++){
  IMU.linear_acceleration_covariance[i] = 0; 
  IMU.angular_velocity_covariance[i]=0;
  IMU.orientation_covariance[i]=0;
  odom.pose.pose.position.z=0.0;
  }
  
  while(ros::ok() && link_port.isActive()){
    //ROS Loop - All procedures repeated are done here
    char buff1[32] = "\0";
    data_request('D',buff1,DATA);
    loop_rate.sleep();
    std::string A_input = to_string2(buff1);
    if(A_input.find('\n')!=std::string::npos && A_input.find('X')!=std::string::npos){  
      //cout << buff1;
      if(Serial_Store(buff1)){ //&& Serial_Store(buff1,0)){
          sensor_imu_publisher.publish(IMU);
          odom_publisher.publish(odom);
        }   
    }
    loop_rate.sleep();
  }
  
  ROS_ERROR("Sensor Input Node Terminated"); 
  return 0; 
}
bool connect_device(std::string device_name){
    int i = 0;
    while (i < 9){
    char buff[32]="\0";
    if(!open_port(i)){
        ROS_ERROR("PORT ERROR");
        return false;
    }
    data_request('I',buff,1);
    std::string s_input;
    do{
     data_request('I',buff,1);
     s_input = to_string2(buff);
    } while (s_input.find('\n')==std::string::npos);
    if(s_input.find(device_name)!=std::string::npos)
        return true;
    else
        i++;
    }
    ROS_ERROR("Connected device is not GPS");
    return false; 
}
bool open_port(unsigned int count){
    while(!link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME + to_string(count)))&& count < 9){count++;} 
    cout << endl;
    if(link_port.connect(BAUD_RATE,(ARDUINO_PORT_NAME+to_string(count)))){cout << "Connected on port" << ARDUINO_PORT_NAME << count << endl;
    return true;} 
    else{cout << "[1]Check Permissions" << endl << "[2]Check USB" <<endl;
    return false;
    }
}

std::string to_string(int i){
  ostringstream out;
  out << i;
  return out.str();
}
std::string to_string2(char *c){
  ostringstream out;
  out << c; 
  return out.str();
}

bool Serial_Store(char *buffer){
  //Parses the buffer characters and stores them into the respective IMU position 
  //When strtok is passed a NULL pointer, it checks the previous succeful truncation location
  //sensor = 0 for accelerometer, sensor = 3 for gyro, sensor = 3 undetermined
//  if (sensor > 1) return false;
  //cout << "buffer: " << buffer;  
  char *c;  //temp char that stores the char to convert to integer
  c = strtok(buffer,", :");
  while (c != NULL){
   // cout << "in buffer: " << buffer << endl;
    //cout << "c : " << c << endl;
    if (c[0] == 'X'){ //Looks for linear_accel_x 
      c = strtok(NULL,",:");
      //cout << "1: " << c << endl;
      msg_store(1,c);     
    }
    else if (c[0] == 'Y'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      //cout << "Y" << endl;
      msg_store(2,c);
    }
    else if (c[0] == 'Z'){ //Looks for linear_accel_y
      c = strtok(NULL,",:");
      msg_store(3,c);
    }
    else if (c[0] == 'A'){
      c = strtok(NULL,",:");
      msg_store(4,c);
      return true;
    }
    else if (c[0] == 'L'){
      c = strtok(NULL,",:");  
      //store msg  
    }
    else if (c[0] == 'R'){
      c = strtok(NULL,",:");    
      //store msgwa
    }
    else
      return false;
   
      c = strtok(NULL,",:");
  }
  return true;
}
void msg_store(int type, char *data){
  double val = atoi(data);
  switch(type){
  case 1: 
    val *= ARDUINO_ANALOG_PU; 
    //conversion from PU (1 to 1024 int) to voltage 
    val /= ADXL_V_G_PU; //conversion from voltage to gs 
    val *= G_TO_ACCEL;
    //conversion from g to m/s^2 as specified in std_msgs/IMU.h 
    IMU.linear_acceleration.x = val;
    //cout << IMU.linear_acceleration << endl;
    break;
  case 2: 
    val *= ARDUINO_ANALOG_PU;
    val /= ADXL_V_G_PU; 
    val *= G_TO_ACCEL; 
    IMU.linear_acceleration.y = val;
  case 3: 
    val *= ARDUINO_ANALOG_PU;
    val /= ADXL_V_G_PU; 
    val *= G_TO_ACCEL;
    IMU.linear_acceleration.z = val;
  case 4: 
    IMU.angular_velocity.z = val;
  case 5: 
    odom.twist.twist.linear.y = atof(data);
  case 6: 
    odom.pose.pose.position.y = atof(data);
  }
}

void data_request(char c, char*buffer, int mode){
  //clears input buffer and sends confirmation byte to prepare acceptance of message in serial
  //link_port.clearBuffer(); 
  stringstream ss; 
  ss << c; 
  int i = 0;
  link_port.writeData(ss.str(),1);
  if (mode == 1){
    while(buffer[0] == '\0'){
      link_port.writeData(ss.str(),1);
      link_port.readData(32,buffer); 
    } 
  }
  else{
    while(buffer[0] == '\0' && i < 20){
      link_port.readData(32,buffer);
    }
  }
  return;
 }


