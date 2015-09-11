#include <iostream>
#include <cstdlib>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

// SOLUTION - DO NOT COPY
bool is_prime(unsigned int n){
	if(n == 1) return false;
	static std::vector<unsigned int> prev;
	unsigned int start = prev.size() > 0 ? prev.back() : 2;
	for(unsigned int i = start; i < n; ++i){
		bool divides_by_any = false;
		for(std::vector<unsigned int>::iterator it = prev.begin(); it != prev.end(); ++it){
			if(i % *it == 0){
				divides_by_any = true;
				break;
			}
		}
		if(!divides_by_any){
			prev.push_back(i);
		}
	}
	for(std::vector<unsigned int>::iterator it = prev.begin(); it != prev.end(); ++it){
		if(n % *it == 0){
			return false;
		}
	}
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "cpp");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::String>("primes", 1000);
	for(int i = 1; i < 101; ++i){
		std::ostringstream out;
		if(is_prime(i)){
			out << "prime";
		}else{
			bool fizzbuzz = false;
			if(i % 3 == 0){
				fizzbuzz = true;
				out << "fizz";
			}
			if(i % 5 == 0){
				fizzbuzz = true;
				out << "buzz";
			}
			if(!fizzbuzz){
				out << i;
			}
		}
		std_msgs::String msg;
		msg.data = out.str();
		publisher.publish(msg);
		ROS_INFO("%s\n", out.str().c_str());
	}
	ros::spin();
	return EXIT_SUCCESS;
}

