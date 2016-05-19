#include <iostream>
#include <sstream>
#include <functional>
#include <boost/asio.hpp>

class SerialReader{
public:
  const char MSG_TERMINATOR = '\n';
  
  SerialReader(std::string name):
    serial_port(io_service, name),
    input(&read_buffer)
  {}
  void read(std::function<void(std::istream&)> callback){
    boost::asio::read_until(serial_port, read_buffer, MSG_TERMINATOR);
    callback(input);
  }
  void write(std::string data){
    boost::asio::write(serial_port, boost::asio::buffer(data));
  }
private:
  boost::asio::io_service io_service;
  std::istream input;
  boost::asio::streambuf read_buffer;
  boost::asio::serial_port serial_port;
};

// e.g
/*
void get_tokens(std::istream& in){
  int val1;
  in >> val1;
}
*/
/*
void test(){
  SerialReader serial("/dev/tty0");
  serial.write("I");
  serial.read([](std::istream& in){
    std::string msg;
    std::getline(in, msg);
    std::cout << msg << "\n";
  });
  serial.write("D");
  serial.read([](std::istream& in){
    std::string msg;
    std::getline(in, msg);
    std::cout << msg << "\n";
    get_tokens(std::istringstream(msg));
  });
}i*/

