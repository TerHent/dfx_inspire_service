#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <boost/program_options.hpp>

namespace param
{

namespace po = boost::program_options;

inline std::string serial_port;
inline std::string serial_left_port;
inline std::string network; 
inline std::string ns; 
inline float threhold;
inline bool swap_hands = false;
inline int force = 500;
inline int speed = 500;

inline po::variables_map helper(int argc, char** argv)
{
  po::options_description desc("Unitree H1 Inspire Hand Serial to DDS");
  desc.add_options()
    ("help,h", "produce help message")
    ("serial,s", po::value<std::string>(&serial_port)->default_value("/dev/ttyUSB0"), "right serial port")
    ("serial_left", po::value<std::string>(&serial_left_port)->default_value("/dev/ttyUSB1"), "left serial port")
    ("network", po::value<std::string>(&network)->default_value(""), "DDS network interface")
    ("namespace", po::value<std::string>(&ns)->default_value("inspire"), "DDS topic namespace")
    ("swap", po::bool_switch(&swap_hands)->default_value(false), "swap left and right hands")
    ("force,f", po::value<int>(&force)->default_value(500), "maximum force limit (0-1000)")
    ("speed,v", po::value<int>(&speed)->default_value(500), "maximum speed limit (0-1000)")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if(ns.empty())
  {
    std::cerr << "Error: Namespace cannot be empty" << std::endl;
    exit(1);
  }

  return vm;
}

}

#endif // PARAM_H