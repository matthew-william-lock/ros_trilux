/*
BSD 3-Clause License

Copyright (c) 2023, Matthew Lock

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TRILUX_NODE_H
#define TRILUX_NODE_H

#include <trilux/trilux_serial.h>
#include <trilux/trilux_serial.h>

#include <ros/ros.h>
#include <ros_trilux_msgs/Measurement.h>
#include <ros_trilux_msgs/EnableAnalog.h>

namespace trilux
{
   /*!
      * @brief The TriLuxNode class

      * This class is the main class for the TriLux node. It is responsible for
      * initialising the node, setting up the serial port and the ROS interface.
      *
      * When the trilux flourometer is connected and powered on, the default setting (unless changed)
      * is to continuously provide measurements at a rate of 1Hz.
      *
      * @author Matthew Lock
      */
   class TriLuxNode
   {
   public:
      /*!
       * @brief Constructor
       * This constructor initialises the node and sets up the serial port.
       */
      TriLuxNode() : trilux_serial(
                         "/dev/ttyUSB0",
                         9600,
                         // Lambda function to handle data received from the TriLux
                         [this](const trilux::TriLuxMeasurement &measurement)
                         {
                            this->onDataCallback(measurement);
                         })
      {
         ROS_INFO("TriLux node started");
         init();
      };

      ~TriLuxNode(){};

   private:
      trilux::TriLuxSerial trilux_serial;

      ros::ServiceServer enable_analog_output_service;
      ros::ServiceServer enable_reporting_service;
      ros::ServiceServer enable_single_shot_service;
      ros::ServiceServer trigger_measurement_service;
      ros::ServiceServer reboot_service;
      ros::ServiceServer save_setup_service;
      ros::ServiceServer set_rate_service;
      ros::ServiceServer start_continuous_measurement_service;

      ros::Publisher measurement_publisher;

      /*!
       * @brief Initialise the node
       */
      void init()
      {
         // Initialise the node
         ros::NodeHandle nh, ph("~");

         // ROS subscriber and service servers
         this->enable_analog_output_service = nh.advertiseService("enable_analog_output", &TriLuxNode::enableAnalogOutput, this);
         // this->enable_reporting_service = nh.advertiseService("enable_reporting", &TriLuxNode::enableReporting, this);
         // this->enable_single_shot_service = nh.advertiseService("enable_single_shot", &TriLuxNode::enableSingleShot, this);
         // this->trigger_measurement_service = nh.advertiseService("trigger_measurement", &TriLuxNode::triggerMeasurement, this);
         // this->reboot_service = nh.advertiseService("reboot", &TriLuxNode::reboot, this);
         // this->save_setup_service = nh.advertiseService("save_setup", &TriLuxNode::saveSetup, this);
         // this->set_rate_service = nh.advertiseService("set_rate", &TriLuxNode::setRate, this);
         // this->start_continuous_measurement_service = nh.advertiseService("start_continuous_measurement", &TriLuxNode::startContinuousMeasurement, this);

         // ROS publisher
         this->measurement_publisher = nh.advertise<ros_trilux_msgs::Measurement>("measurement", 1);
      }

      // enableAnalogOutput
      bool enableAnalogOutput(ros_trilux_msgs::EnableAnalog::Request &req, ros_trilux_msgs::EnableAnalog::Response &res)
      {
         ROS_INFO("enableAnalogOutput");
         return true;
      }

      const void onDataCallback(const trilux::TriLuxMeasurement &measurement)
      {
         ROS_INFO("onDataCallback");
      }
   };
}

#endif // TRILUX_NODE_H