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

// EXAMPLE SHOWING HOW TO ENABLE AND TRIGGER SINGLE SHOT MEASUREMENTS
// The trilux node should be running when this example is run

#include <ros/ros.h>

#include <ros_trilux_msgs/EnableSingleShot.h>
#include <ros_trilux_msgs/GetMeasurement.h>

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "trilux_single_shot");
    ROS_INFO("Starting trilux_single_shot");

    // Create a node handle
    ros::NodeHandle nh;

    ros::ServiceClient enable_single_shot_client = nh.serviceClient<ros_trilux_msgs::EnableSingleShot>("enable_single_shot");
    ros::ServiceClient get_measurement_client = nh.serviceClient<ros_trilux_msgs::GetMeasurement>("trigger_measurement");

    ros_trilux_msgs::EnableSingleShot srv;
    srv.request.enable = true;

    // Call the service
    if (enable_single_shot_client.call(srv))
    {
        ROS_INFO("Single shot measurements enabled");
    }
    else
    {
        ROS_ERROR("Failed to enable single shot measurements");
        return 1;
    }

    ros::Rate loop_rate(0.2);
    while (ros::ok())
    {
        
        ros_trilux_msgs::GetMeasurement srv;
        if (get_measurement_client.call(srv))
        {
            ROS_INFO("Measurement: %f", srv.response.measurement);
        }
        else
        {
            ROS_ERROR("Failed to get measurement");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}