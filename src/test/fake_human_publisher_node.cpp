/*
 * Copyright (c) 2014, LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <human_aware_layers_msgs/TrackedHumans.h>

#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_human_publisher");

    ros::NodeHandle n;

    ros::Publisher humans_pub = n.advertise<human_aware_layers_msgs::TrackedHumans>("human_tracker", 1);

    ros::Rate loop_rate(5);

    int i = 0;
    double position = 5.0;
    while (ros::ok())
    {
        // reducing rate of changeing human pose that of 10 times than publishing rate
        if(i % 10 == 0)
        {
            //change position with triangulation wave funciton with period of 4
            position =  abs(((i/10) % 8) - 4) - 2;
        }

        // just publish a human with some position, and without velocities
        human_aware_layers_msgs::TrackedHuman human;

        human.track_id = 1;
        human.pose.pose.position.x = 1.5;
        human.pose.pose.position.y = position + 2.5;
        human.pose.pose.position.z = 0.0;
        human.pose.pose.orientation.x = 0.0;
        human.pose.pose.orientation.y = 0.0;
        human.pose.pose.orientation.z = 0.0;
        human.pose.pose.orientation.w = 1.0;

        human_aware_layers_msgs::TrackedHumans humans;
        humans.header.stamp = ros::Time::now();
        humans.header.frame_id = "humans_frame";
        humans.tracks.push_back(human);

        humans_pub.publish(humans);

        ros::spinOnce();

        i++;
        loop_rate.sleep();
    }

    return 0;
}
