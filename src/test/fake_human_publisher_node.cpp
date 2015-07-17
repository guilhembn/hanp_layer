/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Fri Jun 12 2015
 */

#include <ros/ros.h>
#include <hanp_msgs/TrackedHumans.h>
#include <visualization_msgs/Marker.h>

#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_human_publisher");

    ros::NodeHandle n;

    ros::Publisher humans_pub = n.advertise<hanp_msgs::TrackedHumans>("human_tracker", 1);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "human_marker", 0 );

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
        hanp_msgs::TrackedHuman human;
        visualization_msgs::Marker human_marker;

        human.track_id = 1;
        human.pose.pose.position.x = 1.5;
        human.pose.pose.position.y = position + 2.5;
        human.pose.pose.position.z = 0.0;
        human.pose.pose.orientation.x = 0.0;
        human.pose.pose.orientation.y = 0.0;
        human.pose.pose.orientation.z = 0.0;
        human.pose.pose.orientation.w = 1.0;

        hanp_msgs::TrackedHumans humans;
        humans.header.stamp = ros::Time::now();
        humans.header.frame_id = "humans_frame";
        humans.tracks.push_back(human);

        // add marker for the fake human for visualization_msgs
        human_marker.header.frame_id = humans.header.frame_id;
        human_marker.header.stamp = humans.header.stamp;
        human_marker.id = 0;
        human_marker.type = visualization_msgs::Marker::ARROW;
        human_marker.action = visualization_msgs::Marker::MODIFY;
        human_marker.pose.position.x = human.pose.pose.position.x;
        human_marker.pose.position.y = human.pose.pose.position.y;
        human_marker.pose.position.z = human.pose.pose.position.z;
        human_marker.pose.orientation.x = human.pose.pose.orientation.x;
        human_marker.pose.orientation.y = human.pose.pose.orientation.y;
        human_marker.pose.orientation.z = human.pose.pose.orientation.z;
        human_marker.pose.orientation.w = human.pose.pose.orientation.w;
        human_marker.scale.x = 0.5;
        human_marker.scale.y = 0.08;
        human_marker.scale.z = 0.08;
        human_marker.color.a = 1.0;
        human_marker.color.r = 0.5;
        human_marker.color.g = 0.5;
        human_marker.color.b = 0.0;

        humans_pub.publish(humans);
        vis_pub.publish(human_marker);

        ros::spinOnce();

        i++;
        loop_rate.sleep();
    }

    return 0;
}
