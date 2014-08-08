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

#include <math.h>

#include<hanp_layer/safety_layer.h>
#include<geometry_msgs/PoseStamped.h>

// declare the SafetyLayer as a Polygon class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hanp_layer::SafetyLayer, costmap_2d::Layer)

namespace hanp_layer
{
    SafetyLayer::SafetyLayer() {}

    // onInitialize is called just after setting tf_, name_, and layered_costmap_
    // layered_costmap_ is the costmap which plugin gets from the map_server
    void SafetyLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        //current_ = true; //TODO: find out use of this variable

        // subscribe to human positions
        std::string humans_topic;
        nh.param("humans_topic", humans_topic, std::string("/human_tracker"));
        humans_sub = nh.subscribe(humans_topic, 1, &SafetyLayer::humansUpdate, this);

        // create satic grids according to human posture, standing, sitting, walking
        // to calcuate cost use a sigmoid function

        // for safety
        // create sigmoid function up to radius
        // distance is distance to point from human, radis is where we want to end the function
        // sigmoid = cos(distance / radius * M_PI_2) + 0;
        // quot = 1 / (0.6 + distance );
        // val = pow(height * (sigmoid * quot), 3);

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<hanp_layer::SafetyLayerConfig>(nh);
        dynamic_reconfigure::Server<hanp_layer::SafetyLayerConfig>::CallbackType cb =
            boost::bind(&SafetyLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // store global_frame name locally
        global_frame_ = layered_costmap_->getGlobalFrameID();
        resolution = layered_costmap_->getCostmap()->getResolution();

        // initialize last bounds
        last_min_x = last_min_y = last_max_x = last_max_y = 0.0;
    }

    // method for updating internal map f this layer
    void SafetyLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
    {
        // clear last data
        lastTransformedHumans.clear();

        // check for enable and availability of human tracking before continuing
        if(!enabled_
           || !lastTrackedHumans
           || ((ros::Time::now() - lastTrackedHumans->header.stamp) > human_tracking_delay)
          )
        {
            return;
        }

        // TODO: use locking for lastTrackedHumans
        // check if frame transformation is still possible
        auto humans_header = lastTrackedHumans->header;

        /*bool transformOK = false;
        try
        {
            transformOK = tf_->waitForTransform(global_frame_, lastTrackedHumans->header.frame_id,
                                      lastTrackedHumans->header.stamp, ros::Duration(0.1));
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR_NAMED("safety_layer", "tf exception: %s", ex.what());
        }

        if(!transformOK)
        {
            ROS_ERROR_NAMED("safety_layer", "can't transform from %s to %s at %f",
                            global_frame_.c_str(), lastTrackedHumans->header.frame_id.c_str(),
                            lastTrackedHumans->header.stamp.toSec());
            return;
        }*/

        double min_x_, min_y_, max_x_, max_y_;  // limits for the costmap update
        // transform each human into costmap frame and calculate bounds for costmap
        //lastTransformedHumans = new geometry_msgs::PoseStamped[lastTrackedHumans->tracks.size()]
        for(auto &human : lastTrackedHumans->tracks)
        {
            // fist transform human positions
            geometry_msgs::PoseStamped in_human, out_human;
            in_human.header = humans_header;
            in_human.header.frame_id = humans_header.frame_id;
            in_human.pose = human.pose.pose;
            try
            {
                tf_->transformPose(global_frame_, in_human, out_human);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR_NAMED("safety_layer", "tf exception while transforming human pose in \\%s frame: %s",
                                global_frame_.c_str(), ex.what());
                lastTransformedHumans.clear();
                return;
            }

            // now calculate costmap bounds
            // note: bouds should be returned in meters, not in grid-points
            auto size_x = safety_max + 0.05, size_y = size_x;

            min_x_ = std::min(*min_x, out_human.pose.position.x - size_x);
            min_y_ = std::min(*min_y, out_human.pose.position.y - size_y);
            max_x_ = std::max(*max_x, out_human.pose.position.x + size_x);
            max_y_ = std::max(*max_y, out_human.pose.position.y + size_y);

            lastTransformedHumans.push_back(out_human);
        }

        // for correct clearing of the map apply last calculated bounds
        *min_x = std::min(min_x_, last_min_x);
        *min_y = std::min(min_y_, last_min_y);
        *max_x = std::max(max_x_, last_max_x);
        *max_y = std::max(max_y_, last_max_y);

        // save current update bounds for future clearance
        last_min_x = min_x_;
        last_min_y = min_y_;
        last_max_x = max_x_;
        last_max_y = max_y_;
    }

    // method for applyting changes in this layer to global costmap
    void SafetyLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        if(!enabled_ || (lastTransformedHumans.size() == 0))
        {
            return;
        }

        // update map according to each human position
        for(auto human : lastTransformedHumans)
        {
            // get position of human in map
            unsigned int cell_x, cell_y;
            if(!master_grid.worldToMap(human.pose.position.x, human.pose.position.y, cell_x, cell_y))
            {
                ROS_ERROR_NAMED("safety_layer", "no world coordinates for human at x:%f y:%f",
                               human.pose.position.x, human.pose.position.y);
                continue;
            }
            //ROS_INFO_NAMED("safety_layer", "human x:%d y:%d", cell_x, cell_y);

            // apply safety grid according to position of each human
            auto size_x = (int)((safety_max / resolution) + resolution), size_y = size_x;

            // by convention x is number of columns, y is number of rows
            for(int y = 0; y <= 2 * size_y; y++)
            {
                for(int x = 0; x <= 2 * size_x; x++)
                {
                    // add to current cost
                    auto cost = (int)master_grid.getCost(x + cell_x - size_x, y + cell_y - size_y)
                        + (int)safety_grid[x + ((2 * size_x + 1) * y)];
                    // detect overflow
                    if(cost > (int)costmap_2d::LETHAL_OBSTACLE)
                    {
                        cost = costmap_2d::LETHAL_OBSTACLE;
                    }


                    master_grid.setCost(x + cell_x - size_x, y + cell_y - size_y, (unsigned int)cost);
                }
            }
        }
    }

    void SafetyLayer::humansUpdate(const hanp_layer_msgs::TrackedHumansPtr& humans)
    {
        // store humans to local variable
        lastTrackedHumans = humans;
    }

    // configure the SafetyLayer parameters
    void SafetyLayer::reconfigureCB(hanp_layer::SafetyLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
        safety_max = config.safety_max;
        human_tracking_delay = ros::Duration(config.human_tracking_delay);

        // re-create safety grid, with changed safety_max and current resolution
        // TODO: what if resolution changes
        resolution = layered_costmap_->getCostmap()->getResolution();
        safety_grid = createSafetyGrid(safety_max, resolution);
    }

    // creates safety grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* SafetyLayer::createSafetyGrid(double safety_max, double resolution)
    {
        if (safety_max < 0 || resolution < 0)
        {
            ROS_ERROR_NAMED("safety_layer", "safety_max or resolution of safety grid cannot be negative");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)((safety_max / resolution) + resolution), size_y = size_x;

        // populate array using safety function
        //  r = pi/safety_max
        //  safety(x, y) =
        //      (cos(r * x) + 1)(cos(r * y) +1) / 4     if dist(x,y) <= safety_max
        //      0                                       if dist(x,y) >  safety_max

        // create a grid where human is at center, and #cells both sides = size
        auto safetyGrid = new unsigned char[(2 * size_x + 1) * (2 * size_y + 1)];

        // multiply r by resolution to get proper values for cells in sub-meter grid
        double r = M_PI * resolution / safety_max;

        // reducing index calculation from
        // (y + size_y) (2 size_x + 1) + (x + size_x)
        // to
        // (2 size_x size_y + size_x + size_y) + (1 + 2 size_y) y + x
        int index_1 = (2 * size_x * size_y) + size_x + size_y;
        int index_2 = 1 + (2 * size_y);

        // by convention x is number of columns, y is number of rows
        for(int y = -size_y; y <= size_y; y++)
        {
            for(int x = -size_x; x <= size_x; x++)
            {
                if (sqrt((x * x) + (y * y)) <= (safety_max / resolution))
                {
                    safetyGrid[index_1 + (index_2 * y) + x] =
                        (unsigned char)(((cos(r * x) +1) * (cos(r * y) +1) * costmap_2d::LETHAL_OBSTACLE / 4) + 0.5);
                    // multiply by LETHAL_OBSTACLE to get full value range from 0-LETHAL_OBSTACLE
                    // addint 0.5 at the end because converstain of u_int trancates
                }
                else
                {
                    safetyGrid[index_1 + (index_2 * y) + x] = (unsigned char)0;
                }
            }
        }
        ROS_DEBUG_NAMED("safety_grid", "created safety_grid of size %d %d", 2 * size_x + 1, 2 * size_y + 1);

        return safetyGrid;
    }
}
