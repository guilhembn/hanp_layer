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

#include<human_aware_layers/safety_layer.h>

// declare the SafetyLayer as a Polygon class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(human_aware_layers::SafetyLayer, costmap_2d::Layer)

namespace human_aware_layers
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
        nh.param("humans_topic", humans_topic, std::string("/humans"));
        humans_sub = nh.subscribe(humans_topic, 1, &SafetyLayer::humansUpdate, this);

        // create satic grids according to human posture, standing, sitting, walking
        // to calcuate cost use a sigmoid function

        // for safety
        ////sigmoid function up to radius
        ////distance is distance to point from human, radis is where we want to end the function
        //sigmoid = cos(distance / radius * M_PI_2) + 0;
        //quot = 1 / (0.6 + distance );
        //val = pow(height * (sigmoid * quot), 3);

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<human_aware_layers::SafetyLayerConfig>(nh);
        dynamic_reconfigure::Server<human_aware_layers::SafetyLayerConfig>::CallbackType cb =
            boost::bind(&SafetyLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    // method for updating internal map f this layer
    void SafetyLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if(!enabled_)
        {
            // check if the human is inside the bouds
            // if inside put cost around human position by applyting static_girds around human positions

            return;
        }
    }

    // method for applyting changes in this layer to global costmap
    void SafetyLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        if(!enabled_)
        {
            // todo: implement
            return;
        }
    }

    void SafetyLayer::humansUpdate(const human_aware_layers_msgs::TrackedHumansPtr& humans)
    {
        // store humans to local variable
    }

    // configure the SafetyLayer parameters
    void SafetyLayer::reconfigureCB(human_aware_layers::SafetyLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    // creates safety grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* SafetyLayer::createSafetyGrid(double safetyMax, double resolution)
    {
        if (safetyMax < 0 || resolution < 0)
        {
            ROS_ERROR_NAMED("safety_layer", "safetyMax or resolution of safety grid cannot be negative");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)((safetyMax / resolution) + 0.05), size_y = size_x;

        // populate array using safety function
        //  r = pi/safetyMax
        //  safety(x, y) =
        //      (cos(r * x) + 1)(cos(r * y) +1) / 4     if dist(x,y) <= safetyMax
        //      0                                       if dist(x,y) >  safetyMax

        // create a grid where human is at center, and #cells both sides = size
        auto safetyGrid = new unsigned char[(2 * size_x + 1) * (2 * size_y + 1)];

        // multiply r by resolution to get proper values for cells in sub-meter grid
        double r = M_PI * resolution / safetyMax;

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
                if (sqrt((x * x) + (y * y)) <= (safetyMax / resolution))
                {
                    safetyGrid[index_1 + (index_2 * y) + x] =
                        (unsigned char)(((cos(r * x) +1) * (cos(r * y) +1) * 255 / 4) + 0.5);
                    // multiply by 255 to get full value range from 0-255
                    // addint 0.5 at the end because converstain of u_int trancates
                }
                else
                {
                    safetyGrid[index_1 + (index_2 * y) + x] = (unsigned char)0;
                }
            }
        }

        return safetyGrid;
    }
}
