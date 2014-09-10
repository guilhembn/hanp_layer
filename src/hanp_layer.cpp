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

#include<hanp_layer/hanp_layer.h>
#include<geometry_msgs/PoseStamped.h>

// declare the SafetyLayer as a Polygon class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hanp_layer::HANPLayer, costmap_2d::Layer)

namespace hanp_layer
{
    HANPLayer::HANPLayer() {}

    // onInitialize is called just after setting tf_, name_, and layered_costmap_
    // layered_costmap_ is the costmap which plugin gets from the map_server
    void HANPLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        //current_ = true; //TODO: find out use of this variable

        // subscribe to human positions
        std::string humans_topic;
        nh.param("humans_topic", humans_topic, std::string("/human_tracker"));
        humans_sub = nh.subscribe(humans_topic, 1, &HANPLayer::humansUpdate, this);

        // create satic grids according to human posture, standing, sitting, walking
        // to calcuate cost use a sigmoid function

        // for safety
        // create sigmoid function up to radius
        // distance is distance to point from human, radis is where we want to end the function
        // sigmoid = cos(distance / radius * M_PI_2) + 0;
        // quot = 1 / (0.6 + distance );
        // val = pow(height * (sigmoid * quot), 3);

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<hanp_layer::HANPLayerConfig>(nh);
        dynamic_reconfigure::Server<hanp_layer::HANPLayerConfig>::CallbackType cb =
            boost::bind(&HANPLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // store global_frame name locally
        global_frame_ = layered_costmap_->getGlobalFrameID();
        resolution = layered_costmap_->getCostmap()->getResolution();

        // initialize last bounds
        last_min_x = last_min_y = last_max_x = last_max_y = 0.0;
    }

    // method for updating internal map f this layer
    void HANPLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
    {
        // check for enable and availability of human tracking before continuing
        if(!enabled_
           || !lastTrackedHumans
           || ((ros::Time::now() - lastTrackedHumans->header.stamp) > human_tracking_delay)
           || !(use_safety || use_visibility)
          )
        {
            if(lastTransformedHumans.size() != 0)
            {
                lastTransformedHumans.clear();
                *min_x = last_min_x;
                *min_y = last_min_y;
                *max_x = last_max_x;
                *max_y = last_max_y;
            }
            return;
        }

        // clear last data
        lastTransformedHumans.clear();

        // TODO: use locking for lastTrackedHumans
        // check if frame transformation is still possible
        auto humans_header = lastTrackedHumans->header;

        // copy limits for the costmap to local variables
        double min_x_ = *min_x, min_y_ = *min_y, max_x_ = *max_x, max_y_ = *max_y;
        // transform each human into costmap frame and calculate bounds for costmap
        //lastTransformedHumans = new geometry_msgs::PoseStamped[lastTrackedHumans->tracks.size()]
        for(auto &human : lastTrackedHumans->tracks)
        {
            // don't consider this human if walking
            if (sqrt(human.twist.twist.linear.x * human.twist.twist.linear.x  +
                human.twist.twist.linear.y * human.twist.twist.linear.y) >= walking_velocity )
            {
                continue;
            }

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
                ROS_ERROR_NAMED("hanp_layer", "tf exception while transforming human pose in \\%s frame: %s",
                                global_frame_.c_str(), ex.what());
                lastTransformedHumans.clear();
                return;
            }

            // now calculate costmap bounds
            // note: bouds should be returned in meters, not in grid-points
            auto size_x = std::max(safety_max, visibility_max), size_y = size_x;

            min_x_ = std::min(min_x_, out_human.pose.position.x - size_x);
            min_y_ = std::min(min_y_, out_human.pose.position.y - size_y);
            max_x_ = std::max(max_x_, out_human.pose.position.x + size_x);
            max_y_ = std::max(max_y_, out_human.pose.position.y + size_y);

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
    void HANPLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
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
                ROS_ERROR_NAMED("hanp_layer", "no world coordinates for human at x:%f y:%f",
                               human.pose.position.x, human.pose.position.y);
                continue;
            }
            //ROS_INFO_NAMED("hanp_layer", "human x:%d y:%d", cell_x, cell_y);

            if(use_safety)
            {
                // apply safety grid according to position of each human
                auto size_x = (int)(safety_max / resolution), size_y = size_x;

                // by convention x is number of columns, y is number of rows
                for(int y = 0; y <= 2 * size_y; y++)
                {
                    for(int x = 0; x <= 2 * size_x; x++)
                    {
                        // add to current cost
                        auto cost = (double)master_grid.getCost(x + cell_x - size_x, y + cell_y - size_y)
                            + (double)(safety_grid[x + ((2 * size_x + 1) * y)] * safety_weight);
                        // detect overflow
                        if(cost > (int)costmap_2d::LETHAL_OBSTACLE)
                        {
                            cost = costmap_2d::LETHAL_OBSTACLE;
                        }

                        master_grid.setCost(x + cell_x - size_x, y + cell_y - size_y, (unsigned int)cost);
                    }
                }
            }

            if(use_visibility)
            {
                // calculate visibility grid
                auto visibility_grid = createVisibilityGrid(visibility_max, resolution, costmap_2d::LETHAL_OBSTACLE);

                // apply the visibility grid
                auto size_x = (int)(visibility_max / resolution), size_y = size_x;

                for(int y = 0; y <= 2 * size_y; y++)
                {
                    for(int x = 0; x <= 2 * size_x; x++)
                    {
                        // add to current cost
                        auto cost = (double)master_grid.getCost(x + cell_x - size_x, y + cell_y - size_y)
                            + (double)(visibility_grid[x + ((2 * size_x + 1) * y)] * visibility_weight);
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
    }

    void HANPLayer::humansUpdate(const hanp_layer_msgs::TrackedHumansPtr& humans)
    {
        // store humans to local variable
        lastTrackedHumans = humans;
    }

    // configure the SafetyLayer parameters
    void HANPLayer::reconfigureCB(hanp_layer::HANPLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;

        use_safety = config.use_safety;
        use_visibility= config.use_visibility;

        safety_max = use_safety ? config.safety_max : 0.0;
        visibility_max = use_visibility ? config.visibility_max : 0.0;

        // first copy the weights
        auto safety_weight_val = use_safety ? config.safety_weight : 0.0;
        auto visibility_weight_val = use_visibility ? config.visibility_weight : 0.0;

        // calculate effective weights, only when required
        auto total_weight = safety_weight_val + visibility_weight_val;
        if (total_weight > 0.0)
        {
            safety_weight = safety_weight_val / total_weight;
            visibility_weight = visibility_weight_val / total_weight;
        }
        else
        {
            safety_weight = visibility_weight = 0.0;
        }

        walking_velocity = config.walking_velocity;

        ROS_DEBUG_NAMED("hanp_layer", "safety_weight = %f, visibility_weight = %f",
                        safety_weight, visibility_weight);

        human_tracking_delay = ros::Duration(config.human_tracking_delay);

        // re-create safety grid, with changed safety_max and current resolution
        // TODO: what if resolution changes
        resolution = layered_costmap_->getCostmap()->getResolution();
        safety_grid = createSafetyGrid(safety_max, resolution, costmap_2d::LETHAL_OBSTACLE);
    }

    // creates safety grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* HANPLayer::createSafetyGrid(double safety_max, double resolution, unsigned int max_value)
    {
        if (safety_max < 0 || resolution < 0)
        {
            ROS_ERROR_NAMED("hanp_layer", "safety_max or resolution of safety grid cannot be negative");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)(safety_max / resolution), size_y = size_x;

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
                        (unsigned char)(((cos(r * x) +1) * (cos(r * y) +1) * max_value / 4) + 0.5);
                    // multiply by max_value to get full value range from 0-max_value
                    // addint 0.5 at the end because converstain of u_int trancates
                }
                else
                {
                    safetyGrid[index_1 + (index_2 * y) + x] = (unsigned char)0;
                }
            }
        }
        ROS_DEBUG_NAMED("hanp_layer", "created safety_grid of size %d %d", 2 * size_x + 1, 2 * size_y + 1);

        return safetyGrid;
    }


    // creates visibility grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* HANPLayer::createVisibilityGrid(double visibilityMax, double resolution,
                                                   unsigned int max_value, double look_x, double look_y)
    {
        if (visibilityMax < 0 || resolution < 0)
        {
            ROS_ERROR_NAMED("hanp_layer", "visibilityMax or resolution of visibility grid cannot be negative");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)(visibilityMax / resolution), size_y = size_x;

        // populate array using visibility function
        //  r = pi/visibilityMax
        //  g(i,j) = cos(r x i + 1) cos(r x j + 1) / 4
        //  Δψ = | arccos(Pi,j . L) |
        //    where P is vector to point (i,j) and L is vector of human looking direction
        //  fvisibility(x, y) = Δψ g(i,j)      if dist(i,j) <= D_visMax and Δψ >= Ψ
        //                    = 0              if dist(i,j) > D_visMax  or  Δψ < Ψ

        // we calculate the grid for Ψ = π/2
        // and we nomalize values for max value of 255  if dist(x,y) >  visibitlityMax

        // create a grid where human is at center, and #cells both sides = size
        auto visibilityGrid = new unsigned char[(2 * size_x + 1) * (2 * size_y + 1)];

        // fix angle for visibility
        double psi = M_PI / 2;

        // multiply r by resolution to get proper values for cells in sub-meter grid
        double r = M_PI * resolution / visibilityMax;

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
                double delta_psi;
                if ((x == 0) && (y == 0))
                {
                    delta_psi = M_PI / 2;
                }
                else
                {
                    delta_psi = fabs( acos( (look_x * x + look_y * y) /
                                      (sqrt(x*x + y*y) * sqrt(look_x*look_x + look_y*look_y)) ));
                }


                if ((sqrt((x * x) + (y * y)) <= (visibilityMax / resolution)) && delta_psi >= psi)
                {
                    visibilityGrid[index_1 + (index_2 * y) + x] =
                        (unsigned char)((delta_psi * ((cos(r * x) +1) * (cos(r * y) +1) * 255 / 4) / M_PI) + 0.5);

                    // devide by π to normalize
                    // multiply by 255 to get full value range from 0-255
                    // addint 0.5 at the end because converstain of u_int trancates
                }
                else
                {
                    visibilityGrid[index_1 + (index_2 * y) + x] = (unsigned char)0;
                }
            }
        }

        return visibilityGrid;
    }
}
