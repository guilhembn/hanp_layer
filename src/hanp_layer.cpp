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

#define THROTTLE_TIME 4 // seconds
#define TRACKED_HUMANS_TOPIC "humans"
#define DEFAULT_HUMAN_SEGMENT hanp_msgs::TrackedSegmentType::TORSO

#include <math.h>

#include <hanp_layer/hanp_layer.h>
#include <geometry_msgs/PoseStamped.h>
#include <hanp_msgs/TrackedSegmentType.h>

// declare the HANPLayer as a plugin class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hanp_layer::HANPLayer, costmap_2d::Layer)

namespace hanp_layer
{
    HANPLayer::HANPLayer() {}

    // onInitialize is called just after setting tf_, name_, and layered_costmap_
    // layered_costmap_ is the costmap which this plugin gets from the map_server
    void HANPLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);

        // subscribe to human positions
        std::string tracked_humans_topic;
        nh.param("tracked_humans_topic", tracked_humans_topic, std::string(TRACKED_HUMANS_TOPIC));
        humans_sub = nh.subscribe(tracked_humans_topic, 1, &HANPLayer::humansUpdate, this);
        nh.param("default_human_segment", default_human_segment_, (int)(DEFAULT_HUMAN_SEGMENT));

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<hanp_layer::HANPLayerConfig>(nh);
        dynamic_reconfigure::Server<hanp_layer::HANPLayerConfig>::CallbackType cb =
            boost::bind(&HANPLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // store global_frame name locally
        global_frame_ = layered_costmap_->getGlobalFrameID();
        resolution = layered_costmap_->getCostmap()->getResolution();
        if(resolution <= 0)
        {
            ROS_WARN("hanp_layer: resolution for grids is set to <= 0 (%f)", resolution);
        }

        // initialize last bounds
        last_min_x = last_min_y = last_max_x = last_max_y = 0.0;

        // declare current state of layer up-to-date
        current_ = true;
    }

    // method for updating internal map of this layer
    void HANPLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
    {
//        ROS_INFO_NAMED("HANPLayer", "updating Bounds.");
        update_mutex_.lock();
        // store trackedHumans for current update
        lastTrackedHumans = trackedHumans;
        update_mutex_.unlock();

        boost::recursive_mutex::scoped_lock lock(configuration_mutex_);

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
//            ROS_INFO_NAMED("HANPLayer", "(aborted) Bounds set to : %f, %f, %f, %f.", *min_x, *min_y, *max_x, *max_y);
//            //ROS_INFO_NAMED("HANPLayer", "enabled: %d, lastTrackedHuman: %d, delay: %d, safety/visi : %d/%d",enabled_, !lastTrackedHumans, (ros::Time::now() - lastTrackedHumans->header.stamp) > human_tracking_delay, use_safety, use_visibility);
//            ROS_INFO_NAMED("HANPLayer", "enabled : %d, sqfety : %d, visi : %d", enabled_, use_safety, use_visibility);
//            if (lastTrackedHumans) {
//                ROS_INFO_NAMED("HANPLayer", "delay : %f", (ros::Time::now() - lastTrackedHumans->header.stamp).toSec());
//            }
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
        //lastTransformedHumans = new geometry_msgs::PoseStamped[lastTrackedHumans->humans.size()]
        for(auto human : lastTrackedHumans->humans)
        {
            // check if default segment exists for this human
            auto segment_it = human.segments.end();
            for(auto it = human.segments.begin(); it != human.segments.end(); ++it)
            {
                if(it->type == default_human_segment_)
                {
                    segment_it = it;
                    break;
                }
            }

            // don't consider human if it does not have default segment
            if(segment_it == human.segments.end())
            {
                continue;
            }
            else
            {
                auto segment = *segment_it;

                // don't consider this human if walking (segment is moving)
                if (sqrt(segment.twist.twist.linear.x * segment.twist.twist.linear.x  +
                    segment.twist.twist.linear.y * segment.twist.twist.linear.y) >= walking_velocity )
                {
                    continue;
                }

                // fist transform human positions
                geometry_msgs::PoseStamped in_human_segment, out_human_segment;
                in_human_segment.header.stamp = humans_header.stamp;
                in_human_segment.header.frame_id = humans_header.frame_id;
                in_human_segment.pose = segment.pose.pose;
                try
                {
                    // lookup and transform in_human in global_frame_
                    // at time found in in_human
                    tf_->transformPose(global_frame_, in_human_segment, out_human_segment);
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("hanp_layer: tf exception while transforming human pose in \\%s frame: %s",
                                    global_frame_.c_str(), ex.what());
                    lastTransformedHumans.clear();
                    return;
                }

                // now calculate costmap bounds
                // note: bouds should be returned in meters, not in grid-points
                auto size_x = std::max(safety_max, visibility_max), size_y = size_x;

                min_x_ = std::min(min_x_, out_human_segment.pose.position.x - size_x);
                min_y_ = std::min(min_y_, out_human_segment.pose.position.y - size_y);
                max_x_ = std::max(max_x_, out_human_segment.pose.position.x + size_x);
                max_y_ = std::max(max_y_, out_human_segment.pose.position.y + size_y);

                lastTransformedHumans.push_back(out_human_segment);
            }
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
        ROS_DEBUG("hanp_layer: updated bounds to min_x=%f, min_y=%f, max_x=%f, max_y=%f",
             last_min_x, last_min_y, last_max_x, last_max_y);
    }

    // method for applyting changes in this layer to global costmap
    void HANPLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
        ROS_DEBUG("updating Costs...");

        if(!enabled_ || (lastTransformedHumans.size() == 0))
        {
            ROS_DEBUG("Not Enabled or lastTransformedHuman empty");
            return;
        }

        // update map according to each human position
        for(auto human : lastTransformedHumans)
        {
            // get position of human in map
            unsigned int cell_x, cell_y;
            if(!master_grid.worldToMap(human.pose.position.x, human.pose.position.y, cell_x, cell_y))
            {
                ROS_ERROR("hanp_layer: no world coordinates for human at x:%f y:%f",
                               human.pose.position.x, human.pose.position.y);
                continue;
            }

            // general algorithm is to
            // create satic grids according to human posture, standing, sitting, walking
            // to calcuate cost use a sigmoid function
            // however 'sitting' posture is not implemented yet

            if(use_safety)
            {
                if(safety_grid == NULL)
                {
                    ROS_WARN_THROTTLE(THROTTLE_TIME, "hanp_layer: not updating costmap as safety_grid is empty");
                    return;
                }

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
                double yaw = tf::getYaw(human.pose.orientation);
                auto visibility_grid = createVisibilityGrid(visibility_max, resolution,
                                                            costmap_2d::LETHAL_OBSTACLE, cos(yaw), sin(yaw));
                if(visibility_grid == NULL)
                {
                    ROS_WARN_THROTTLE(THROTTLE_TIME, "hanp_layer: not updating costmap as visibility_grid is empty");
                    return;
                }

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

        ROS_DEBUG("hanp_layer: updated costs");
    }

    void HANPLayer::humansUpdate(const hanp_msgs::TrackedHumansPtr& humans)
    {
        update_mutex_.lock();
        // store humans to local variable
        trackedHumans = humans;
        ROS_INFO_ONCE("hanp_layer: subscribed to humans");
        update_mutex_.unlock();
    }

    // configure the SafetyLayer parameters
    void HANPLayer::reconfigureCB(hanp_layer::HANPLayerConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock lock(configuration_mutex_);

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

        human_psi = config.humans_psi * M_PI / 180;
        walking_velocity = config.walking_velocity;

        ROS_DEBUG("hanp_layer: safety_weight = %f, visibility_weight = %f",
                        safety_weight, visibility_weight);

        human_tracking_delay = ros::Duration(config.human_tracking_delay);

        // re-create safety grid, with changed safety_max and current resolution
        resolution = layered_costmap_->getCostmap()->getResolution();
        if(resolution <= 0)
        {
            ROS_WARN("hanp_layer: resolution for grids is set to <= 0 (%f)", resolution);
        }
        safety_grid = createSafetyGrid(safety_max, resolution, costmap_2d::LETHAL_OBSTACLE);
    }

    // creates safety grid around the human for different postures
    // radius is in meters
    // resolution is in meters/cell, default is 0.5
    unsigned char* HANPLayer::createSafetyGrid(double safety_max, double resolution, unsigned int max_value)
    {
        if (safety_max < 0)
        {
            ROS_ERROR("hanp_layer: safety_max of safety grid cannot be negative");
            return NULL;
        }
        if(resolution <= 0)
        {
            ROS_ERROR("hanp_layer: resolution of safety grid has to be positive");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape. since that is not possible, we use square and put 0 otherwise
        auto size_x = (int)(safety_max / resolution), size_y = size_x;

        // create safety grid array using safety function (a sigmoid function)
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
        ROS_DEBUG("hanp_layer: created safety_grid of size %d %d", 2 * size_x + 1, 2 * size_y + 1);

        return safetyGrid;
    }


    // creates visibility grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* HANPLayer::createVisibilityGrid(double visibility_max, double resolution,
                                                   unsigned int max_value, double look_x, double look_y)
    {
        if (visibility_max < 0)
        {
            ROS_ERROR("hanp_layer: visibility_max of visibility grid cannot be negative");
            return NULL;
        }
        if(resolution <= 0)
        {
            ROS_ERROR("hanp_layer: resolution of visibility grid has to be positive");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)(visibility_max / resolution), size_y = size_x;

        // create visibility grid array using visibility function
        //  r = pi/visibility_max
        //  g(i,j) = cos(r x i + 1) cos(r x j + 1) / 4
        //  Δψ = | arccos(Pi,j . L) |
        //    where P is vector to point (i,j) and L is vector of human looking direction
        //  fvisibility(x, y) = Δψ g(i,j)      if dist(i,j) <= D_visMax and Δψ >= Ψ
        //                    = 0              if dist(i,j) > D_visMax  or  Δψ < Ψ

        // we calculate the grid for Ψ = π/2
        // and we nomalize values for max value of 255  if dist(x,y) >  visibitlityMax

        // create a grid where human is at center, and #cells both sides = size
        auto visibilityGrid = new unsigned char[(2 * size_x + 1) * (2 * size_y + 1)];

        // angle for visibility
        double psi = human_psi / 2;

        // multiply r by resolution to get proper values for cells in sub-meter grid
        double r = M_PI * resolution / visibility_max;

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


                if ((sqrt((x * x) + (y * y)) <= (visibility_max / resolution)) && delta_psi >= psi)
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
