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

#ifndef HANP_LAYER_H
#define HANP_LAYER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>

#include <hanp_msgs/TrackedHumans.h>
#include <hanp_layer/HANPLayerConfig.h>

namespace hanp_layer
{
    class HANPLayer : public costmap_2d::Layer
    {
        public:
        HANPLayer();

        // onInitialize is called when tf_, name_, and layered_costmap_ are already set
        virtual void onInitialize();

        // method for updating internal map f this layer
        virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y);

        // method for applyting changes in this layer to global costmap
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                               int max_i, int max_j);

        unsigned char* createSafetyGrid(double radius,  double resolution=0.05, unsigned int max_value=255);

        unsigned char* createVisibilityGrid(double radius,  double resolution = 0.5,
                                            unsigned int max_value=255, double look_x = 1.0, double look_y = 0.0);

        private:
        ros::Subscriber humans_sub;
        void humansUpdate(const hanp_msgs::TrackedHumansPtr& humans);
        hanp_msgs::TrackedHumansPtr lastTrackedHumans;
        std::vector<geometry_msgs::PoseStamped> lastTransformedHumans;

        std::string global_frame_;

        void reconfigureCB(hanp_layer::HANPLayerConfig &config, uint32_t level);
        dynamic_reconfigure::Server<hanp_layer::HANPLayerConfig> *dsrv_;

        std::string default_human_segment_; // human segment to use
        double size_x, size_y;              // size of grid around human
        bool use_safety, use_visibility;    // whether to use these layers
        double safety_max, visibility_max;  // radius for safetry grid around human
        double safety_weight, visibility_weight;    // weights for weited-sum of costs
        double resolution;                  // resolution of the map, copied from master map
        double walking_velocity;            // above this velocity human will be considered walking
        ros::Duration human_tracking_delay; // maximum time to wait before considering human_tracking is no more available
        unsigned char* safety_grid; // variable to store created safety grid
        double last_min_x, last_min_y, last_max_x, last_max_y;  // last updated grid values

        // thread lock mutexex
        boost::mutex update_mutex_;
        boost::recursive_mutex configuration_mutex_;

        //std::map<HumanPosture, double[]> safetyGrids;
    };
}

#endif
