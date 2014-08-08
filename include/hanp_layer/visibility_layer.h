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

#ifndef HANP_VISIBILITY_LAYER_H
#define HANP_VISIBILITY_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

#include <hanp_layer_msgs/TrackedHumans.h>
#include <hanp_layer/VisibilityLayerConfig.h>

namespace hanp_layer
{
    class VisibilityLayer : public costmap_2d::Layer
    {
        public:
        VisibilityLayer();

        // onInitialize is called when tf_, name_, and layered_costmap_ are already set
        virtual void onInitialize();

        // method for updating internal map f this layer
        virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y);

        // method for applyting changes in this layer to global costmap
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                               int max_i, int max_j);

        unsigned char* createVisibilityGrid(double radius,  double resolution = 0.5, double look_x = 1.0, double look_y = 0.0);

        private:
        ros::Subscriber humans_sub;
        void humansUpdate(const hanp_layer_msgs::TrackedHumansPtr& humans);

        void reconfigureCB(hanp_layer::VisibilityLayerConfig &config, uint32_t level);
        dynamic_reconfigure::Server<hanp_layer::VisibilityLayerConfig> *dsrv_;

        //std::map<HumanPosture, double[]> safetyGrids;
    };
}

#endif
