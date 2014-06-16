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

#include<human_aware_layers/safety_layer.h>

// declare the SafetyLayer as a Polygon class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(human_aware_layers::SafetyLayer, costmap_2d::Layer)

namespace human_aware_layers
{
    // onInitialize is called just after setting tf_, name_, and layered_costmap_
    void SafetyLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        //current_ = true; //TODO: find out use of this variable

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<human_aware_layers::SafetyLayerConfig>(nh);
        dynamic_reconfigure::Server<human_aware_layers::SafetyLayerConfig>::CallbackType cb =
            boost::bind(&SafetyLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // todo: subscribe to human positions
    }

    // method for updating internal map f this layer
    void SafetyLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if(!enabled_)
        {
            return;
        }
        // todo: implement
    }

    // method for applyting changes in this layer to global costmap
    void SafetyLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        if(!enabled_)
        {
            return;
        }

        // todo: implement
    }

    // configure the SafetyLayer parameters
    void SafetyLayer::reconfigureCB(human_aware_layers::SafetyLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }
}
