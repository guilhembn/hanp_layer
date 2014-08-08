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

#include<hanp_layer/visibility_layer.h>

// declare the SafetyLayer as a Polygon class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hanp_layer::VisibilityLayer, costmap_2d::Layer)

namespace hanp_layer
{
    VisibilityLayer::VisibilityLayer() {}

    // onInitialize is called just after setting tf_, name_, and layered_costmap_
    // layered_costmap_ is the costmap which plugin gets from the map_server
    void VisibilityLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        //current_ = true; //TODO: find out use of this variable

        // subscribe to human positions
        std::string humans_topic;
        nh.param("humans_topic", humans_topic, std::string("/humans"));
        humans_sub = nh.subscribe(humans_topic, 1, &VisibilityLayer::humansUpdate, this);

        // create satic grids according to human posture, standing, sitting, walking
        // to calcuate cost use a sigmoid function

        // for visibility
        //// cosine function is 0 at borders of radius
        //distance_cosine = pow(cos((distance * M_PI_2) / radius), 2); // value between 0 and 1
        //
        //// use stretch to increase / decrease weight more on more backward angles
        //val = distance_cosine * (height + (angle_deviation - M_PI_4) * stretch_back);

        // set up dynamic reconfigure server
        dsrv_ = new dynamic_reconfigure::Server<hanp_layer::VisibilityLayerConfig>(nh);
        dynamic_reconfigure::Server<hanp_layer::VisibilityLayerConfig>::CallbackType cb =
            boost::bind(&VisibilityLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    // method for updating internal map f this layer
    void VisibilityLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
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
    void VisibilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j)
    {
        if(!enabled_)
        {
            // todo: implement
            return;
        }
    }

    void VisibilityLayer::humansUpdate(const hanp_layer_msgs::TrackedHumansPtr& humans)
    {
        // store humans to local variable
    }

    // configure the SafetyLayer parameters
    void VisibilityLayer::reconfigureCB(hanp_layer::VisibilityLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    // creates visibility grid around the human for different postures
    // radius in meters
    // resolution in meters/cell, default is 0.5
    unsigned char* VisibilityLayer::createVisibilityGrid(double visibilityMax, double resolution,
                                                         double look_x, double look_y)
    {
        if (visibilityMax < 0 || resolution < 0)
        {
            ROS_ERROR_NAMED("visibility_layer", "visibilityMax or resolution of visibility grid cannot be negative");
            return NULL;
        }

        // size of the grid depends on the resolution
        // array should be of 'round' shape, since it not possible we use square and put 0 otherwise
        auto size_x = (int)((visibilityMax / resolution) + 0.05), size_y = size_x;

        // populate array using visibility function
        //  r = pi/visibilityMax
        //  g(i,j) = cos(r x i + 1) cos(r x j + 1) / 4
        //  Δψ = | arccos(Pi,j . L) |
        //    where P is vector to point (i,j) and L is vector of human looking direction
        //  fvisibility(x, y) = Δψ g(i,j)      if dist(i,j) <= D_visMax and Δψ >= Ψ
        //                    = 0              if dist(i,j) > D_visMax  or  Δψ < Ψ

        // we calculate the grid for Ψ = π/2
        // and we nomalize values for max value of 255  if dist(x,y) >  safetyMax

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
