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

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "human_aware_layers/safety_layer.h"
#include "human_aware_layers/visibility_layer.h"

int main( int argc, char* const argv[] )
{
  Catch::Session session; // there must be exactly one instance

  // writing to session.configData() here sets defaults
  // this is the preferred way to set them

  int returnCode = session.applyCommandLine( argc, argv );
  if( returnCode != 0 ) // indicates a command line error
    return returnCode;

  // writing to session.configData() or session.Config() here
  // overrides command line args
  // only do this if you know you need to

  return session.run();
}

TEST_CASE( "safety grid creation test" ) {
    //make dummy grid of 11x11 cells, resolution 0.5 meter/cell, safetyMax = 2.0 meters
    // based on follwing equation,
    //  r = pi/safetyMax
    //  safety(x, y) = (cos(r * x) + 1)(cos(r * y) +1) / 4

    int grid[] = { 0,  0,   0,   0,   0,   0,   0,  0, 0,
                   0,  0,  19,  32,  37,  32,  19,  0, 0,
                   0, 19,  64, 109, 128, 109,  64, 19, 0,
                   0, 32, 109, 186, 218, 186, 109, 32, 0,
                   0, 37, 128, 218, 255, 218, 128, 37, 0,
                   0, 32, 109, 186, 218, 186, 109, 32, 0,
                   0, 19,  64, 109, 128, 109,  64, 19, 0,
                   0,  0,  19,  32,  37,  32,  19,  0, 0,
                   0,  0,   0,   0,   0,   0,   0,  0, 0 };

    // get the grid from safetyLayer
    human_aware_layers::SafetyLayer safetyLayer;
    auto safetyGrid = safetyLayer.createSafetyGrid(2.0, 0.5);

    // first check the size
    //REQUIRE (sizeof grid == sizeof safetyGrid*);
    //cannot check size of char *

    // now check each element
    for (int i = 0; i < ((sizeof(grid) / sizeof(int))); i++)
    {
        CHECK(grid[i] == (int)safetyGrid[i]);
    }
}

TEST_CASE( "visibility grid creation test" ) {
    //make dummy grid of 11x11 cells, resolution 0.5 meter/cell, safetyMax = 2.0 meters
    // based on follwing equation,
    //  r = pi/visibilityMax
    //  g(i,j) = cos(r x i + 1) cos(r x j + 1) / 4
    //  Δψ = | arccos(Pi,j . L) |
    //    where P is vector to point (i,j) and L is vector of human looking direction
    //  fvisibility(x, y) = Δψ g(i,j)      if dist(i,j) <= D_visMax and Δψ >= Ψ
    //                    = 0              if dist(i,j) > D_visMax  or  Δψ < Ψ

    // we calculate the grid for Ψ = π/2
    // and we nomalize values for max value of 255
    int grid[] = { 0,  0,   0,   0,   0, 0, 0, 0, 0,
                   0,  0,  13,  19,  19, 0, 0, 0, 0,
                   0, 15,  48,  70,  64, 0, 0, 0, 0,
                   0, 29,  93, 139, 109, 0, 0, 0, 0,
                   0, 37, 128, 218, 128, 0, 0, 0, 0,
                   0, 29,  93, 139, 109, 0, 0, 0, 0,
                   0, 15,  48,  70,  64, 0, 0, 0, 0,
                   0,  0,  13,  19,  19, 0, 0, 0, 0,
                   0,  0,   0,   0,   0, 0, 0, 0, 0 };

    // get the grid from safetyLayer
    human_aware_layers::VisibilityLayer visibilityLayer;
    auto visibilityGrid = visibilityLayer.createVisibilityGrid(2.0, 0.5);

    // first check the size
    //REQUIRE (sizeof grid == sizeof safetyGrid*);
    //cannot check size of char *

    // now check each element
    for (int i = 0; i < ((sizeof(grid) / sizeof(int))); i++)
    {
        CHECK(grid[i] == (int)visibilityGrid[i]);
    }
}


