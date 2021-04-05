/**
 * @file
 * @brief Source file for dxGraspLoCoMo
 * @author Maxime Adjigble
 * @author Naresh Marturi
 * @ref License
 */
 
/*
* BSD 3 - Clause License
*
* Copyright(c) 2021, Maxime Adjigble 
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met :
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#ifndef DX_GRASP_LOCOMO_INCLUDE
#define DX_GRASP_LOCOMO_INCLUDE

#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <time.h>
#include <exception>
#include "dxGripperModel.h"
#include "dxVoxel.h"
#include "dxSearchTree.h"
//#include "dxVision.h"


class dxGraspLoCoMo
{
public:
    dxGraspLoCoMo();
    ~dxGraspLoCoMo();

    typedef std::pair<dxGripperModel::Feature, dxGripperModel::Feature> FeaturePair;
    typedef std::vector<FeaturePair> FeaturePairVec;

    typedef std::pair<dxGripperModel::Feature, std::vector<dxGripperModel::Feature>> FeaturePairOneToMany;
    typedef std::vector<FeaturePairOneToMany> FeaturePairOneToManyVec;

    typedef dxGripperModel::GraspModelPG70 GraspPG70;
    typedef std::vector<GraspPG70> GraspPG70Vec;

    struct GraspResult
    {
        GraspPG70Vec grasps;

        void print()
        {
            cout << "[RESULTS] : "
                 << "Ngrasps " << grasps.size()  << endl;
        }
    };

    void setResolution(double downsampling = 0.008, int resolutionFactor = 4);
    GraspResult locomoGrasp(dxPointCloud cloud);
    void saveGrasps(string path);

    static void printMsg(string message, string prep = "", string end = "\n");
protected:

    void loadGripperCloud()
    {
        gripper.cloud.addPoint(1, 0, 0, 1, 0, 0);
        gripper.cloud.addPoint(-1, 0, 0, -1, 0, 0);
    };


    //shared_ptr<vision> pVision;
    //shared_ptr<dxVision> viz;

    dxVoxel::VoxelData scene;
    dxVoxel::VoxelData gripper;
    double resolution;
    double downsampling;

    GraspResult results;
};

#endif // !DX_GRASP_LOCOMO_INCLUDE
