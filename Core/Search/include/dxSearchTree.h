/**
 * @file
 * @brief Source file for dxSearchTree
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
#ifndef DX_SEARCH_TREE_INCLUDE
#define DX_SEARCH_TREE_INCLUDE

#include <vector>
#include <nanoflann.hpp>
#include "dxPointCloud.h"
#include "Eigen/Dense"
#include <memory>

class dxSearchTree
{
public:

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTree;

    dxSearchTree() {}

    void setInputCloud(dxPointCloud cloud)
    {
        const int dim = 3;
        const int max_leaf = 15;

        dataMat.resize(cloud.size(), dim);

        //Filling the matrix
        for (int i = 0; i < cloud.size(); i++)
        {
            dataMat(i, 0) = cloud.points[i].x;
            dataMat(i, 1) = cloud.points[i].y;
            dataMat(i, 2) = cloud.points[i].z;
        }

        //std::cout << "------------------" << std::endl;
        //std::cout << dataMat << std::endl;
        //std::cout << "------------------" << std::endl;

        pTree.reset(new KDTree(dim, dataMat, max_leaf));
        pTree->index->buildIndex();

        this->cloud.clear();
        this->cloud = cloud;
    }

    int getCloudSize()
    {
        return cloud.size();
    }

    void setTree(const dxSearchTree tree)
    {
        *this = tree;
    }

    bool isValid()
    {
        return cloud.size() > 0;
    }

    std::pair<std::vector<int>, std::vector<float>> getPointsInSphere(dxPointCloud::PointT p, float radius)
    {
        if (!isValid())
        {
            std::cout << "getPointsInSphere() no cloud set" << std::endl;
            throw std::runtime_error("kdtree not initialized");
        }

        std::vector<float> query_pt(3);
        query_pt[0] = p.x;
        query_pt[1] = p.y;
        query_pt[2] = p.z;

        nanoflann::SearchParams params;
        std::vector<std::pair<Eigen::Index, float>> matches;
        double _radusSquared = radius * radius;
        const size_t nMatches = pTree->index->radiusSearch(&query_pt[0], _radusSquared, matches, params);
        //std::cout << "RadiusSearch(): radus^2 = " << _radusSquared << " -> "
        //	<< nMatches << " matches" << std::endl;

        std::vector<int> indexes;
        std::vector<float> distances;
        int i = 0;
        for (auto m : matches)
        {
            indexes.push_back(m.first);
            distances.push_back(m.second);
            //std::cout << "Idx[" << i << "] = " << matches[i].first
            //	<< " dist[" << i << "] = " << matches[i].second << std::endl;
            i++;
        }
        return make_pair(indexes, distances);
    }

protected:
    dxPointCloud cloud;
    std::shared_ptr<KDTree> pTree;
    Eigen::MatrixXf dataMat;
};

#endif
