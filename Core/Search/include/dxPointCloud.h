/**
 * @file
 * @brief Source file for dxPointCloud
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
#ifndef DX_POINTCLOUD_INCLUDE
#define DX_POINTCLOUD_INCLUDE
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>

using namespace std;

class dxPointCloud
{
public:

    struct PointT
    {
        float x, y, z;
    };

    struct Normal
    {
        float normal_x, normal_y, normal_z;
    };

    struct PointNT
    {
        float x, y, z;
        float normal_x, normal_y, normal_z;
    };

    dxPointCloud() {};
    ~dxPointCloud() {};

    void addPoint(float x, float y, float z, float nx, float ny, float nz)
    {
        PointNT p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.normal_x = nx;
        p.normal_y = ny;
        p.normal_z = nz;
        points.push_back(p);
    }

    bool isInBounds(int i)
    {
        if (i >= points.size())
        {
            std::cout << "isInBounds(): Index out of Bound" << std::endl;
            return false;
        }
        return true;
    }

    PointT getPoint(int i)
    {
        PointT p;
        if (!isInBounds(i))
            return p;

        PointNT pnt = points[i];
        p.x = pnt.x;
        p.y = pnt.y;
        p.z = pnt.z;
        return p;
    }

    Normal getNormal(int i)
    {
        Normal n;
        if (!isInBounds(i))
            return n;

        PointNT pnt = points[i];
        n.normal_x = pnt.normal_x;
        n.normal_y = pnt.normal_y;
        n.normal_z = pnt.normal_z;
        return n;
    }

    void clear()
    {
        points.clear();
    }

    int size()
    {
        return points.size();
    }

    bool loadFromFile(string path)
    {
        ifstream f(path);
        if (!f.is_open())
        {
            std::cout << "loadPointCloud() could not open file: " << path << std::endl;
            f.close();
            return false;
        }

        bool headerSkipped = false;
        string line;
        while (getline(f, line))
        {
            if (!headerSkipped)
            {
                headerSkipped = true;
                continue;
            }

            istringstream iss(line);
            vector<string> lineVec{ istream_iterator<string>{iss},
                                    istream_iterator<string>{} };

            if (lineVec.size() >= 6)
            {
                addPoint(std::stof(lineVec[0]), std::stof(lineVec[1]), std::stof(lineVec[2]),
                         std::stof(lineVec[3]), std::stof(lineVec[4]), std::stof(lineVec[5]));
            }
        }
        f.close();
        return true;
    }

    std::vector<PointNT> points;
};
#endif
