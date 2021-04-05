/**
 * @file
 * @brief Source file for dxPoint3
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
#ifndef DX_POINT3_INCLUDE
#define DX_POINT3_INCLUDE

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "dxPoint3.h"
#include <vector>
#include <array>


class dxP3
{
public:
    dxP3() {};
    dxP3(double x, double y, double z)
    {
        setVector(x, y, z);
    };

    dxP3(Eigen::Vector3d v)
    {
        setVector(v);
    };

    ~dxP3() {};

    void setZero()
    {
        x = y = z = 0;
    }

    Eigen::Vector3d getVector()
    {
        Eigen::Vector3d v;
        v << x, y, z;
        return v;
    }

    std::vector<double> get()
    {
        return { x, y, z };
    }

    void setVector(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void setVector(Eigen::Vector3d v)
    {
        this->x = v[0];
        this->y = v[1];
        this->z = v[2];
    }

    double norm()
    {
        return std::sqrt(x * x + y * y + z *z);
    }

    std::array<double, 3> toSherical()
    {
        double phi = atan2(y, x);
        phi = (phi > 0 ? phi : (2 * M_PI + phi));
        double r = norm();
        double theta = 0;
        if (r > 1e-6)
            theta = acos(z / r);
        return { r, theta, phi };
    }

    std::array<int, 3> cartesianIJK(double resolution)
    {
        if (resolution == 0)
            resolution = 1;

        std::array<int, 3> ijk;
        ijk[0] = static_cast<int>(x/resolution);
        ijk[1] = static_cast<int>(y/resolution);
        ijk[2] = static_cast<int>(z/resolution);

        return ijk;
    }

    std::array<int, 3> shpericalIJK(double resolution, int band)
    {
        if (resolution == 0)
            resolution = 1;

        std::array<double, 3> polar = toSherical();
        int idr = static_cast<int>(polar[0] / resolution);
        std::array<int, 2> jk = spherical2JK(polar[1], polar[2], band);
        return { idr, jk[0], jk[1] };
    }

    static std::array<int, 2> spherical2JK(double theta, double phi, int band)
    {
        int j = static_cast<int>((2 * band*theta) / M_PI - 0.5);
        int k = static_cast<int>(band*phi / M_PI);
        return { j, k };
    }

    void print(std::string name="")
    {
        std::cout << name << " => x: " << x << " | y: "<< y << " | z: " << z << std::endl;
    }

protected:
    double x, y, z;
};


class dxPoint3 : public dxP3
{
public:
    dxPoint3(): dxP3() {};
    dxPoint3(double x, double y, double z) : dxP3(x, y, z) {};
    dxPoint3(Eigen::Vector3d v) : dxP3(v) {};

    dxPoint3 transform(Eigen::Matrix4d M, bool set=false)
    {
        Eigen::Vector3d vin = getVector();
        Eigen::Vector4d vout;
        vout << vin[0], vin[1], vin[2], 1;
        Eigen::Vector4d v = M * vout;
        if (set)
            setVector(v[0], v[1], v[2]);

        return dxPoint3(v[0], v[1], v[2]);
    }

};


class dxNormal3: public dxP3
{
public:
    dxNormal3() : dxP3() {};
    dxNormal3(double nx, double ny, double nz) : dxP3(nx, ny, nz) {};
    dxNormal3(Eigen::Vector3d v) : dxP3(v) {};

    dxNormal3 transform(Eigen::Matrix4d M, bool set = false)
    {
        Eigen::Vector3d v = M.block(0,0,3,3) * getVector();
        if (set)
            setVector(v);

        return dxNormal3(v[0], v[1], v[2]);
    }

};

#endif // !DX_POINT3_INCLUDE