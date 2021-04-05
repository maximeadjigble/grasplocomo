/**
 * @file
 * @brief Source file for dxVoxel
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
#ifndef DX_VOXEL_INCLUDE
#define DX_VOXEL_INCLUDE

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include "dxAsyncTasks.h"
#include "dxSearchTree.h"

using namespace std;

class dxVoxel 
{
public:
	struct Color
	{
		double r, g, b, a;
	};

	struct Leaf
	{
		Leaf() :
			mean_(Eigen::Vector3d::Zero()),
			fisher_mean_(Eigen::Vector3d::Zero()),
			point_(Eigen::Vector3d::Zero()),
			zms_(Eigen::Vector3d::Zero()),
			zms_norm_(0),
			id_(-1),
			locomo_(0)
		{}

		void addIndex(int idx)
		{
			index.push_back(idx);
		}

		std::vector<int> getIndex()
		{
			return index;
		}

		double computeLocomo(double resolution)
		{
			zms_ = mean_ - point_;
			zms_proj_[2] = 0;
			zms_proj_[0] = zms_.dot(normal_);
			zms_proj_[1] = std::sqrt(zms_.norm()*zms_.norm() - zms_proj_[0] * zms_proj_[0]);


			Eigen::Matrix3d sigma;
			sigma << resolution / 100, 0, 0,
				0, resolution / 100, 0,
				0, 0, resolution / 100;

			double val = gaussian3(zms_proj_, Eigen::Vector3d::Zero(), sigma);
			double valMax = gaussian3(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sigma);
			locomo_ = val / valMax;
			return locomo_;
		}

		double gaussian3(Eigen::Vector3d x, Eigen::Vector3d mu, Eigen::Matrix3d sigma)
		{
			double val = -1;
			double det = sigma.determinant();
			if (det)
			{
				Eigen::VectorXd _x = -0.5*(x.transpose() - mu.transpose())*sigma.inverse()*(x - mu);
				val = 1 / (std::sqrt(std::pow((2 * M_PI), 3)*det))*std::exp(_x(0));
			}
			return val;
		}

		static inline Color colorMap(const float value, const float min, const float max)
		{
			uint8_t r, g, b;
			Color c;

			// Normalize value to [0, 1]
			float value_normalized = (value - min) / (max - min);

			float a = (1.0f - value_normalized) / 0.25f;
			int X = static_cast<int>(floorf(a));
			int Y = static_cast<int>(floorf(255.0f * (a - X)));

			switch (X)
			{
			case 0:
				r = 255;
				g = Y;
				b = 0;
				break;
			case 1:
				r = 255 - Y;
				g = 255;
				b = 0;
				break;
			case 2:
				r = 0;
				g = 255;
				b = Y;
				break;
			case 3:
				r = 0;
				g = 255 - Y;
				b = 255;
				break;
			}
			c.r = r;
			c.g = g;
			c.b = b;
			c.a = 255;
			return c;
		}


		Eigen::Vector3d mean_;
		Eigen::Vector3d fisher_mean_;
		Eigen::Vector3d zms_;
		Eigen::Vector3d zms_proj_;
		double zms_norm_;

		double locomo_;
		Eigen::Vector3d point_;
		Eigen::Vector3d normal_;

		std::vector<int> index;
		int id_;
	};

	typedef std::unordered_map<int, Leaf> MapS2el;
	typedef std::unordered_map<int, std::vector<Leaf>> MapS2elVec;

	struct VoxelData
	{
		std::vector<Leaf> elements;
		MapS2elVec s2;
		dxPointCloud cloud;
		dxSearchTree kdtree;
		string name;
		double resolution;

		VoxelData()
		{
			name = "voxel";
			resolution = 0.001;
		}

		void init(double resolution)
		{
			this->resolution = resolution;
			elements.clear();

			kdtree.setInputCloud(cloud);

			typedef struct Params {
				int id;
			};

			auto computeLeaf = [&](Params p) -> Leaf {
				int i = p.id;
				Leaf leaf;
				vector<int> indexes = getPointsInSphere(cloud.getPoint(i), resolution).first;
				int _size = indexes.size();
				for (int j = 0; j < _size; j++)
				{
					int jdx = indexes[j];
					Eigen::Vector3d p(cloud.getPoint(jdx).x, cloud.getPoint(jdx).y, cloud.getPoint(jdx).z);
					Eigen::Vector3d n(cloud.getNormal(jdx).normal_x, cloud.getNormal(jdx).normal_y, cloud.getNormal(jdx).normal_z);
					leaf.addIndex(jdx);

					leaf.mean_ += p;
				}
				leaf.point_[0] = cloud.getPoint(i).x;
				leaf.point_[1] = cloud.getPoint(i).y;
				leaf.point_[2] = cloud.getPoint(i).z;

				leaf.normal_[0] = cloud.getNormal(i).normal_x;
				leaf.normal_[1] = cloud.getNormal(i).normal_y;
				leaf.normal_[2] = cloud.getNormal(i).normal_z;
				leaf.mean_ = leaf.mean_ / _size;

				leaf.fisher_mean_[0] = leaf.normal_[0];
				leaf.fisher_mean_[1] = leaf.normal_[1];
				leaf.fisher_mean_[2] = leaf.normal_[2];

				double val = leaf.computeLocomo(resolution);
				//cout << "Locomo " << val << endl;
				return leaf;
			};

			//Asynchronous task to compute the leafs
			dxAsyncTasks<Params, Leaf> tasks;
			tasks.setAsyncFunction(computeLeaf);
			for (int i = 0; i < cloud.size(); i++) {
				Params p;
				p.id = i;
				tasks.addTask(p);
			}
			elements = tasks.getResults();

			//Color cloud
			auto _min_max = std::minmax_element(elements.begin(), elements.end(),
				[](Leaf e1, Leaf e2)
			{
				return e1.locomo_ < e2.locomo_;
			});

			double _min = _min_max.first->locomo_;
			double _max = _min_max.second->locomo_;
		}

		pair<vector<int>, vector<float>> getPointsInSphere(dxPointCloud::PointT p, double radius) {
			if (!cloud.size())
				throw std::runtime_error("kdtree not initialized: use initKdTree()");

			return kdtree.getPointsInSphere(p, radius);
		}

		int getNumPointsInSphere(dxPointCloud::PointT p, double radius) {
			return getPointsInSphere(p, radius).first.size();
		}

		double boxCollisionRatio(vector<double> box,
			dxPointCloud::PointT p,
			Eigen::Matrix3d axis = Eigen::Matrix3d::Identity())
		{
			double vMin = *std::min_element(box.begin(), box.end());
			double vMax = *std::max_element(box.begin(), box.end());
			int nMin = getNumPointsInSphere(p, vMin / 2.0);
			int nMax = getNumPointsInSphere(p, vMax / 2.0);
			if (!nMax)
				return -1;

			return (nMin / static_cast<double>(nMax));
		}

		bool isBoxColliding(vector<double> box,
			dxPointCloud::PointT p,
			double ratioMax,
			Eigen::Matrix3d axis = Eigen::Matrix3d::Identity())
		{
			return boxCollisionRatio(box, p, axis) > ratioMax;
		}
	};
};

#endif // !DX_VOXEL_INCLUDE