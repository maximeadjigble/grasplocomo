/**
 * @file
 * @brief Source file for dxGripperModel
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
#ifndef DX_GRIPPER_MODEL_INCLUDE
#define DX_GRIPPER_MODEL_INCLUDE

#include <iostream>
#include <string>
#include "dxPoint3.h"
#include <vector>
#include <unordered_map>
#include "dxSearchTree.h"
#include <map>

using namespace std;

class dxGripperModel
{
public:
    typedef std::pair<dxPoint3, dxNormal3> ptNorm;
    typedef std::pair<int, ptNorm> IndexedPtNorm;

    dxGripperModel() {};
    ~dxGripperModel();

    struct Feature
    {
        Eigen::Matrix4d ref;
        std::vector<std::pair<int, ptNorm>> features;
        double prob;
        bool isValid;

        Feature()
        {
            ref = Eigen::Matrix4d::Identity();
            isValid = false;
            clear();
        }

        static dxGripperModel::IndexedPtNorm getIndexedPtNorm(int idx, dxPoint3 p, dxNormal3 n)
        {
            return IndexedPtNorm(idx, std::pair<dxPoint3, dxNormal3>(p, n));
        }

        bool isForceClosurePG70(double angle, double opening = 1e6)
        {
            if (features.size() != 2)
            {
                cout << "[Error] Size() should be 2. size=" << features.size() << endl;
                return false;
            }

            //std::cout << "----------------------" << std::endl;
            for (auto f : features)
            {
                Eigen::Vector3d pj = f.second.first.getVector();
                pj.normalize();
                Eigen::Vector3d nj = f.second.second.getVector();
                //std::cout << "p " << pj.transpose() << " n " << nj.transpose() << std::endl;
                if (pj.dot(nj) < ::cos(angle))
                    return false;
            }

            Eigen::Vector3d p1 = features[0].second.first.getVector();
            Eigen::Vector3d p2 = features[1].second.first.getVector();

            if ((p1 - p2).norm() > opening)
                return false;

            return true;
        }

        Eigen::Matrix4d computeRef(std::pair<int, ptNorm> f1, std::pair<int, ptNorm> f2, bool save = true)
        {
            double epsd = 1e-9;
            double epsdot = 1 - 1e-9;
            int i = f1.first;
            int j = f2.first;
            ptNorm p1 = f1.second;
            ptNorm p2 = f2.second;
            Eigen::Vector3d axisX, axisY, axisZ;
            Eigen::Vector3d pj = p1.first.getVector();
            Eigen::Vector3d nj = p1.second.getVector();
            Eigen::Vector3d pk = p2.first.getVector();
            Eigen::Vector3d nk = p2.second.getVector();

            Eigen::Vector3d pm = (pj + pk) / 2.0;
            Eigen::Vector3d pm2pj = pj - pm;
            double d = pm2pj.norm();

            if (d < epsd)
                return Eigen::Matrix4d::Identity();

            axisX = pm2pj;
            axisX.normalize();

            //double njProjpjk = nj.dot(pm2pj) / d;
            double njProjpjk = nj.dot(axisX);

            if (::fabs(njProjpjk) > epsdot)
            {
                Eigen::Vector3d vecZ;
                vecZ << 0, 0, 1;
                if (::fabs(axisX.dot(vecZ)) < epsdot)
                    axisZ = axisX.cross(vecZ);
                else
                {
                    Eigen::Vector3d vecY;
                    vecY << 0, 1, 0;
                    axisZ = axisX.cross(vecY);
                }
            }
            else
                axisZ = axisX.cross(nj);

            axisZ.normalize();
            axisY = axisZ.cross(axisX);

            Eigen::Matrix4d _ref;
            _ref.col(0) = Eigen::Vector4d(axisX[0], axisX[1], axisX[2], 0.0);
            _ref.col(1) = Eigen::Vector4d(axisY[0], axisY[1], axisY[2], 0.0);
            _ref.col(2) = Eigen::Vector4d(axisZ[0], axisZ[1], axisZ[2], 0.0);
            _ref(0, 3) = pm(0);
            _ref(1, 3) = pm(1);
            _ref(2, 3) = pm(2);
            _ref(3, 3) = 1.0;

            if (save)
                ref = _ref;
            return _ref;
        }


        Feature fromPtNorm(std::pair<int, ptNorm> f1, std::pair<int, ptNorm> f2)
        {
            double epsd = 1e-9;
            double epsdot = 1 - 1e-9;
            int i = f1.first;
            int j = f2.first;
            ptNorm p1 = f1.second;
            ptNorm p2 = f2.second;
            Eigen::Vector3d axisX, axisY, axisZ;
            Eigen::Vector3d pj = p1.first.getVector();
            Eigen::Vector3d nj = p1.second.getVector();
            Eigen::Vector3d pk = p2.first.getVector();
            Eigen::Vector3d nk = p2.second.getVector();

            Eigen::Matrix4d _ref = computeRef(f1, f2);
            Eigen::Matrix4d err = _ref - Eigen::Matrix4d::Identity();

            if (err.norm() < epsd)
                return *this;

            Eigen::Matrix4d ref_inv = ref.inverse();
            Eigen::Vector4d pjr = ref_inv * Eigen::Vector4d(pj[0], pj[1], pj[2], 1.0);
            Eigen::Vector3d njr = ref_inv.block(0, 0, 3, 3) * nj;
            Eigen::Vector4d pkr = ref_inv * Eigen::Vector4d(pk[0], pk[1], pk[2], 1.0);
            Eigen::Vector3d nkr = ref_inv.block(0, 0, 3, 3) * nk;

            //Update Feature
            //setRef(ref);
            add(i, dxPoint3(pjr[0], pjr[1], pjr[2]), dxNormal3(njr[0], njr[1], njr[2]));
            add(j, dxPoint3(pkr[0], pkr[1], pkr[2]), dxNormal3(nkr[0], nkr[1], nkr[2]));
            isValid = true;
            return *this;
        }

        int size()
        {
            return features.size();
        }

        Eigen::Matrix4d getRef2()
        {
            return computeRef(features[0], features[1], false);
        }

        std::vector<double> getPoint(int i)
        {
            if (i >= features.size())
                return std::vector<double>();

            return features[i].second.first.get();
        }

        Feature getInLocalCoord()
        {
            Feature feat;
            Eigen::Matrix4d T = ref.inverse();
            feat.setRef(T);
            for (auto f : features)
            {
                Eigen::Vector3d pj = f.second.first.getVector();
                Eigen::Vector3d nj = f.second.second.getVector();
                Eigen::Vector4d pjr = T * Eigen::Vector4d(pj[0], pj[1], pj[2], 1.0);
                Eigen::Vector3d njr = T.block(0, 0, 3, 3) * nj;
                feat.add(f.first, dxPoint3(pjr[0], pjr[1], pjr[2]), dxNormal3(njr[0], njr[1], njr[2]));
            }
            return feat;
        }

        Feature getInGlobalCoord()
        {
            Feature feat;
            Eigen::Matrix4d T = ref;
            feat.setRef(T);
            for (auto f : features)
            {
                Eigen::Vector3d pj = f.second.first.getVector();
                Eigen::Vector3d nj = f.second.second.getVector();
                Eigen::Vector4d pjr = T * Eigen::Vector4d(pj[0], pj[1], pj[2], 1.0);
                Eigen::Vector3d njr = T.block(0, 0, 3, 3) * nj;
                feat.add(f.first, dxPoint3(pjr[0], pjr[1], pjr[2]), dxNormal3(njr[0], njr[1], njr[2]));
            }
            return feat;
        }

        Feature transform(Eigen::Matrix4d M = Eigen::Matrix4d::Identity())
        {
            Feature feat;
            Eigen::Matrix4d T = ref * M;
            feat.setRef(T);
            for (auto f : features)
            {
                Eigen::Vector3d pj = f.second.first.getVector();
                Eigen::Vector3d nj = f.second.second.getVector();
                Eigen::Vector4d pjr = T * Eigen::Vector4d(pj[0], pj[1], pj[2], 1.0);
                Eigen::Vector3d njr = T.block(0, 0, 3, 3) * nj;
                feat.add(f.first, dxPoint3(pjr[0], pjr[1], pjr[2]), dxNormal3(njr[0], njr[1], njr[2]));
            }
            return feat;
        }

        int getIndex(int i)
        {
            if (i >= features.size())
                return -1;

            return features[i].first;
        }

        std::vector<double> getNormal(int i)
        {
            if (i >= features.size())
                return std::vector<double>();

            return features[i].second.second.get();
        }

        std::vector<double> getVec()
        {
            std::vector<double> vec;
            for (auto f : features)
            {
                std::vector<double> ps = f.second.first.get();
                std::vector<double> ns = f.second.second.get();
                vec.emplace_back(ps[0]);
                vec.emplace_back(ps[1]);
                vec.emplace_back(ps[2]);
                vec.emplace_back(ns[0]);
                vec.emplace_back(ns[1]);
                vec.emplace_back(ns[2]);
                vec.emplace_back(f.first);
            }

            //Add Reference frame 12*1
            for (int c = 0; c < 4; c++)
            {
                for (int l = 0; l < 3; l++)
                {
                    vec.emplace_back(ref(l, c));
                }
            }

            return vec;
        }

        void add(int idx, dxPoint3 p, dxNormal3 n)
        {
            features.emplace_back(getIndexedPtNorm(idx, p, n));
        }

        void setRef(Eigen::Matrix4d ref)
        {
            this->ref = ref;
        }

        void clear()
        {
            features.clear();
        }

    };

    struct BSphere
    {
        double x, y, z;
        double radius;
        std::vector<int> idxPtsInSphere;

        BSphere()
        {
            init();
        }

        BSphere(double x, double y, double z, double radius)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->radius = radius;
        }

        BSphere(dxPointCloud::PointT p, double radius)
        {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
            this->radius = radius;
        }

        void init()
        {
            x = 0;
            y = 0;
            z = 0;
            radius = 0;
            idxPtsInSphere.clear();
        }

        dxPointCloud::PointT getCenter()
        {
            dxPointCloud::PointT p;
            p.x = x;
            p.y = y;
            p.z = z;
            return p;
        }

        double getRadius()
        {
            return radius;
        }

        int getPtSize()
        {
            return idxPtsInSphere.size();
        }
    };

    struct BBox : public dxSearchTree
    {
        double x, y, z;
        Eigen::Matrix4d M;
        std::vector<int> idxPtsInBox;
        double collisionRatio;
        bool useMinDim;
        bool useMaxDim;
        BSphere bSphereMin;
        BSphere bSphereMax;
        std::vector<BSphere> bSphereApprox;

        BBox()
        {
            init();
        }

        BBox(double x, double y, double z, Eigen::Matrix4d M = Eigen::Matrix4d::Identity())
        {
            init();
            this->x = x;
            this->y = y;
            this->z = z;
            this->M = M;
            setMinMaxSpheres();
        }

        void init()
        {
            x = 0;
            y = 0;
            z = 0;
            M = Eigen::Matrix4d::Identity();

            idxPtsInBox.clear();
            bSphereApprox.clear();
            useMinDim = false;
            useMaxDim = false;
            collisionRatio = 0;
        }

        vector<double> getValues()
        {
            return { x, y, z };
        }

        void setUseMinMaxDim(bool useMinDim, bool useMaxDim)
        {
            this->useMinDim = useMinDim;
            this->useMaxDim = useMaxDim;
        }

        double getMinValue()
        {
            vector<double> values = getValues();
            return *std::min_element(values.begin(), values.end());
        }

        double getMaxValue()
        {
            vector<double> values = getValues();
            return *std::max_element(values.begin(), values.end());
        }

        double getMinRadius()
        {
            return useMinDim ? getMinValue() / 2.0 : getMaxValue() / 2.0;
        }

        double getMaxRadius()
        {
            return useMaxDim ? getMaxValue() / 2.0 : sqrt(x*x + y * y + z * z) / 2.0;
        }

        vector<double> getCenter()
        {
            return { M(0,3), M(1,3), M(2,3) };
        }

        Eigen::Matrix3d getRotation()
        {
            return M.block<3, 3>(0, 0);
        }

        void setPose(Eigen::Matrix4d M)
        {
            this->M = M;
            setMinMaxSpheres();
        }

        void setMinMaxSpheres()
        {
            dxPointCloud::PointT p;
            p.x = M(0, 3);
            p.y = M(1, 3);
            p.z = M(2, 3);
            bSphereMin = BSphere(p, getMinRadius());
            bSphereMax = BSphere(p, getMaxRadius());
        }

        pair<vector<int>, vector<float>> getPointsInSphere(BSphere sphere)
        {
            return dxSearchTree::getPointsInSphere(sphere.getCenter(), sphere.radius);
        }

        void setApproxSpheres(double overlapRatio = 0.8)
        {
            //Contructing the spheres to approximate the bbox
            std::vector<double> dims = getValues();
            int idxDimMin = std::distance(dims.begin(), std::min_element(dims.begin(), dims.end()));
            double dimMin = dims[idxDimMin];
            double radius = dimMin / 2.0;

            double halfx = dims[0] / 2.0 - radius;
            double halfy = dims[1] / 2.0 - radius;
            double halfz = dims[2] / 2.0 - radius;
            double dr = 2 * radius *(1 - overlapRatio);
            bSphereApprox.clear();
            bool fistIt = true;
            for (double xi = 0; xi <= halfx; xi += dr)
            {
                for (double yi = 0; yi <= halfy; yi += dr)
                {
                    for (double zi = 0; zi <= halfz; zi += dr)
                    {
                        Eigen::Vector4d pi;
                        pi << xi, yi, zi, 1;
                        Eigen::Vector4d piRb = M * pi;
                        bSphereApprox.push_back(BSphere(piRb(0), piRb(1), piRb(2), radius));
                        if (fistIt)
                        {
                            fistIt = false;
                            continue;
                        }

                        //TODO - Remove duplicates

                        //Create mirror point on each axis
                        std::vector<int> flipIdx;
                        for (int idx = 0; idx < dims.size(); idx++)
                        {
                            pi << xi, yi, zi, 1;
                            if (idx == idxDimMin)
                                continue;

                            flipIdx.push_back(idx);
                            pi[idx] = -pi[idx];
                            piRb = M * pi;
                            bSphereApprox.push_back(BSphere(piRb(0), piRb(1), piRb(2), radius));
                        }

                        //Create oposite point on all axis
                        pi << xi, yi, zi, 1;
                        for (int idx : flipIdx)
                        {
                            pi[idx] = -pi[idx];
                        }
                        piRb = M * pi;
                        bSphereApprox.push_back(BSphere(piRb(0), piRb(1), piRb(2), radius));
                    }
                }
            }
        }

        double getNPtsInBBoxApprox()
        {
            idxPtsInBox.clear();
            for (auto& sphere : bSphereApprox)
            {
                sphere.idxPtsInSphere = getPointsInSphere(sphere).first;
                for (auto idx : sphere.idxPtsInSphere)
                {
                    if (std::find(idxPtsInBox.begin(), idxPtsInBox.end(), idx) == idxPtsInBox.end())
                    {
                        idxPtsInBox.emplace_back(idx);
                    }
                }
            }

            return idxPtsInBox.size();
        }

        double getCollisionRatio()
        {
            bSphereMin.idxPtsInSphere = getPointsInSphere(bSphereMin).first;
            bSphereMax.idxPtsInSphere = getPointsInSphere(bSphereMax).first;
            int nMin = bSphereMin.getPtSize();
            int nMax = bSphereMax.getPtSize();
            if (!nMax)
                return -1;

            return (nMin / static_cast<double>(nMax));
        }

        bool inCollision(double ratioMax)
        {
            collisionRatio = getCollisionRatio();
            return collisionRatio > ratioMax;
        }
    };

    struct Finger : public BBox
    {
        Eigen::Matrix4d offsetRg;

        Finger(Eigen::Matrix4d offsetRg = Eigen::Matrix4d::Identity()) : BBox(), offsetRg(offsetRg) {}
        Finger(BBox bbox, Eigen::Matrix4d offsetRg = Eigen::Matrix4d::Identity()) : BBox(bbox), offsetRg(offsetRg) {}

        void setPose(Eigen::Matrix4d pose, bool isGripperPose = false)
        {
            isGripperPose ? BBox::setPose(pose*offsetRg) : BBox::setPose(pose);
        }
    };

    struct GraspModel
    {
        Eigen::Matrix4d pose;
        Eigen::Matrix4d preGrasp;
        Eigen::Matrix4d postGrasp;
        std::vector<Finger> fingersClose;
        std::vector<Finger> fingersOpen;
        std::vector<Finger> fingerPads;
        BBox bboxGripper;
        Feature fs;
        Feature fg;

        Eigen::Matrix4d getPoseFromFeatures(bool save = false)
        {
            Eigen::Matrix4d pose = fs.ref * fg.ref.inverse();
            if (save)
            {
                setPose(pose);
            }
            return pose;
        }

        void setPose(Eigen::Matrix4d pose, double approxOverlap = 0.8)
        {
            this->pose = pose;
            bboxGripper.setPose(pose);

            for (auto& f : fingersClose)
            {
                f.setPose(pose, true);
            }
            for (auto& f : fingersOpen)
            {
                f.setPose(pose, true);
                f.setApproxSpheres(approxOverlap);
            }
            for (auto& f : fingerPads)
            {
                f.setPose(pose, true);
            }
        }

        void computePrePostGrasp(Eigen::Matrix4d Tpre, Eigen::Vector3d OffsetPost)
        {
            preGrasp = pose * Tpre;
            postGrasp = pose;
            postGrasp.block<3, 1>(0, 3) = postGrasp.block<3, 1>(0, 3) + OffsetPost;
        }

        void setTree(dxSearchTree tree)
        {
            bboxGripper.setTree(tree);

            for (auto& f : fingersOpen)
            {
                f.setTree(tree);
            }
            for (auto& f : fingerPads)
            {
                f.setTree(tree);
            }
        }

        Eigen::Map<Eigen::RowVectorXd> getColMajorVector(Eigen::Matrix4d M)
        {
            return Eigen::Map<Eigen::RowVectorXd>(M.data(), M.size());
        }

        void save(std::ostream& os, bool isHeader = false, bool isEndl = true)
        {
            if (isHeader)
            {
                os << "Pregrasp pose | Grasp pose | Postgrasp pose | probability (Matrix as Col vectors)" << endl;
                return;
            }
            os << getColMajorVector(preGrasp) << "|";
            os << getColMajorVector(pose) << "|";
            os << getColMajorVector(postGrasp) << "|";
            os << fs.prob;
            if (isEndl)
                os << endl;
        }
    };

    // PG70 Model
    //			   ____________
    //			  /		      /|
    //(dy)0.112	 /  		 / |
    //			/___________/  |
    //			|	  	    |  ________ * 0.016 (*: width)
    //			|	  	Y	| -------- |	    (x: Contact Location)
    //(dz) 0.08 | X <-o		| ______X_|	 0.03
    //			|	  |		| /  0.06
    //			|_____Z_____|/
    //			<----------->
    //				0.093
    //				  <---------->
    //					   dxf = 0.0765 //Position of the finger
    //				  <------------>
    //						 dx
    // dx = 0.093/2 + 0.06*0.7(=70%)
    //
    // Feature Frame
    //				X
    //			o -> Y
    //			|
    //			Z
    //Pad
    //			0.016
    // 0.024  ___/
    //			 | 0.03

    struct GraspModelPG70 : public GraspModel
    {
        double dx;
        double dxf;
        double opening;
        double openingMin;
        double openingMax;
        double frictionAngle;
        double collisionZmin;
        double collisionMaxRatio;
        int collisionNmaxPts;
        int NrotSampled;
        Eigen::Matrix4d TfeatureInRgripper;
        Eigen::Matrix4d TgripperInfeature;
        Eigen::Matrix4d TpreGraspInRgripper;
        Eigen::Vector3d OffsetPostGraspInRbase;
        std::array<double, 3> fingerxyz;
        std::array<double, 3> gripperxyz;

        GraspModelPG70()
        {
            opening = 0;
            openingMin = 0.001;
            openingMax = 0.05;
            frictionAngle = 45 * M_PI / 180.0;
            NrotSampled = 10;
            collisionMaxRatio = 0.0;
            collisionNmaxPts = 0;
            fingerxyz = { 0.06, 0.016, 0.03 };
            gripperxyz = { 0.093, 0.112, 0.08 };
            double contactLocRatio = 0.8;
            collisionZmin = 0.62;
            double dxPad = 2 * (1 - contactLocRatio)*fingerxyz[0];
            dxf = -(gripperxyz[0] + fingerxyz[0]) / 2.0;
            dx = -(gripperxyz[0] / 2.0 + fingerxyz[0] * contactLocRatio);

            TfeatureInRgripper << 0, -1, 0, dx,
                               1, 0, 0, 0,
                               0, 0, 1, 0,
                               0, 0, 0, 1;

            TpreGraspInRgripper << 1, 0, 0, 0.1,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;

            OffsetPostGraspInRbase << 0, 0, 0.2;

            bboxGripper = BBox(gripperxyz[0], gripperxyz[1], gripperxyz[2]);
            fingersClose.push_back(Finger(BBox(fingerxyz[0], fingerxyz[1], fingerxyz[2])));
            fingersClose.push_back(Finger(BBox(fingerxyz[0], fingerxyz[1], fingerxyz[2])));
            fingerPads.push_back(Finger(BBox(dxPad, fingerxyz[1], fingerxyz[2])));
            fingerPads.push_back(Finger(BBox(dxPad, fingerxyz[1], fingerxyz[2])));

            for (Finger f : fingersClose)
            {
                fingersOpen.push_back(f);
            }
            for (auto& f : fingerPads)
            {
                f.setUseMinMaxDim(true, true);
            }

            setFingersOffset(opening);
            TgripperInfeature = TfeatureInRgripper.inverse();
        }

        void setFingersOffset(double opening)
        {
            if (fingersOpen.size() != 2 || fingersClose.size() != 2 || fingerPads.size() != 2)
            {
                cout << "setFingersOffset() fingers != 2" << endl;
                throw runtime_error("setFingersOffset() fingers != 2");
            }

            fingersClose[0].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dxf, -(fingerxyz[1] + opening) / 2.0, 0)).matrix();
            fingersClose[1].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dxf, (fingerxyz[1] + opening) / 2.0, 0)).matrix();
            fingersOpen[0].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dxf, -(fingerxyz[1] + openingMax) / 2.0, 0)).matrix();
            fingersOpen[1].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dxf, (fingerxyz[1] + openingMax) / 2.0, 0)).matrix();
            fingerPads[0].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dx, -(fingerxyz[1] + opening) / 2.0, 0)).matrix();
            fingerPads[1].offsetRg = Eigen::Affine3d(Eigen::Translation3d(dx, (fingerxyz[1] + opening) / 2.0, 0)).matrix();
        }

        void setPose(Eigen::Matrix4d pose, double opening)
        {
            this->opening = opening;
            setFingersOffset(opening);
            GraspModel::setPose(pose);
        }

        void computePrePostGrasp()
        {
            GraspModel::computePrePostGrasp(TpreGraspInRgripper, OffsetPostGraspInRbase);
        }

        bool inCollision()
        {
            if (bboxGripper.M(2, 3) <= collisionZmin)
            {
                return true;
            }

            if (bboxGripper.inCollision(collisionMaxRatio))
            {
                return true;
            }

            for (auto f : fingersOpen)
            {
                if (f.getNPtsInBBoxApprox() > collisionNmaxPts)
                {
                    return true;
                }
            }

            return false;
        }

        void save(std::ostream& os, bool isHeader = false)
        {
            if (isHeader)
            {
                os << "Pregrasp pose | Grasp pose | Postgrasp pose | probability | gripper close | gripper open (Matrix as Col vectors)" << endl;
                return;
            }
            GraspModel::save(os, false, false);
            os << "|" << opening;
            os << "|" << openingMax;
            os << endl;
        }
    };

};

#endif // !DX_GRIPPER_MODEL_INCLUDE