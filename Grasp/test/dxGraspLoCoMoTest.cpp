/**
 * @file
 * @brief Source file for dxGraspLoCoMoTest
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

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include "dxGraspLoCoMo.h"
#include "dxPointCloud.h"


using namespace std;

void printMsg(string message, string prep = "")
{
	std::cout << prep << message << endl;
}

// This demo uses the Schunk PG70 gripper.
// The parameters of the gripper are described below.
// All the result poses are given using the gripper frame
// If you use a different gripper you can set the parameters
// of your gripper in the file dxGripperModel.h (line 669)
//
// Note: The gripper collision model relies on kdtree, and uses the largest sphere
// fitting in the gripper box. It may not work well if the proportions of your gripper
// are not comparable. Future releases will fix this problem.
//
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

int main()
{
	printMsg("LoCoMo Grasping -------- ");

	dxPointCloud cloud;
	dxGraspLoCoMo grasp;

	//IMPORTANT 
	//downsampling: resolution of the point cloud
	//resolutionFactor: Used to compute the LoCoMo sphere radius
	grasp.setResolution(0.008);

	string cloudPath = "../../Clouds/wood_cloud.txt";
	printMsg("Loading: " + cloudPath);

	//Loading the point cloud
	cloud.loadFromFile(cloudPath);

	//Computing the grasps
	printMsg("Computing grasps...");
	dxGraspLoCoMo::GraspPG70Vec graspResults = grasp.locomoGrasp(cloud).grasps;
	if (!graspResults.size())
		cout << "No grasps found" << endl;

	//Display the 10 highest ranked grasps
	int Ngrasps = 10;
	cout << "pre-grasp pose | grasp pose | post-grasp pose | gripper opening | score" << endl;
	for (int i = 0; i < min(Ngrasps, static_cast<int>(graspResults.size())); i++) {	
		dxGraspLoCoMo::GraspPG70 g = graspResults[i];

		cout << "Grasp #" << i << endl;
		cout << g.getColMajorVector(g.preGrasp) << "|";
		cout << g.getColMajorVector(g.pose) << "|";
		cout << g.getColMajorVector(g.postGrasp) << "|";
		cout << g.opening << " | ";
		cout << g.fs.prob << endl << endl;
	}

	//Save grasps to file
	//grasp.saveGrasps("graps_results.txt");

	return 0;
}
