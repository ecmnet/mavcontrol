/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * This is a port to Java from minimum_jerkt_trajectories in
 * https://zenodo.org/record/5517791#.YW_kBS-21B1
 * 
 * The algorithm is described in the following paper: 
 * M.W. Mueller, M. Hehn, and R. D'Andrea, "A computationally efficient motion 
 * primitive for quadrocopter trajectory generation," 
 * IEEE Transactions on Robotics Volume 31, no.8, pages 1294-1310, 2015.
 * 
 * The paper may be downloaded from 
 * http://muellerlab.berkeley.edu/publications/
 *
 ****************************************************************************/

package com.comino.mavcontrol.trajectory.minjerk;

import georegression.struct.point.Point3D_F64;

public class demo {

	public static void main(String[] args) {

		//Define the trajectory starting state:
		Point3D_F64 pos0 = new Point3D_F64(0,0,0);
		Point3D_F64 vel0 = new Point3D_F64(0,0,0);
		Point3D_F64 acc0 = new Point3D_F64(0,0,0);

		//Define how gravity lies in our coordinate system
		Point3D_F64 gravity = new Point3D_F64(0,0,-9.81);


		RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator(pos0, vel0, acc0, gravity);

		//define the goal state:
		Point3D_F64 posf = new Point3D_F64(Math.random(),Math.random(),Math.random() );
		Point3D_F64 velf = new Point3D_F64(Math.random(),Math.random(),0);
		Point3D_F64 accf = new Point3D_F64(0,0,0);

		Point3D_F64 tmp = new Point3D_F64(0,0,0);

		//define the duration:
		double Tf = 1;

		double fmin = 5;              //[m/s**2]
		double fmax = 25;            //[m/s**2]
		double wmax = 20;            //[rad/s]

		double minTimeSec = 0.05;    //[s]


		//Define the state constraints. We'll only check that we don't fly into the floor:
		Point3D_F64 floorPos    = new Point3D_F64(0,0,0);   //any point on the boundary
		Point3D_F64 floorNormal = new Point3D_F64(0,0,1);   //we want to be in this direction of the boundary

		long tms = System.nanoTime();
		for(int i=0; i < 100000;i++) {
			
			posf.setTo(Math.random(),Math.random(),Math.random() );
			velf.setTo(Math.random(),Math.random(),0);
			accf.setTo(0,0,0 );
			
			traj.setGoalPosition(posf);
			traj.setGoalVelocity(velf);
			traj.setGoalAcceleration(accf);
			
			traj.generate(Tf);
			traj.checkInputFeasibility(fmin,fmax,wmax,minTimeSec);
			traj.checkPositionFeasibility(floorPos, floorNormal);
		}
		System.out.println((System.nanoTime()-tms)/1000);


		for(int i = 0; i < 3; i++) {
			System.out.println(" Axis: "+i);
			System.out.println("\talpha = "+traj.getAxisParamAlpha(i));
			System.out.println("\tbeta  = "+traj.getAxisParamBeta(i));
			System.out.println("\tgamma = "+traj.getAxisParamGamma(i));
			System.out.println();
		}

		System.out.println("Total cost = "+traj.getCost());
		System.out.println("Input feasible: "+ traj.checkInputFeasibility(fmin,fmax,wmax,minTimeSec));
		System.out.println("Position feasible: "+traj.checkPositionFeasibility(floorPos, floorNormal));

//		System.out.println(); int i=0;
//		for(double time_s =0; time_s < 1; time_s = time_s + Tf/100) {
//			traj.getPosition(time_s, tmp);
//			traj.getAcceleration(time_s, tmp);
//			System.out.println((++i)+": "+tmp);
//
//		} 

	}

}
