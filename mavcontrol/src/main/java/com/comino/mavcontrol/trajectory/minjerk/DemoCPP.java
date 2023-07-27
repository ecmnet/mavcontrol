//package com.comino.mavcontrol.trajectory.minjerk;
//
//import org.bytedeco.MavRQT.Vec3;
//import org.bytedeco.javacpp.PointerScope;
//
//
//public class DemoCPP {
//
//	@SuppressWarnings("resource")
//	public static void main(String[] args) {
//
//		try (PointerScope scope = new PointerScope()) {	
//
//			Vec3 pos0 = new Vec3(0,0,0);
//			Vec3 vel0 = new Vec3(0,0,0);
//			Vec3 acc0 = new Vec3(0,0,0);
//
//			Vec3 gravity = new Vec3(0,0,-9.81);
//
//			Vec3 posf = new Vec3(Math.random(),Math.random(),Math.random() );
//			Vec3 velf = new Vec3(Math.random(),Math.random(),0 );
//			Vec3 accf = new Vec3(0,0,0);
//
//
//			org.bytedeco.MavRQT.RapidTrajectoryGenerator traj = new org.bytedeco.MavRQT.RapidTrajectoryGenerator(pos0, vel0, acc0, gravity);
//
//
//			double Tf = 1;
//
//			double fmin = 5;              //[m/s**2]
//			double fmax = 25;            //[m/s**2]
//			double wmax = 20;            //[rad/s]
//
//			double minTimeSec = 0.05;    //[s]
//
//
//			Vec3 floorPos  = new Vec3(0,0,0);
//			Vec3 floorNormal = new Vec3(0,0,1);
//
//			long tms = System.nanoTime();
//			for(int i=0; i < 100000;i++) {
//			    posf.x(Math.random());  posf.y(Math.random());  posf.z(Math.random()); 
//				velf.x(Math.random());  velf.y(Math.random());  velf.z(Math.random()); 
//				accf.x(0); accf.y(0); accf.z(0);
//				traj.SetGoalPosition(posf);
//				traj.SetGoalVelocity(velf);
//				traj.SetGoalAcceleration(accf);
//				traj.Generate(Tf);
//				traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
//				traj.CheckPositionFeasibility(floorPos, floorNormal);
//			}
//			System.out.println((System.nanoTime()-tms)/1000);
//
//
//			for(int i = 0; i < 3; i++) {
//				System.out.println(" Axis: "+i);
//				System.out.println("\talpha = "+traj.GetAxisParamAlpha(i));
//				System.out.println("\tbeta  = "+traj.GetAxisParamBeta(i));
//				System.out.println("\tgamma = "+traj.GetAxisParamGamma(i));
//				System.out.println();
//			}
//
//			System.out.println("Total cost = "+traj.GetCost());
//			System.out.println("Input feasible: "+ traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec));
//			System.out.println("Position feasible: "+traj.CheckPositionFeasibility(floorPos, floorNormal));
//
//
//		}
//
//
//	}
//
//}
