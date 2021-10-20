package com.comino.mavcontrol.trajectory;

import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

import georegression.struct.point.Point3D_F64;

public class TrajectoryTests {

	private static final double MAX = 0.001;

	public RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator();

	//Define the trajectory starting state:
	Point3D_F64 pos0 = new Point3D_F64(0,0,2);
	Point3D_F64 vel0 = new Point3D_F64(0,0,0);
	Point3D_F64 acc0 = new Point3D_F64(0,0,0);

	//define the goal state:
	Point3D_F64 posf = new Point3D_F64(0,0,0);
	Point3D_F64 velf = new Point3D_F64(0,0,0);
	Point3D_F64 accf = new Point3D_F64(0,0,0);

	//define the duration:
	double Tf = 1.3;

	double fmin = 5;              //[m/s**2]
	double fmax = 25;            //[m/s**2]
	double wmax = 20;            //[rad/s]

	double minTimeSec = 0.02;    //[s]

	//Define how gravity lies in our coordinate system
	Point3D_F64 gravity = new Point3D_F64(0,0,-9.81);

	//Define the state constraints. We'll only check that we don't fly into the floor:
	Point3D_F64 floorPos    = new Point3D_F64(0,0,0);   //any point on the boundary
	Point3D_F64 floorNormal = new Point3D_F64(0,0,1);   //we want to be in this direction of the boundary


	boolean eInputFeasibility = false;
	boolean ePositionFeasibility = false;

	double  eCosts = 0;

	Point3D_F64 eAlpha = new Point3D_F64(0,0,0);
	Point3D_F64 eBeta  = new Point3D_F64(0,0,0);
	Point3D_F64 eGamma = new Point3D_F64(0,0,0);

	public boolean run() {

		setTestData1(); if(!performTest(1)) return false;
		setTestData1(); if(!performTest(2)) return false;

		setTestData2(); if(!performTest(3)) return false;
		setTestData3(); if(!performTest(4)) return false;

		return true;
	}

	public boolean performTest(int j) {

		System.out.println("Test step "+j);
		traj.setInitialState(pos0, vel0, acc0, gravity);
		traj.setGoalPosition(posf);
		traj.setGoalVelocity(velf);
		traj.setGoalAcceleration(accf);

		traj.generate(Tf);

		for(int i = 0; i < 3; i++) {
			if(Math.abs(eAlpha.getIdx(i) - traj.getAxisParamAlpha(i)) > MAX)
				return false;
			if(Math.abs(eBeta.getIdx(i) - traj.getAxisParamBeta(i)) > MAX)
				return false;
			if(Math.abs(eGamma.getIdx(i) - traj.getAxisParamGamma(i)) > MAX)
				return false;
		}
		
		if(Math.abs(eCosts - traj.getCost()) > MAX)
			return false;

		if(eInputFeasibility != traj.checkInputFeasibility(fmin,fmax,wmax,minTimeSec))
			return false;
		if(ePositionFeasibility != traj.checkPositionFeasibility(floorPos, floorNormal))
			return false;
		return true;
	}

	public void setTestData1() {

		// Input
		posf.setTo(1,0,5);
		velf.setTo(0,0,1);
		accf.setTo(0,0,0);

		// Expected results
		eInputFeasibility = false;
		ePositionFeasibility = true;

		eAlpha.setTo(193.91693352689109, 0, 455.7047937881941 );
		eBeta.setTo (-126.0460067924792, 0,-301.6701095900002 );
		eGamma.setTo(27.309968138370497, 0, 67.72872098315884 );

		eCosts = 977.1424558129265;

	}

	public void setTestData2() {

		// Input
		posf.setTo(1,0,1);
		velf.setTo(0,0,1);
		accf.setTo(0,0,0);

		// Expected results
		eInputFeasibility    = true;
		ePositionFeasibility = true;

		eAlpha.setTo(193.91693352689109, 0,-319.9629403193703 );
		eBeta.setTo (-126.0460067924792, 0, 202.5139175799166 );
		eGamma.setTo(27.309968138370497, 0,-41.51115157032316 );

		eCosts = 559.4752143704109;

	}
	
	public void setTestData3() {

		// Input
		posf.setTo(1,0,-1);
		velf.setTo(0,0,1);
		accf.setTo(0,0,0);

		// Expected results
		eInputFeasibility    = false;
		ePositionFeasibility = false;

		eAlpha.setTo(193.91693352689109, 0,-707.7968073731524 );
		eBeta.setTo (-126.0460067924792, 0, 454.605931164875  );
		eGamma.setTo(27.309968138370497, 0,-96.13108784706415 );

		eCosts = 2140.6440569743036;

	}


}
