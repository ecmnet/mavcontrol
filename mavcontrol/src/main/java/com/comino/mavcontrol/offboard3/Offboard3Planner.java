package com.comino.mavcontrol.offboard3;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.states.Offboard3CurrentState;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;

import georegression.struct.point.Point3D_F64;

public class Offboard3Planner {

	// Planners
	private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
	private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));
	
	// Current state
	private final Offboard3CurrentState current;
	
	
	public Offboard3Planner(Offboard3CurrentState current) {
		
        this.current = current;   
		
		
	}

}
