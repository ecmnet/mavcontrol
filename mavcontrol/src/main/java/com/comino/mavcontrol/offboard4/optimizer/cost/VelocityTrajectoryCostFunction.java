package com.comino.mavcontrol.offboard4.optimizer.cost;

import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class VelocityTrajectoryCostFunction extends AbstractTrajectoryCostFunctions {
	
	protected GeoTuple4D_F32<?> velocity = new Vector4D_F32( );

	@Override
	public double value(double x) {
		
		velocity.setTo(this.goalDir.x, this.goalDir.y, this.goalDir.z,0);
		velocity.timesIP((float)x);
		
		RapidTrajectoryGenerator traj = buildTrajectory(this.current, this.velocity,tf);
		return getCost(x,traj,tf);
	}

}
