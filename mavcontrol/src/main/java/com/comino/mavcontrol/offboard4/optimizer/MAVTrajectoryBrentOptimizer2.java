package com.comino.mavcontrol.offboard4.optimizer;

import org.apache.commons.math3.optimization.GoalType;
import org.apache.commons.math3.optimization.univariate.BrentOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariatePointValuePair;

import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard4.optimizer.cost.VelocityTrajectoryCostFunction;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.esdf.D2.DynamicESDF2D;

import georegression.struct.GeoTuple4D_F32;

@SuppressWarnings("deprecation")
public class MAVTrajectoryBrentOptimizer2 {
	
	private final VelocityTrajectoryCostFunction velocity_costs;
	private final BrentOptimizer brent;

	public MAVTrajectoryBrentOptimizer2(DynamicESDF2D edf) {
		
		this.brent     = new BrentOptimizer(1e-5, 1e-6);
		this.velocity_costs = new VelocityTrajectoryCostFunction( );
	   
	}
	
	public RapidTrajectoryGenerator optimize(final Offboard3Current current,
			                                 final GeoTuple4D_F32<?> velocity, 
			                                 final GeoTuple4D_F32<?> target, 
			                                 final GeoTuple4D_F32<?> dir, 
			                                 double tf) {
		current.vel().w = 0;
		float vel0 = current.vel().norm();
		velocity_costs.set(current, target, dir, null, tf);
		UnivariatePointValuePair r1 = brent.optimize(1000, velocity_costs , GoalType.MINIMIZE, Math.max(vel0 - 4/tf, 0.1), Math.min(1.0, vel0 + .3/tf ) );
		
		System.out.println("Optimized speed: "+r1.getPoint()+" with "+brent.getEvaluations()+" Iterations");
		
		velocity.setTo(dir.x, dir.y, dir.z,0);
		velocity.timesIP((float)r1.getPoint());
		
		return velocity_costs.buildTrajectory(current,velocity,tf);
		
	}
	
}
