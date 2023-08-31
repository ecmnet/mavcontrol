package com.comino.mavcontrol.offboard4.optimizer.cost;

import org.apache.commons.math3.analysis.UnivariateFunction;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

import georegression.struct.GeoTuple3D_F64;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;

public abstract class AbstractTrajectoryCostFunctions implements UnivariateFunction {
	
	
	private static final GeoTuple4D_F32<?> acceleration = new Vector4D_F32(0,0,0,0);
	
	protected Offboard3Current current;
	protected GeoTuple4D_F32<?> goalPos;
	protected GeoTuple4D_F32<?> goalDir;
	protected GeoTuple4D_F32<?> velocity;
	protected double tf;
	
	private final GeoTuple4D_F32<?> pf = new Vector4D_F32();


	public void set(Offboard3Current current,final GeoTuple4D_F32<?> goalPos, final GeoTuple4D_F32<?> goalDir, final GeoTuple4D_F32<?> velocity, double tf) {
		this.current  = current;
		this.goalPos  = goalPos;
		this.goalDir  = goalDir;
		this.velocity = velocity;
		this.tf       = tf;
	}
	
	
	protected double getCost(final double value,final RapidTrajectoryGenerator traj, final double tf) {
		
		traj.getPosition(tf,pf);
		
		float est_distance_to_goal = MSP3DUtils.distance3D(goalPos, pf);
		
		float costs = est_distance_to_goal;
		
	//	System.out.println(" ---> Velocity "+value +" => Costs: "+ costs);
		
		return costs;
	}
	
   public RapidTrajectoryGenerator buildTrajectory(Offboard3Current current, GeoTuple4D_F32<?> velocity, double tf) {
	   
	   RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));
	   traj.setInitialState(current.pos(),current.sev(),current.acc()); 
	   traj.setGoalVelocity(velocity);
	   traj.setGoalAcceleration(acceleration);
	   traj.generate(tf);
	   return traj;
	   
   }

}
