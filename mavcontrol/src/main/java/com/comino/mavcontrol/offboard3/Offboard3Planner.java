package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3CurrentState;
import com.comino.mavcontrol.offboard3.states.Offboard3TargetState;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;

public class Offboard3Planner {
	
	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float DEFAULT_RADIUS_ACCEPT            = 0.3f;                     // Acceptance radius in [m]
	private static final float DEFAULT_MAX_XYZ_VEL              = 1.5f;                     // Maxumum speed in [m/s]

	// Planners
	private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
	private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

	// List of final targets
	private final LinkedList<Offboard3TargetState> final_plan = new LinkedList<Offboard3TargetState>();

	// Current state
	private final Offboard3CurrentState current;

	// 
	private float acceptance_radius;
	private float max_xyz_velocity;


	public Offboard3Planner(Offboard3CurrentState current) {
		this.current = current;   
		
		MSPConfig config	= MSPConfig.getInstance();

		max_xyz_velocity = config.getFloatProperty(MSPParams.AUTOPILOT_MAX_XYZ_VEL, String.valueOf(DEFAULT_MAX_XYZ_VEL));
		System.out.println("Maximum planning velocity: "+max_xyz_velocity+" m/s");
		acceptance_radius = config.getFloatProperty(MSPParams.AUTOPILOT_RADIUS_ACCEPT, String.valueOf(DEFAULT_RADIUS_ACCEPT));
		System.out.println("Acceptance radius: "+acceptance_radius+" m");
	}

	public LinkedList<?> getFinalPlan() {
		return final_plan;
	}
	
	

	private Offboard3TargetState planTarget(Offboard3TargetState target, Offboard3CurrentState current_state) {

		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0;

		if(MSP3DUtils.isFinite(current.sep())) {
			target.replaceNaNPositionBy(current_state.sep());
			target.setTargetIsSetpoint(true);
		}
		else
			target.replaceNaNPositionBy(current_state.pos());

		// Yaw Planning 

		yawPlanner.reset();
		if(Float.isFinite(target.getTargetPosition().w)) {

			target.getTargetPosition().w = normAngle(target.getTargetPosition().w );  
			current_state.pos().w = normAngle(current_state.pos().w);

			//        	System.err.println("1. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

			if((target.getTargetPosition().w - current_state.pos().w) > (float)Math.PI) {
				target.getTargetPosition().w  = (target.getTargetPosition().w -(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
			}

			if((target.getTargetPosition().w - current_state.pos().w) < -(float)Math.PI) {
				target.getTargetPosition().w  = (target.getTargetPosition().w +(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
			}

			//			System.err.println("2. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

			yawPlanner.setInitialState(current_state.pos().w, current_state.vel().w, 0);
			yawPlanner.setTargetState(target.getTargetPosition().w , 0, 0);

			if(target.getDuration() < 0)
				estimated_yaw_duration = Math.abs(target.getTargetPosition().w  - current_state.pos().w)/MAX_YAW_VEL;
			else
				estimated_yaw_duration = target.getDuration();

			if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
				if(estimated_yaw_duration < 3)
					estimated_yaw_duration = 3;
				yawPlanner.generateTrajectory(estimated_yaw_duration);
				System.out.println("\tYaw: "+MSPMathUtils.fromRad(target.getTargetPosition().w )+" in "+estimated_yaw_duration+" secs");
			} 

		} 


		// XYZ planning

		xyzPlanner.reset();
		if(target.isPositionFinite() || isValid(target.getTargetVelocity()) && 
				!target.isPosReached(current_state.pos(),acceptance_radius,Float.NaN)) {

			xyzPlanner.setInitialState(current_state.pos(),current_state.vel(),current_state.acc());

			if(isValid(target.getTargetVelocity()))
				xyzPlanner.setGoal(null, target.getTargetVelocity(), target.getTargetAcceleration());
			else
				xyzPlanner.setGoal(target.getTargetPosition(), target.getTargetVelocity(), target.getTargetAcceleration());

			if(target.getDuration() < 0) {
				estimated_xyz_duration = MSP3DUtils.distance3D(target.getTargetPosition(), current_state.pos()) * 2.0f / max_xyz_velocity;

				if(estimated_xyz_duration < estimated_yaw_duration)
					estimated_xyz_duration = estimated_yaw_duration;

			}
			else
				estimated_xyz_duration = target.getDuration();

			if(isValid(target.getTargetVelocity())) {
				xyzPlanner.generate(estimated_xyz_duration);
				System.out.println("\tXYZ Velocity: "+target+" (" +MSP3DUtils.distance3D(target.getTargetPosition(), current_state.pos()) +") in "+estimated_xyz_duration+" secs");

			}
			else {
				if(estimated_xyz_duration < 2)
					estimated_xyz_duration = 2f;
				xyzPlanner.generate(estimated_xyz_duration);
				System.out.println("\tXYZ Position: "+target+" ("+MSP3DUtils.distance3D(target.getTargetPosition(), current_state.pos())+") in "+estimated_xyz_duration+" secs");
			}
		}

		return target;

	}

	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}
	
	private boolean isValid(GeoTuple4D_F32<?> p) {
		return (p.x != 0 || p.y != 0 ) && p.z != 0;
	}




}
