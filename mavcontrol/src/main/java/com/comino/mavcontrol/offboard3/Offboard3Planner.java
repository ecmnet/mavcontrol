package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.exceptions.Offboard3CollisionException;
import com.comino.mavcontrol.offboard3.states.Offboard3CurrentState;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.offboard3.states.Offboard3TargetState;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;

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
	private final Offboard3CurrentState            current;
	
	// collsion check
	private final Offboard3CollisionCheck          collisionCheck;
	
	// DataModel
	private final DataModel                        model;

	// 
	private float acceptance_radius;
	private float max_xyz_velocity;


	public Offboard3Planner(IMAVController control, Offboard3CurrentState current, float acceptance_radius, float max_xyz_velocity) {
		
		this.current = current;  
		this.model   = control.getCurrentModel();
		this.max_xyz_velocity = max_xyz_velocity;
		this.acceptance_radius = acceptance_radius;
		
		this.collisionCheck = new Offboard3CollisionCheck(xyzPlanner);
		
	}

	public LinkedList<Offboard3TargetState> getFinalPlan() {
		return final_plan;
	}
	
	public void clear() {
		final_plan.clear();
	}
	
	public void planDirectYaw(float yaw) {
		final_plan.add(new Offboard3TargetState(new Point4D_F32(Float.NaN,Float.NaN,Float.NaN,yaw)));
	}
	
	public void planDirectPath(GeoTuple4D_F32<?> pos_target) {
		
		LinkedList<Offboard3TargetState> new_plan = new LinkedList<Offboard3TargetState>();

		current.update();

		float estimated_xyz_duration = MSP3DUtils.distance3D(pos_target, current.pos()) / max_xyz_velocity;

		if(estimated_xyz_duration < (5/max_xyz_velocity+2.0f)) {
			new_plan.add(new Offboard3TargetState(pos_target));
		}

		else {
			System.out.println("Planner: Estimated duration: "+estimated_xyz_duration);
			new_plan.add(new Offboard3TargetState(pos_target,current.pos(),max_xyz_velocity,2.0f));
			new_plan.add(new Offboard3TargetState(pos_target,current.pos(),max_xyz_velocity,estimated_xyz_duration*5f/8f));
			new_plan.add(new Offboard3TargetState(pos_target));
		}
		
		try {
			
			double costs = planPath(new_plan, current);
			
			
		} catch (Offboard3CollisionException e) {
			System.out.println("Plan not valid: "+estimated_xyz_duration);
			// TODO: Adjust plan and do replanning
		}
		
		final_plan.addAll(new_plan);
	}
	
	
	private double planPath(LinkedList<Offboard3TargetState> plan, Offboard3CurrentState initial_state) throws Offboard3CollisionException {
		
		Offboard3State nextPlannedCurrentState = initial_state; double total_costs = 0; float total_time = 0;
		Point3D_F64 obstacle = new Point3D_F64(model.slam.ox,model.slam.oy,model.slam.oz);
		
		for(Offboard3TargetState section : plan) {
			
			nextPlannedCurrentState = planSection(section, nextPlannedCurrentState);
			collisionCheck.check(obstacle, 0);
			
			total_costs += xyzPlanner.getCost();
			total_time  += xyzPlanner.getTotalTime();
			
		}
		
		System.out.println("Planner: Total costs of planned path: "+total_costs +" in "+total_time+" secs");
		
		// Plan is ok, return total costs of the plan
		return total_costs;
	}
	
	

	private Offboard3State planSection(Offboard3TargetState target, Offboard3State current_state)  {

		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0; 

			target.replaceNaNPositionBy(current_state.pos());

		// Yaw Planning 

		yawPlanner.reset();
		if(Float.isFinite(target.pos().w)) {

			target.pos().w = normAngle(target.pos().w );  
			current_state.pos().w = normAngle(current_state.pos().w);

			//        	System.err.println("1. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

			if((target.pos().w - current_state.pos().w) > (float)Math.PI) {
				target.pos().w  = (target.pos().w -(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
			}

			if((target.pos().w - current_state.pos().w) < -(float)Math.PI) {
				target.pos().w  = (target.pos().w +(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
			}

			//			System.err.println("2. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

			yawPlanner.setInitialState(current_state.pos().w, current_state.vel().w, 0);
			yawPlanner.setTargetState(target.pos().w , 0, 0);

			if(target.getDuration() < 0)
				estimated_yaw_duration = Math.abs(target.pos().w  - current_state.pos().w)/MAX_YAW_VEL;
			else
				estimated_yaw_duration = target.getDuration();

			if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
				if(estimated_yaw_duration < 3)
					estimated_yaw_duration = 3;
				yawPlanner.generateTrajectory(estimated_yaw_duration);
				System.out.println("\tYaw (Planner): "+MSPMathUtils.fromRad(target.pos().w )+" in "+estimated_yaw_duration+" secs");
			} 
			
			current_state.pos().w = (float)yawPlanner.getGoalPosition();
			current_state.vel().w = (float)yawPlanner.getGoalVelocity();
			current_state.acc().w = 0;

		} 


		// XYZ planning

		xyzPlanner.reset();
		if(target.isPositionFinite() || isValid(target.vel()) && 
				!target.isPosReached(current_state.pos(),acceptance_radius,Float.NaN)) {

			xyzPlanner.setInitialState(current_state.pos(),current_state.vel(),current_state.acc());

			if(isValid(target.vel()))
				xyzPlanner.setGoal(null, target.vel(), target.acc());
			else
				xyzPlanner.setGoal(target.pos(), target.vel(), target.acc());

			if(target.getDuration() < 0) {
				estimated_xyz_duration = MSP3DUtils.distance3D(target.pos(), current_state.pos()) * 2.0f / max_xyz_velocity;

				if(estimated_xyz_duration < estimated_yaw_duration)
					estimated_xyz_duration = estimated_yaw_duration;

			}
			else
				estimated_xyz_duration = target.getDuration();

			if(isValid(target.vel())) {
				xyzPlanner.generate(estimated_xyz_duration);
				System.out.println("\tXYZ Velocity  (Planner): "+target+" (" +MSP3DUtils.distance3D(target.pos(), current_state.pos()) +") in "+estimated_xyz_duration+" secs");

			}
			else {
				if(estimated_xyz_duration < 2)
					estimated_xyz_duration = 2f;
				xyzPlanner.generate(estimated_xyz_duration);
				System.out.println("\tXYZ Position  (Planner): "+target+" ("+MSP3DUtils.distance3D(target.pos(), current_state.pos())+") in "+estimated_xyz_duration+" secs");
			}
			
			xyzPlanner.getGoalPosition(current_state.pos());
			xyzPlanner.getGoalVelocity(current_state.vel());
			xyzPlanner.getGoalAcceleration(current_state.acc());
		}
		
		

		return current_state;

	}

	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}
	
	private boolean isValid(GeoTuple4D_F32<?> p) {
		return p.x != 0 || p.y != 0 || p.z != 0;
	}




}
