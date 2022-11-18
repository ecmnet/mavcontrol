package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.exceptions.Offboard3CollisionException;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosVelTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3VelTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3YawTarget;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;

public class Offboard3Planner {

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float MIN_XYZ_ESTIMATED_TIME           = 5f;                       // Minumum duration the planner ist used in [s]

	// Planners
	private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
	private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

	// List of final targets
	private final Offboard3Plan<Offboard3AbstractTarget> final_plan = new Offboard3Plan<Offboard3AbstractTarget>();

	// Current state
	private final Offboard3Current            current;

	// Collsion check
	private final Offboard3CollisionCheck     collisionCheck;

	// DataModel && Controller
	private final DataModel                    model;
	private final IMAVController               control;


	// Planning parameters
	private float acceptance_radius;
	private float max_xyz_velocity;


	public Offboard3Planner(IMAVController control, float acceptance_radius, float max_xyz_velocity) {

		this.current = new Offboard3Current(control.getCurrentModel());  
		this.control = control;
		this.model   = control.getCurrentModel();
		this.max_xyz_velocity = max_xyz_velocity;
		this.acceptance_radius = acceptance_radius;

		this.collisionCheck = new Offboard3CollisionCheck(xyzPlanner);

	}

	public LinkedList<Offboard3AbstractTarget> getFinalPlan() {
		return final_plan;
	}

	public void reset() {
		final_plan.clear();
		yawPlanner.reset();
		xyzPlanner.reset();
		current.update();
	}

	public void planDirectYaw(float yaw) {
		reset();
		final_plan.add(new Offboard3YawTarget(yaw));
	}

	public void planDirectPath(GeoTuple4D_F32<?> pos_target) {

		Offboard3Plan<Offboard3AbstractTarget> new_plan = new Offboard3Plan<Offboard3AbstractTarget>();

		reset(); current.update();

		new_plan.setEstimatedTime(MSP3DUtils.distance3D(pos_target, current.pos()) / max_xyz_velocity);

		if(new_plan.getEstimatedTime() < (5/max_xyz_velocity+2.0f)) {
			new_plan.add(new Offboard3PosTarget(pos_target));
		}

		else {
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,2.0f));
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,new_plan.getEstimatedTime()*5.5f/8f));
			new_plan.add(new Offboard3PosTarget(pos_target));
		}

		//		if(new_plan.size() > 1 && control.isSimulation()) {
		//			
		//	    // Test split middle segment into 3 parts
		//		float split = new_plan.get(1).getDuration();
		//		
		//		GeoTuple4D_F32<?> test = pos_target.copy();
		//		
		////		test.x = 1f * Math.signum(test.x);
		////		test.y = 1f * Math.signum(test.y);
		//		
		//		new_plan.replaceWith(1, 
		//        		new Offboard3VelTarget(pos_target,max_xyz_velocity,split/3f),
		//        		new Offboard3VelTarget(test,max_xyz_velocity,split/2.5f),
		//        		new Offboard3VelTarget(pos_target,max_xyz_velocity,split/3f)
		//        		);
		//		}

		try {

			planPath(new_plan, current);


		} catch (Offboard3CollisionException col) {

			System.err.println("WARNING: Collsion expected");



			//			 System.err.println("Collsion expected. Re-planning....");
			//			
			//			
			//			Offboard3TargetState critical_target = new_plan.get(col.getPlanningSectionIndex());
			//			
			//			pos_target.x = pos_target.x + 5;
			//    
			//            new_plan.replaceWith(col.getPlanningSectionIndex(), 
			//            		new Offboard3TargetState(pos_target,current.pos(),max_xyz_velocity,critical_target.getPlannedSectionTime()/2f),
			//            		new Offboard3TargetState(pos_target,current.pos(),max_xyz_velocity,critical_target.getPlannedSectionTime()/2f)
			//            		);
			//            
			//            try {
			//				planPath(new_plan, current);
			//			} catch (Offboard3CollisionException e) {
			//				 System.out.println("Collsion expected. Re-planning failed.");
			//				 System.err.println(new_plan);
			//				 return;
			//			}
		}

		MSPStringUtils.getInstance().out(new_plan);
		final_plan.addAll(new_plan);
	}


	private void planPath(Offboard3Plan<Offboard3AbstractTarget> plan, Offboard3Current initial_state) throws Offboard3CollisionException {

		Offboard3State nextPlannedCurrentState = initial_state; 
		Point3D_F64 obstacle = new Point3D_F64(model.slam.ox,model.slam.oy,model.slam.oz);

		int index = 0;
		// Plan sections
		for(Offboard3AbstractTarget section : plan) {
			nextPlannedCurrentState = planSection(section, nextPlannedCurrentState);
			section.setIndex(++index);
			plan.addCostsAndTime((float)xyzPlanner.getCost(), xyzPlanner.getTotalTime());
		}
		
		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP))
			return;

		// Check collision
		for(int sections = 0; sections < plan.size();sections++) {
			collisionCheck.check(obstacle, 0, sections);
		}	
	}



	private Offboard3State planSection(Offboard3AbstractTarget target, Offboard3State current_state)  {

		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0; 
		float planned_xyz_duration = 0; float planned_yaw_duration = 0; 
		
		final Offboard3Current new_current_state = new Offboard3Current();

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
				estimated_yaw_duration = Math.abs(target.pos().w  - current_state.pos().w) * 2.0f /MAX_YAW_VEL;
			else
				estimated_yaw_duration = target.getDuration();

			if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
				if(estimated_yaw_duration < 3)
					estimated_yaw_duration = 3;
				planned_yaw_duration = yawPlanner.generateTrajectory(estimated_yaw_duration);
				//System.out.println("\tYaw (Planner): "+MSPMathUtils.fromRad(target.pos().w )+" in "+estimated_yaw_duration+" secs");
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

			switch(target.getType()) {
			case Offboard3AbstractTarget.TYPE_POS:
				xyzPlanner.setGoal(target.pos(), target.vel(), target.acc());
				break;
			case Offboard3AbstractTarget.TYPE_POS_VEL:
				target.determineTargetVelocity(current_state.pos());
				xyzPlanner.setGoal(target.pos(), target.vel(), target.acc());
				break;
			case Offboard3AbstractTarget.TYPE_VEL:
				target.determineTargetVelocity(current_state.pos());
				xyzPlanner.setGoal(null, target.vel(), target.acc());
				break;
			}

			if(target.getDuration() < 0) {
				estimated_xyz_duration = MSP3DUtils.distance3D(target.pos(), current_state.pos()) * 2.0f / max_xyz_velocity;
				
				if(estimated_xyz_duration < MIN_XYZ_ESTIMATED_TIME)
					estimated_xyz_duration = MIN_XYZ_ESTIMATED_TIME;
				
				if(estimated_xyz_duration < estimated_yaw_duration)
					estimated_xyz_duration = estimated_yaw_duration;

			}
			else
				estimated_xyz_duration = target.getDuration();

			if(isValid(target.vel())) {
				planned_xyz_duration = xyzPlanner.generate(estimated_xyz_duration);
				//System.out.println("\tXYZ Velocity  (Planner): "+target+" (" +MSP3DUtils.distance3D(target.pos(), current_state.pos()) +") in "+estimated_xyz_duration+" secs");

			}
			else {
				if(estimated_xyz_duration < 2)
					estimated_xyz_duration = 2f;
				planned_xyz_duration = xyzPlanner.generate(estimated_xyz_duration);
				//System.out.println("\tXYZ Position  (Planner): "+target+" ("+MSP3DUtils.distance3D(target.pos(), current_state.pos())+") in "+estimated_xyz_duration+" secs");
			}
			

			target.setPlannedSectionTime(planned_xyz_duration > planned_yaw_duration ? planned_xyz_duration : planned_yaw_duration);

			
			// Update next planned current_state with estimated end state of section
			xyzPlanner.getGoalPosition(new_current_state.pos());
			xyzPlanner.getGoalVelocity(new_current_state.vel());
			xyzPlanner.getGoalAcceleration(new_current_state.acc());
		}



		return new_current_state;

	}

	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}

	private boolean isValid(GeoTuple4D_F32<?> p) {
		return p.x != 0 || p.y != 0 || p.z != 0;
	}




}
