package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.control.impl.MAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
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
import georegression.struct.point.Point3D_F32;
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
	private final Offboard3Plan final_plan = new Offboard3Plan();

	// Current state
	private final Offboard3Current            current;

	// Collsion check
	private final Offboard3CollisionCheck     collisionCheck;

	// DataModel && Controller
	private final DataModel                    model;
	private final IMAVController               control;

	// Admins
	private int plannedSectionCount = 0;

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

	public Offboard3Plan getFinalPlan() {
		return final_plan;
	}

	public int getPlannedSectionCount() {
		return plannedSectionCount;
	}

	public void reset() {

		model.slam.ix = Float.NaN;
		model.slam.iy = Float.NaN;
		model.slam.iz = Float.NaN;

		plannedSectionCount = 0;
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
		planDirectPath(pos_target,false);
	}

	public void planDirectPath(GeoTuple4D_F32<?> pos_target, boolean replanning) {

		long tms = System.nanoTime();

		Offboard3Plan new_plan = new Offboard3Plan();

		reset(); current.update();

		new_plan.setEstimatedTime(MSP3DUtils.distance3D(pos_target, current.pos()) / max_xyz_velocity);

		if(new_plan.getEstimatedTime() < (5/max_xyz_velocity+2.0f)) {
			new_plan.add(new Offboard3PosTarget(pos_target));
		}

		else {
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,2.0f));
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,new_plan.getEstimatedTime()*5f/8f));
			new_plan.add(new Offboard3PosTarget(pos_target));
		}



		try {

			planPath(new_plan, current);


		} catch (Offboard3CollisionException col) {

			if(!control.isSimulation() || replanning) 
				return;

			MSPStringUtils.getInstance().out("Replanning");

			MSPStringUtils.getInstance().out("Old plan");
			MSPStringUtils.getInstance().out(new_plan);

			Point3D_F32 obstacle           = new Point3D_F32(model.slam.ox,model.slam.oy,model.slam.oz);

			float time = new_plan.getTotalTimeUpTo(col.getPlanningSectionIndex()) + col.getExpectedTimeOfCollision();
			float velocity = MSP3DUtils.norm3D(col.getExpectedStateAtCollision().vel());

			Offboard3AbstractTarget target = new Offboard3PosVelTarget(velocity,time);
			generateAvoidanceTarget(col.getExpectedStateAtCollision(),obstacle, target, true);
			new_plan = doReplanning(new_plan, target,col.getPlanningSectionIndex());

			if(new_plan.isEmpty()) {
				control.writeLogMessage(new LogMessage("[msp] Replanning found no solution.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			}

			model.slam.setInfoPoint(target.pos());
			MSPStringUtils.getInstance().out("New plan");
		}

		final_plan.set(new_plan);
		MSPStringUtils.getInstance().out(final_plan);
		MSPStringUtils.getInstance().out("Planning time: "+(System.nanoTime()-tms)/1000+"us");
	}


	private Offboard3Plan doReplanning(Offboard3Plan plan, Offboard3AbstractTarget new_target, int replace_at_index) {

		control.writeLogMessage(new LogMessage("[msp] Replanning performed.", MAV_SEVERITY.MAV_SEVERITY_WARNING));

		Offboard3Plan new_plan = new Offboard3Plan();

		// TODO: Eventually split path to new_target
	
		new_plan.add(new_target);
		float time = (plan.getTotalTime() - new_target.getDuration())/3.0f;
		if(time > 2.0)
			new_plan.add(new Offboard3VelTarget(plan.getFirst().pos(),max_xyz_velocity,time));
		new_plan.add(new Offboard3PosTarget(plan.getFirst().pos()));

		// TODO: Try path left and right side to the obstacle: Choose the one with lower costs
		try {
			planPath(new_plan, current);
		} catch (Offboard3CollisionException e) {
			new_plan.clear();
		}

		return new_plan;	
	}

	private void planPath(Offboard3Plan plan, Offboard3Current initial_state) throws Offboard3CollisionException {

		Offboard3State nextPlannedCurrentState = initial_state; 

		Point3D_F32 obstacle = new Point3D_F32(model.slam.ox,model.slam.oy,model.slam.oz);

		plannedSectionCount = 0;

		// Plan sections
		for(Offboard3AbstractTarget section : plan) {

			section.setIndex(plannedSectionCount++);

			nextPlannedCurrentState = planSection(section, nextPlannedCurrentState);
			plan.addCostsAndTime((float)xyzPlanner.getCost(), xyzPlanner.getTotalTime());

			//	if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP))
			collisionCheck.check(obstacle, 0, section.getIndex());
		}
	}

	private Offboard3State planSection(Offboard3AbstractTarget target, Offboard3State current_state)  {

		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0; 
		float planned_xyz_duration = 0; float planned_yaw_duration = 0; 

		final Offboard3Current new_current_state = new Offboard3Current();

		target.replaceNaNPositionBy(current_state.pos());

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
			}
			else {
				if(estimated_xyz_duration < 2)
					estimated_xyz_duration = 2f;
				planned_xyz_duration = xyzPlanner.generate(estimated_xyz_duration);
			}


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
				} 

				current_state.pos().w = (float)yawPlanner.getGoalPosition();
				current_state.vel().w = (float)yawPlanner.getGoalVelocity();
			} 


			target.setPlannedSectionTime(planned_xyz_duration > planned_yaw_duration ? planned_xyz_duration : planned_yaw_duration);


			// Update next planned current_state with estimated end state of section
			xyzPlanner.getGoalPosition(new_current_state.pos());
			xyzPlanner.getGoalVelocity(new_current_state.vel());
			xyzPlanner.getGoalAcceleration(new_current_state.acc());

		}



		return new_current_state;

	}

	private void generateAvoidanceTarget(Offboard3State state_of_collision,Point3D_F32 obstacle, Offboard3AbstractTarget target, boolean right ) {

		final float       DISTANCE = 0.6f;

		float dx = state_of_collision.vel().x;
		float dy = state_of_collision.vel().y;

		float scale = (float)Math.sqrt(dx * dx + dy *dy);
		dx /= scale;
		dy /= scale;

		if(right) {
			if (Math.abs(dx) < (float)Math.abs(dy))
				target.pos().setTo(dy * DISTANCE, -dx * DISTANCE,Float.NaN,Float.NaN);
			else
				target.pos().setTo(-dy * DISTANCE, dx * DISTANCE,Float.NaN,Float.NaN);

		} else {
			if (Math.abs(dx) > (float)Math.abs(dy))
				target.pos().setTo(dy * DISTANCE, -dx * DISTANCE,Float.NaN,Float.NaN);
			else
				target.pos().setTo(-dy * DISTANCE, dx * DISTANCE,Float.NaN,Float.NaN);
		}
		//		
		target.pos().x += obstacle.x;
		target.pos().y += obstacle.y;



	}


	private void generateRandomPoint(GeoTuple4D_F32<?> center, float distance, GeoTuple4D_F32<?> output) {
		output.setTo(2f*(float)Math.random()-1f, 2*(float)Math.random()-1f, 2*(float)Math.random()-1f , 0);
		output.normalize();
		output.scale(distance);
		output.plusIP(center);
	}

	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}

	private boolean isValid(GeoTuple4D_F32<?> p) {
		return p.x != 0 || p.y != 0 || p.z != 0;
	}


	public static void main(String[] args) {

		Offboard3Planner planner = new Offboard3Planner(new MAVController(),0.5f,0.01f);

		Offboard3State soc = new Offboard3State();
		soc.pos().setTo(1,1,-1.5f,0);

		Point3D_F32 obstacle = new Point3D_F32(1.2f,1,-1.5f);

		Offboard3AbstractTarget t = new Offboard3PosVelTarget(1.0f,1f);

		planner.generateAvoidanceTarget(soc,obstacle,t,true);

		System.out.println(t);

	}


}
