package com.comino.mavcontrol.offboard3;

import java.util.concurrent.locks.ReentrantLock;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.generator.Offboard3SphereTrajectoryGenerator;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3VelTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3YawTarget;
import com.comino.mavcontrol.offboard3.utils.RuntimeAnalysis;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavcontrol.trajectory.minjerk.struct.Sphere;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;

public class Offboard3Planner {

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float MIN_XYZ_ESTIMATED_TIME           = 5f;                       // Minumum duration the planner ist used in [s]
	private static final float MIN_AVOIDANCE_DISTANCE           = 0.75f;                    // Distance to obstacle center
	private static final float FMAX_FEASIBILITY                 = 1.5f;                     // Maximum force applied for a plan
	private static final float WMAX_FEASIBILITY                 = 5f;                       // Maximimu rotation force for a plan

	// Planners
	private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
	private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

	// Gnerator for avoidance plans
	private final Offboard3SphereTrajectoryGenerator avoidancePlanGenerator;

	// Current state
	private final Offboard3Current            current;
	private final Offboard3Current            new_current_state = new Offboard3Current();

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

		this.collisionCheck         = new Offboard3CollisionCheck();
		this.avoidancePlanGenerator = new Offboard3SphereTrajectoryGenerator(max_xyz_velocity);

	}

	public int getPlannedSectionCount() {
		return plannedSectionCount;
	}

	public void reset() {

		model.slam.ix = Float.NaN;
		model.slam.iy = Float.NaN;
		model.slam.iz = Float.NaN;

		plannedSectionCount = 0;
		yawPlanner.reset();
		xyzPlanner.reset();
		current.update();
	}

	public Offboard3Plan planDirectYaw(float yaw) {
		Offboard3Plan new_plan = new Offboard3Plan();
		new_plan.add(new Offboard3YawTarget(yaw));
		return new_plan;
	}

	public Offboard3Plan planDirectPath(GeoTuple4D_F32<?> pos_target) {
		return planDirectPath(pos_target,false);
	}

	public Offboard3Plan planDirectPath(GeoTuple4D_F32<?> pos_target, boolean replanning) {


		reset(); current.update();

		if(!collisionCheck.isTargetFeasible(model, pos_target)) {
			control.writeLogMessage(new LogMessage("[msp] Target not feasible.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
			return null;
		}

		Offboard3Plan new_plan = new Offboard3Plan();

		new_plan.setEstimatedTime(MSP3DUtils.distance3D(pos_target, current.pos()) / max_xyz_velocity);

		if(new_plan.getEstimatedTime() < (5/max_xyz_velocity+2.0f)) {
			new_plan.add(new Offboard3PosTarget(pos_target));
		}

		else {
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,2.0f));
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,new_plan.getEstimatedTime()*5f/8f));
			new_plan.add(new Offboard3PosTarget(pos_target));
		}
		
		Offboard3Collision collision = planPath(new_plan, current);

		if(collision != null) {

			if(replanning) 
				return null;

			new_plan = doReplanning(new_plan, collision, MIN_AVOIDANCE_DISTANCE);

			if(new_plan == null) {
				control.writeLogMessage(new LogMessage("[msp] Replanning found no solution.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
				return null;
			}

			MSPStringUtils.getInstance().err("Current state: "+current);
			MSPStringUtils.getInstance().err(new_plan);
			
		} else {
			
			MSPStringUtils.getInstance().out(new_plan);
			
		}

		return new_plan;

	}

	public Offboard3Collision planPath(Offboard3Plan plan, Offboard3Current initial_state) {

		Offboard3State nextPlannedCurrentState = initial_state; 
		Offboard3Collision collision;

		plannedSectionCount = 0;
		
		Sphere obstacle = new Sphere(model.slam.ox,model.slam.oy, model.slam.oz, 0.5f);

		// Plan sections
		for(Offboard3AbstractTarget section : plan) {
			
			section.setIndex(plannedSectionCount++);
			
			nextPlannedCurrentState = planSection(section, nextPlannedCurrentState);
			if(nextPlannedCurrentState==null) {
				control.writeLogMessage(new LogMessage("[msp] Plan not feasible. Aborted.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				plan.clear();
				return null;
			}
			plan.addCostsAndTime((float)xyzPlanner.getCost(), xyzPlanner.getTotalTime());
			
			if(control!=null && !control.isSimulation()) 
				return null;

			collision = collisionCheck.check(xyzPlanner, obstacle , 0, initial_state, section.getIndex());
			if(collision!=null)
				return collision;
		}
		return null;
	}

	private Offboard3State planSection(Offboard3AbstractTarget target, Offboard3State current_state)  {

		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0; 
		float planned_xyz_duration = 0; float planned_yaw_duration = 0; 


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
			
			if(!xyzPlanner.checkInputFeasibility(0, FMAX_FEASIBILITY, WMAX_FEASIBILITY, 0.05f)) {
				return null;
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


	private Offboard3Plan doReplanning(Offboard3Plan plan, Offboard3Collision col, float distance) {

		control.writeLogMessage(new LogMessage("[msp] Replanning performed.", MAV_SEVERITY.MAV_SEVERITY_WARNING));

		if(!col.getObstacle().isValid())
			return null;


		Offboard3Plan new_plan = avoidancePlanGenerator.getAvoidancePlan(this, plan, col, distance);
		if(new_plan!=null)
			model.slam.setInfoPoint(new_plan.getFirst().pos());

		return new_plan;

	}


	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}

	private boolean isValid(GeoTuple4D_F32<?> p) {
		return p.x != 0 || p.y != 0 || p.z != 0;
	}
}
