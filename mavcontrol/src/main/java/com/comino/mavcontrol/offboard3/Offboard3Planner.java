package com.comino.mavcontrol.offboard3;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.collision.Offboard3CollisionCheck;
import com.comino.mavcontrol.offboard3.collision.Offboard3EDF2DCollisionCheck;
import com.comino.mavcontrol.offboard3.collision.Offboard3OctoMapCollisionCheck;
import com.comino.mavcontrol.offboard3.generator.Offboard3OctomapTrajectoryGenerator;
import com.comino.mavcontrol.offboard3.generator.Offboard3SphereTrajectoryGenerator;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3VelTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3YawTarget;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavcontrol.trajectory.minjerk.struct.Sphere;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;

public class Offboard3Planner {

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float MIN_XYZ_ESTIMATED_TIME           = 5f;                       // Minumum duration the planner ist used in [s]
	private static final float MIN_AVOIDANCE_DISTANCE           = 0.75f;                     // Distance to obstacle 
	private static final float FMAX_FEASIBILITY                 = 2.5f;                     // Maximum force applied for a plan
	private static final float WMAX_FEASIBILITY                 = 5f;                       // Maximimu rotation force for a plan

	private static final float ACCELERATION_PHASE_SECS          = 5.0f;                     // Acceleration phase in seconds
	private static final float DECELERATION_PHASE_SECS          = 2.0f;                     // Deceleration phase in seconds

	// Planners
	private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
	private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

	// Gnerator for avoidance plans
	private final Offboard3OctomapTrajectoryGenerator avoidancePlanGenerator;

	// Current state
	private final Offboard3Current            current;
	private final Offboard3Current            new_current_state = new Offboard3Current();

	// Collsion check
	//	private final Offboard3CollisionCheck     collisionCheck;
	private final Offboard3EDF2DCollisionCheck     collisionCheck;

	// DataModel && Controller
	private final DataModel                    model;
	private final IMAVController               control;
	private final MAVOctoMap3D                 map;

	// Admins
	private int plannedSectionCount = 0;

	// Planning parameters
	private float acceptance_radius;
	private float max_xyz_velocity;
	
	// Test
	//private final Offboard3EDFMinimumPlanner edf_planner;


	public Offboard3Planner(IMAVController control, MAVOctoMap3D map,float acceptance_radius, float max_xyz_velocity) {

		this.current = new Offboard3Current(control.getCurrentModel());  
		this.control = control;
		this.map     = map;
		this.model   = control.getCurrentModel();
		this.max_xyz_velocity = max_xyz_velocity;
		this.acceptance_radius = acceptance_radius;
		
	//	this.edf_planner = new Offboard3EDFMinimumPlanner(control,map,acceptance_radius, max_xyz_velocity);

		//		this.collisionCheck         = new Offboard3CollisionCheck();
		this.avoidancePlanGenerator = new Offboard3OctomapTrajectoryGenerator();

		this.collisionCheck         = new Offboard3EDF2DCollisionCheck(map);

	}

	public void setMaxVelocity(float velocity_max_ms) {
		this.max_xyz_velocity = velocity_max_ms;
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

		reset(); current.update();

		Offboard3Plan new_plan = new Offboard3Plan();
		new_plan.add(new Offboard3YawTarget(yaw));
		planPath(new_plan, current);

		MSPStringUtils.getInstance().out(new_plan);

		return new_plan;
	}



	public Offboard3Plan planCircle(GeoTuple4D_F32<?> center, float radius, float angle_rad) {

		// TODO: Works a bit wobbly, exit not good, rotation 
		//       Velocity control not within a single segment but e.g. across 2 segments

		final int circle_segments = 4;// (int)(8.0f*radius+0.5);

		current.update();

		if(MSP3DUtils.distance3D(center, current.pos()) > radius) {
			control.writeLogMessage(new LogMessage("[msp] Position not within circle.Skiped.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return null;
		}

		float velocity = radius/2.0f > max_xyz_velocity ? max_xyz_velocity : radius / 2.0f;
		float heading  = Float.isFinite(model.hud.h) ? model.hud.h : 0;

		Offboard3Plan new_plan = new Offboard3Plan();

		GeoTuple4D_F32<?> point = new Vector4D_F32();

		final int planning_segments = (int)(circle_segments * angle_rad / (2.0*Math.PI));
		float total_time = (float)( 2.0*radius*Math.PI / velocity) ;
		if(planning_segments < circle_segments) total_time = total_time * (float)planning_segments/(float)circle_segments;

		for(int seg = 1; seg <= planning_segments; seg++) {
			float a = (float)(2.0*Math.PI/circle_segments * seg)+heading;
			point.setTo((float)(Math.cos(a)*radius),(float)(Math.sin(a)*radius),center.z,a);
			point.plusIP(center);
			new_plan.add(new Offboard3VelTarget(point, velocity ,total_time/(circle_segments-1)));
		}

		float a = (float)(2.0*Math.PI/circle_segments * planning_segments)+heading;
		point.setTo((float)(Math.cos(a)*radius),(float)(Math.sin(a)*radius),center.z,a);
		point.plusIP(center);
		new_plan.add(new Offboard3PosTarget(point));

		this.acceptance_radius = (float)( 2.0*radius*Math.PI) / (2.0f*circle_segments);

		Offboard3Collision collision = planPath(new_plan, current);
		if(collision != null) {
			control.writeLogMessage(new LogMessage("[msp] Obstacle in circle. Not executed.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return null;
		}

		return new_plan;	
	}

	public Offboard3Plan planDirectPath(GeoTuple4D_F32<?> pos_target) {
		return planDirectPath(pos_target,false);
	}

	public Offboard3Plan planDirectPath(GeoTuple4D_F32<?> pos_target, boolean replanning) {
		return planDirectPath(pos_target,MSP3DUtils.distance3D(pos_target, current.pos()) / max_xyz_velocity, replanning);
	}

	public Offboard3Plan planDirectPath(GeoTuple4D_F32<?> pos_target, float estimatedTime,boolean replanning) {


		reset(); current.update();
		

		//		if(!collisionCheck.isTargetFeasible(model, pos_target)) {
		if(!collisionCheck.isTargetFeasible(pos_target)) {
			control.writeLogMessage(new LogMessage("[msp] Target not feasible.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
			return null;
		}

		Offboard3Plan new_plan = new Offboard3Plan();
		new_plan.setEstimatedTime(estimatedTime);

		float max_speed_time = new_plan.getEstimatedTime() - ACCELERATION_PHASE_SECS - DECELERATION_PHASE_SECS;
		

		if(max_speed_time < 1.0f) {
			new_plan.add(new Offboard3PosTarget(pos_target));
		}
		else {
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,ACCELERATION_PHASE_SECS ));
			new_plan.add(new Offboard3VelTarget(pos_target,max_xyz_velocity,max_speed_time));
			new_plan.add(new Offboard3PosTarget(pos_target));
		}
		
		Offboard3Collision collision = planPath(new_plan, current);
		
	//	edf_planner.planDirectPath(pos_target, 1.0f);

		if(collision != null) {

			if(replanning) 
				return null;

			new_plan = doReplanning(new_plan, collision, MIN_AVOIDANCE_DISTANCE);

			if(new_plan == null || new_plan.isEmpty()) {
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
		
		map.updateESDF(initial_state.pos());
		

		Offboard3State nextPlannedCurrentState = initial_state; 
		Offboard3Collision collision;

		plannedSectionCount = 0;

		//	Sphere obstacle = new Sphere(model.obs.x,model.obs.y, model.obs.z, 0.5f);

		plan.clearCostAndTime();

		// Plan sections
		for(Offboard3AbstractTarget section : plan) {
			
			section.replaceNaNPositionBy(initial_state.pos());
			section.setIndex(plannedSectionCount++);

			nextPlannedCurrentState = planSection(section, nextPlannedCurrentState);
			if(nextPlannedCurrentState==null) {
				plan.clear();
				return null;
			}
			plan.addCostsAndTime((float)xyzPlanner.getCost(), xyzPlanner.getTotalTime());

			if(control!=null && !control.isSimulation()) 
				return null;

			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.COLLISION_PREVENTION)) {
				//				collision = collisionCheck.check(xyzPlanner, obstacle , 0, initial_state, section.getIndex());
				collision = collisionCheck.check(xyzPlanner , 0, initial_state, section.getIndex());
				if(collision!=null) {
					return collision;
				}
			}
		}
		return null;
	}

	private Offboard3State planSection(Offboard3AbstractTarget target, Offboard3State current_state)  {


		float estimated_xyz_duration = 0; float estimated_yaw_duration = 0; 
		float planned_xyz_duration = 0; float planned_yaw_duration = 0; 

//		target.replaceNaNPositionBy(current_state.pos());

		// XYZ planning

		if(target.getDuration() < 0) {
			estimated_xyz_duration = MSP3DUtils.distance3D(target.pos(), current_state.pos()) * 2.0f / max_xyz_velocity;

			if(estimated_xyz_duration < MIN_XYZ_ESTIMATED_TIME)
				estimated_xyz_duration = MIN_XYZ_ESTIMATED_TIME;

			if(estimated_xyz_duration < estimated_yaw_duration)
				estimated_xyz_duration = estimated_yaw_duration;

		}
		else
			estimated_xyz_duration = target.getDuration();


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

			if(isValid(target.vel())) {
				planned_xyz_duration = xyzPlanner.generate(estimated_xyz_duration);
			}
			else {
				if(estimated_xyz_duration < 2)
					estimated_xyz_duration = 2f;
				planned_xyz_duration = xyzPlanner.generate(estimated_xyz_duration);
			}
			
		


			if(!xyzPlanner.checkInputFeasibility(0, FMAX_FEASIBILITY, WMAX_FEASIBILITY, RapidTrajectoryGenerator.TIME_STEP)) {
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

			xyzPlanner.getPosition(target.getPlannedSectionTime(), new_current_state.pos());
			xyzPlanner.getVelocity(target.getPlannedSectionTime(), new_current_state.vel());
			xyzPlanner.getAcceleration(target.getPlannedSectionTime(), new_current_state.acc());

			//			xyzPlanner.getGoalPosition(new_current_state.pos());
			//			xyzPlanner.getGoalVelocity(new_current_state.vel());
			//			xyzPlanner.getGoalAcceleration(new_current_state.acc());


		}

		return new_current_state;
	}


	private Offboard3Plan doReplanning(Offboard3Plan plan, Offboard3Collision col, float distance) {

		control.writeLogMessage(new LogMessage("[msp] Replanning performed.", MAV_SEVERITY.MAV_SEVERITY_WARNING));


		Offboard3Plan new_plan = avoidancePlanGenerator.getAvoidancePlan(map, this, plan, col, distance, max_xyz_velocity);
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
