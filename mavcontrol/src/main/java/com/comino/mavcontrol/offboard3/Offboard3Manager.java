package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.mavlink.MAV_MASK;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.controllib.impl.YawSpeedControl;
import com.comino.mavcontrol.offboard2.ITargetReached;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector4D_F32;


/*
 * TODO:
 * - Planning should take place during execution not when setting the target
 * - Multiple Targets
 */

public class Offboard3Manager {

	private static Offboard3Manager instance;

	private static final int   UPDATE_RATE                 	    = 50;					    // Offboard update rate in [ms]
	private static final float DEFAULT_TIMEOUT                	= 5.0f;					    // Default timeout 1s

	private static final float RADIUS_ACCEPT                    = 0.3f;                     // Acceptance radius in [m]
	private static final float YAW_ACCEPT                	    = MSPMathUtils.toRad(1);    // Acceptance alignmnet yaw in [rad]

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float YAW_PV							= 0.05f;                    // P factor for yaw speed control

	private static final float MAX_XYZ_VEL                      = 1.5f;                     // Maxumum speed in [m/s]

	private static final float MIN_DISTANCE_OBSTACLE            = 0.5F;                     // Minimal distance to obstacle

	private final Offboard3Worker   worker;
	private final DataModel         model;

	public static Offboard3Manager getInstance(IMAVController control) {
		if(instance==null) 
			instance = new Offboard3Manager(control);
		return instance;
	}

	public static Offboard3Manager getInstance() {
		return instance;
	}

	private Offboard3Manager(IMAVController control) {
		this.model   = control.getCurrentModel();
		this.worker  = new Offboard3Worker(control);

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE,Status.NAVIGATION_STATE_AUTO_LOITER, (n) -> {
			if(n.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && worker.isRunning) {
				worker.stop(); worker.reset();
				control.writeLogMessage(new LogMessage("[msp] Offboard externally stopped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			}
		});

	}


	public void rotate(float radians, ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		// Do not allow replanning if planner is active
		if(worker.isPlannerActive())
			return;

		worker.setTarget(radians);	
		worker.start(action);

	}

	public void rotateBy(float radians) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		float target = MSPMathUtils.normAngle(model.attitude.y+radians);

		// Do not allow replanning if planner is active
		if(worker.isPlannerActive())
			return;

		worker.setTarget(target);	
		worker.start();

	}

	public void moveTo(float x, float y, float z, float w,ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		Point4D_F32 p = new Point4D_F32(x,y,z,w);

		worker.setTarget(p);	
		worker.start(action);
	}

	public void moveTo(float x, float y, float z, float w) {
		moveTo(x,y,z,w,null);
	}

	public void abort() {
		if(!model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;
		worker.stopAndLoiter();
	}


	private class Offboard3Worker implements Runnable {

		private final msg_set_position_target_local_ned cmd 		= new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue wq = WorkQueue.getInstance();
		private final LinkedList<Offboard3Target> targets = new LinkedList<Offboard3Target>();

		// current state
		private final GeoTuple4D_F32<Vector4D_F32> pos_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> vel_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> acc_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> pos_setpoint = new Vector4D_F32();
		
		// obstacle
		final Point3D_F64 obstacle = new Point3D_F64();

		private float acceptance_radius = RADIUS_ACCEPT;
		private float acceptance_yaw    = YAW_ACCEPT;

		private float max_xyz_vel       = MAX_XYZ_VEL;

		private int     offboard_worker = 0;
		private boolean isRunning       = false;
		private boolean offboardEnabled = false;

		private ITargetReached  reached = null;
		private ITimeout         timeout = null;

		// Current target
		private Offboard3Target current_target;

		// Planners
		private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
		private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

		// Controllers
		private final YawSpeedControl           yawControl = new YawSpeedControl(YAW_PV, 0 ,MAX_YAW_VEL);

		// Timing
		private float t_timeout = 0;

		private float t_elapsed = 0;
		private float t_elapsed_last = 0;
		private float t_planned_yaw = 0;
		private float t_planned_xyz = 0;



		public Offboard3Worker(IMAVController control) {
			this.control = control;
			this.model   = control.getCurrentModel();

			MSPConfig config	= MSPConfig.getInstance();

			max_xyz_vel = config.getFloatProperty(MSPParams.AUTOPILOT_MAX_XYZ_VEL, String.valueOf(MAX_XYZ_VEL));
			System.out.println("Maximum planning velocity: "+max_xyz_vel+" m/s");
			acceptance_radius = config.getFloatProperty(MSPParams.AUTOPILOT_RADIUS_ACCEPT, String.valueOf(RADIUS_ACCEPT));
			System.out.println("Acceptance radius: "+acceptance_radius+" m");


		}

		public void start() {
			start(() -> stopAndLoiter());
		}

		public void start(ITargetReached reached) {
			start(reached,null);
		}

		public void start(ITargetReached reached_action, ITimeout timeout_action) {

			if(model.sys.isStatus(Status.MSP_LANDED)) {
				reset();
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}

			this.reached     = reached_action;
			this.timeout     = timeout_action;
			this.t_elapsed   = 0;

			this.t_elapsed_last = System.currentTimeMillis();

			if(targets.isEmpty())
				return;

			updateCurrentState();
			
			current_target = planNextTarget(pos_current, vel_current, acc_current);
			
			if(checkCollisionForPlannedTarget(current_target,0)) {
				control.writeLogMessage(new LogMessage("[msp] Collision in planned path.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
				current_target = planNextTarget(pos_current, vel_current, acc_current);	
			//	stopAndLoiter(); reset();
			//	return;
			}
			
			t_timeout = DEFAULT_TIMEOUT + (t_planned_yaw < t_planned_xyz ? t_planned_xyz  : t_planned_yaw) ;
			
			//System.err.println(xyzPlanner.isPlanned()+"/"+yawPlanner.isPlanned());

			if(xyzPlanner.isPlanned() && current_target.isPosReached(pos_current, acceptance_radius, acceptance_yaw)) {
				control.writeLogMessage(new LogMessage("[msp] Target already reached.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				if(reached!=null) 
					reached.action();
				return;
			}

			if(isRunning)
				return;

			isRunning = true;
			offboard_worker = wq.addCyclicTask("HP", UPDATE_RATE, this);

		}

		public void stopAndLoiter() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
			stop();
			reset();
		}

		public void stop() {


			this.isRunning       = false;
			this.offboardEnabled = false;

			reset();

			wq.removeTask("NP", offboard_worker);
			offboard_worker = 0;


			model.slam.clearFlags();

		}

		public void setTarget(float yaw) {

			reset();
			Point4D_F32 pos_target = new Point4D_F32(Float.NaN,Float.NaN,Float.NaN, yaw);
			targets.add(new Offboard3Target(pos_target));

		}

		public void setTarget(GeoTuple4D_F32<?> pos_target) {

			reset();
			updateCurrentState();

			float estimated_xyz_duration = MSP3DUtils.distance3D(pos_target, pos_current) / max_xyz_vel;

			if(estimated_xyz_duration < (5/max_xyz_vel+2.0f)) {
				targets.add(new Offboard3Target(pos_target));
			}

			else {
				System.out.println("Estimated duration: "+estimated_xyz_duration);
				targets.add(new Offboard3Target(pos_target,pos_current,max_xyz_vel,2.0f));
				targets.add(new Offboard3Target(pos_target,pos_current,max_xyz_vel,estimated_xyz_duration*5f/8f));
				targets.add(new Offboard3Target(pos_target));

			}
		}

		public boolean isPlannerActive() {
			return yawPlanner.isPlanned() || xyzPlanner.isPlanned();
		}


		@Override
		public void run() {

			if(!isRunning)
				return;

			// Convert current state
			updateCurrentState();

			t_elapsed_last = t_elapsed;
			t_elapsed = (System.currentTimeMillis() - current_target.getStartedTimestamp()) / 1000f;

			if((t_elapsed - t_elapsed_last) <= 0) {
				return;
			}

			// check current state and perform action 
			if((yawPlanner.isPlanned() || xyzPlanner.isPlanned()) &&
					t_elapsed > yawPlanner.getTotalTime() && t_elapsed > xyzPlanner.getTotalTime()) {

				if(current_target.isPosReached(pos_current, acceptance_radius, acceptance_yaw)) {
					model.slam.setFlag(Slam.OFFBOARD_FLAG_REACHED, true);
					if(reached!=null) {
						reached.action();
						stop();
					}
					else
						stopAndLoiter();
					updateTrajectoryModel(t_elapsed);
					return;
				} 

			}
			
			// Plan next target if required
			if(!targets.isEmpty() && xyzPlanner.isPlanned() && t_elapsed >= xyzPlanner.getTotalTime()) {
				current_target = planNextTarget(pos_current, vel_current, acc_current);	
				t_timeout = DEFAULT_TIMEOUT + (t_planned_yaw < t_planned_xyz ? t_planned_xyz  : t_planned_yaw) ;
				return;
			}

			// Collision check
			if(checkCollisionForPlannedTarget(current_target,t_elapsed)) {
				control.writeLogMessage(new LogMessage("[msp] Collison in planned path. Stopped.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				current_target = planNextTarget(pos_current, vel_current, acc_current);	
				return;
			}
			
			// check timeout
			if(t_timeout > 0 && t_elapsed > t_timeout) {
				model.slam.setFlag(Slam.OFFBOARD_FLAG_TIMEOUT, true);
				stopAndLoiter();
				if(timeout!=null)
					timeout.action();
				updateTrajectoryModel(t_elapsed);
				control.writeLogMessage(new LogMessage("[msp] Offboard timeout. Switched to HOLD.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				return;
			}

			// set setpoint synchronized and send to PX4
			synchronized(this) {

				cmd.type_mask    = 0;
				model.slam.clearFlags();

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, true);
					cmd.x       = (float)xyzPlanner.getPosition(t_elapsed,0);
					cmd.y       = (float)xyzPlanner.getPosition(t_elapsed,1);
					cmd.z       = (float)xyzPlanner.getPosition(t_elapsed,2);
				} else {
					if(!current_target.isTargetSetpoint()) 
						model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_DIRECT, true);
					cmd.type_mask    = MAV_MASK.MASK_LOITER_SETPOINT_TYPE;
					cmd.x       = current_target.getTargetPosition().x;
					cmd.y       = current_target.getTargetPosition().y;
					cmd.z       = current_target.getTargetPosition().z;
				}

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.vx       = (float)xyzPlanner.getVelocity(t_elapsed,0);
					cmd.vy       = (float)xyzPlanner.getVelocity(t_elapsed,1);
					cmd.vz       = (float)xyzPlanner.getVelocity(t_elapsed,2);
				} else {
					cmd.vx       = Float.NaN;
					cmd.vy       = Float.NaN;
					cmd.vz       = Float.NaN;  
				}

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.afx       = (float)xyzPlanner.getAcceleration(t_elapsed,0);
					cmd.afy       = (float)xyzPlanner.getAcceleration(t_elapsed,1);
					cmd.afz       = (float)xyzPlanner.getAcceleration(t_elapsed,2);
				} else {
					cmd.afx       = Float.NaN;
					cmd.afy       = Float.NaN;
					cmd.afz       = Float.NaN;
				}

				if(yawPlanner.isPlanned()) {	

					// Yaw controlled by planner
					if(t_elapsed <= yawPlanner.getTotalTime()) {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_PLANNER, true);
						cmd.yaw_rate = (float)yawPlanner.getVelocity(t_elapsed);
						cmd.yaw      = (float)yawPlanner.getPosition(t_elapsed);
					} else {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
						cmd.yaw_rate = 0;
						cmd.yaw      = current_target.getTargetPosition().w;
					}

				} else {
					

					// Yaw control aligns to path based on velocity direction or setpoint
					if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_CONTROL, true);
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
						if(Math.sqrt(cmd.vx*cmd.vx + cmd.vy*cmd.vy) > 0.001) {
							current_target.getTargetPosition().w = MSP3DUtils.angleXY(cmd.vx,cmd.vy);
							cmd.yaw      = current_target.getTargetPosition().w;
							cmd.yaw_rate = yawControl.update(MSPMathUtils.normAngle(cmd.yaw - pos_current.w), t_elapsed - t_elapsed_last,MAX_YAW_VEL);
						} 
					} else {
						// XYZ Target reached but not yaw: Further yaw turning via YAWControl
						if(!yawPlanner.isPlanned() && !current_target.isYawReached(pos_current, acceptance_yaw)) {
							model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_CONTROL, true);
							cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
							cmd.yaw       = current_target.getTargetPosition().w;
							cmd.yaw_rate  = yawControl.update(MSPMathUtils.normAngle(cmd.yaw - pos_current.w), t_elapsed - t_elapsed_last,MAX_YAW_VEL);		
						} else {
							model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
							cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
							cmd.yaw_rate = 0;
							cmd.yaw      = current_target.getTargetPosition().w;
						}
					}
				}

				model.debug.x = (float)Math.sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy +cmd.vz * cmd.vz);
				model.debug.y = cmd.yaw_rate;
				model.debug.z = cmd.yaw;


				if(isRunning) {
					cmd.time_boot_ms = model.sys.t_boot_ms;
					cmd.isValid  = true;
					cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;

					control.sendMAVLinkMessage(cmd);

					if(!offboardEnabled)
						enableOffboard();
				}
			}

			if(t_elapsed < xyzPlanner.getTotalTime())
				updateTrajectoryModel(t_elapsed);
		}

		private Offboard3Target planNextTarget(GeoTuple4D_F32<?> pc, GeoTuple4D_F32<?> vc, GeoTuple4D_F32<?>ac) {

			if(targets.isEmpty())
				return current_target;
			
			Offboard3Target new_target = targets.poll();

			if(MSP3DUtils.isFinite(new_target.getTargetPosition()))
			  control.writeLogMessage(new LogMessage("[msp] Offboard next target.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

			return planTarget(new_target,pc,vc,ac);
		}

		private Offboard3Target planTarget(Offboard3Target target, GeoTuple4D_F32<?> pc, GeoTuple4D_F32<?> vc, GeoTuple4D_F32<?>ac) {

			float estimated_xyz_duration = 0; float estimated_yaw_duration = 0;

			if(MSP3DUtils.isFinite(pos_setpoint)) {
				MSP3DUtils.replaceNaN3D(target.getTargetPosition(), pos_setpoint);
				target.setTargetIsSetpoint(true);
			}
			else
				MSP3DUtils.replaceNaN3D(target.getTargetPosition(), pc);

			// Yaw Planning 

			yawPlanner.reset(); t_planned_yaw = 0;
			if(Float.isFinite(target.getTargetPosition().w)) {

				target.getTargetPosition().w = normAngle(target.getTargetPosition().w );  
				pc.w = normAngle(pc.w);

				if(target.getTargetPosition().w  >= Math.PI && pos_current.w < 0)
					target.getTargetPosition().w  = (target.getTargetPosition().w -(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;

				if(target.getTargetPosition().w  <= 0 && pos_current.w > Math.PI)
					target.getTargetPosition().w  = ((float)MSPMathUtils.PI2+target.getTargetPosition().w ) % (float)MSPMathUtils.PI2;

				yawPlanner.setInitialState(pc.w, vc.w, 0);
				yawPlanner.setTargetState(target.getTargetPosition().w , 0, 0);

				if(target.getDuration() < 0)
					estimated_yaw_duration = Math.abs(target.getTargetPosition().w  - pc.w)/MAX_YAW_VEL;
				else
					estimated_yaw_duration = target.getDuration();

				if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
					if(estimated_yaw_duration < 3)
						estimated_yaw_duration = 3;
					t_planned_yaw = yawPlanner.generateTrajectory(estimated_yaw_duration);
					System.out.println("\tYaw: "+MSPMathUtils.fromRad(target.getTargetPosition().w )+" in "+estimated_yaw_duration+" secs");
				} 
				
			} 


			// XYZ planning

			xyzPlanner.reset(); t_planned_xyz = 0;
			if((isFinite(target.getTargetPosition()) || isValid(target.getTargetVelocity())) && 
					!target.isPosReached(pc,acceptance_radius,Float.NaN)) {

				xyzPlanner.setInitialState(pc, vc, ac);

				if(isValid(target.getTargetVelocity()))
					xyzPlanner.setGoal(null, target.getTargetVelocity(), target.getTargetAcceleration());
				else
					xyzPlanner.setGoal(target.getTargetPosition(), target.getTargetVelocity(), target.getTargetAcceleration());

				if(target.getDuration() < 0) {
					estimated_xyz_duration = MSP3DUtils.distance3D(target.getTargetPosition(), pc) * 2.0f / max_xyz_vel;

					if(estimated_xyz_duration < estimated_yaw_duration)
						estimated_xyz_duration = estimated_yaw_duration;

				}
				else
					estimated_xyz_duration = target.getDuration();

				if(isValid(target.getTargetVelocity())) {
					t_planned_xyz = xyzPlanner.generate(estimated_xyz_duration);
					System.out.println("\tXYZ Velocity: "+target+" (" +MSP3DUtils.distance3D(target.getTargetPosition(), pc) +") in "+estimated_xyz_duration+" secs");

				}
				else {
					if(estimated_xyz_duration < 2)
						estimated_xyz_duration = 2f;
					t_planned_xyz = xyzPlanner.generate(estimated_xyz_duration);
					System.out.println("\tXYZ Position: "+target+" ("+MSP3DUtils.distance3D(target.getTargetPosition(), pc)+") in "+estimated_xyz_duration+" secs");
				}
			}

			return target;

		}

		private boolean checkCollisionForPlannedTarget(Offboard3Target target, float time_start) {

			if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP))
				return false;

			float time_elapsed = 0; float time_step = 0.2f; 
			final Point3D_F64 position = new Point3D_F64();

			if(!MSP3DUtils.isFinite(obstacle))
				return false;

			// TODO: Put collsions check and re-planning into own class

			// TODO: Analytical solution as the line parameters are known

			// TODO get nearest obstacle position into obstacle

			// Brute force method
			for(time_elapsed = time_start; time_elapsed < xyzPlanner.getTotalTime(); time_elapsed += time_step) {
				xyzPlanner.getPosition(time_elapsed, position);
				
				// Check only YX distance
				if(MSP3DUtils.distance2D(position, obstacle) < MIN_DISTANCE_OBSTACLE) {
					float stop_time = time_elapsed-0.5f; targets.clear();
					if(stop_time > 0) {
						targets.clear();
						xyzPlanner.getPosition(stop_time, position);
						System.out.println("Replanning to stop in front of obstacle:");
						target = new Offboard3Target(position); 
						targets.add(target);
					} else
						stopAndLoiter();

					return true;
				}


				// TODO Check with current map

			}

			return false;
		}


		private void reset() {

			reached = null;

			yawPlanner.reset();
			xyzPlanner.reset();
			yawControl.reset();

			targets.clear();

			acceptance_radius = RADIUS_ACCEPT;
			acceptance_yaw    = YAW_ACCEPT;
			t_timeout         = DEFAULT_TIMEOUT;
			t_elapsed_last    = 0;
			t_planned_yaw     = 0;
			t_planned_xyz     = 0;

		}

		private void updateCurrentState() {

			MSP3DUtils.convertCurrentPosition(model, pos_current);
			MSP3DUtils.convertCurrentSpeed(model, vel_current);
			MSP3DUtils.convertCurrentAcceleration(model, acc_current);
			MSP3DUtils.convertTargetState(model, pos_setpoint);
			
			obstacle.setTo(model.slam.ox,model.slam.oy,model.slam.oz);

		}

		private float normAngle(float a) {
			return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
		}

		private float normAngle(float a, float b) {
			return normAngle(b-a);
		}

		private boolean isFinite(GeoTuple4D_F32<?> p) {
			return Float.isFinite(p.x) && Float.isFinite(p.y) && Float.isFinite(p.z);
		}

		private boolean isValid(GeoTuple4D_F32<?> p) {
			return p.x != 0 || p.y != 0 && p.z != 0;
		}

		private void enableOffboard() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					worker.stopAndLoiter();
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					offboardEnabled = false;
				} else {
					offboardEnabled = true;;
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
		}


		private void updateTrajectoryModel(float elapsed_time) {

			if(!model.slam.isFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER)) {
				model.traj.clear();
				// Clear trajectory in MAVGCL
				control.sendMAVLinkMessage(new msg_msp_trajectory(2,1));
				return;
			}


			model.traj.ls = xyzPlanner.getTotalTime();
			model.traj.fs = elapsed_time;
			model.traj.ax = (float)xyzPlanner.getAxisParamAlpha(0);
			model.traj.ay = (float)xyzPlanner.getAxisParamAlpha(1);
			model.traj.az = (float)xyzPlanner.getAxisParamAlpha(2);


			model.traj.bx = (float)xyzPlanner.getAxisParamBeta(0);
			model.traj.by = (float)xyzPlanner.getAxisParamBeta(1);
			model.traj.bz = (float)xyzPlanner.getAxisParamBeta(2);


			model.traj.gx = (float)xyzPlanner.getAxisParamGamma(0);
			model.traj.gy = (float)xyzPlanner.getAxisParamGamma(1);
			model.traj.gz = (float)xyzPlanner.getAxisParamGamma(2);

			model.traj.sx = (float)xyzPlanner.getInitialPosition(0);
			model.traj.sy = (float)xyzPlanner.getInitialPosition(1);
			model.traj.sz = (float)xyzPlanner.getInitialPosition(2);

			model.traj.svx = (float)xyzPlanner.getInitialVelocity(0);
			model.traj.svy = (float)xyzPlanner.getInitialVelocity(1);
			model.traj.svz = (float)xyzPlanner.getInitialVelocity(2);

			model.traj.tms = DataModel.getSynchronizedPX4Time_us();

		}
	}


	private interface ITimeout {
		public void action();
	}



}