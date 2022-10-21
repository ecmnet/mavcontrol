package com.comino.mavcontrol.offboard2;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
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

public class Offboard2Manager {

	private static final int   UPDATE_RATE                 	    = 50;					    // Offboard update rate in [ms]
	private static final float RADIUS_ACCEPT                    = 0.2f;                     // Acceptance radius in [m]
	private static final float YAW_ACCEPT                	    = MSPMathUtils.toRad(0.5);  // Acceptance alignmnet yaw in [rad]

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.5f;                     // Minumum duration the planner ist used in [s]

	private static final float MAX_XYZ_VEL                      = 2;                        // Maxumum speed in [m/s]
	private static final float MIN_XYZ_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]


	private final Offboard2Worker   worker;
	private final DataModel         model;
	private final IMAVController    control;


	public Offboard2Manager(IMAVController control) {
		this.control = control;
		this.model   = control.getCurrentModel();
		this.worker  = new Offboard2Worker(control);
	}


	public void rotate(float degree) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER))
			return;

		worker.setTarget(MSPMathUtils.toRad(degree));	
		worker.start();
	}
	
	public void moveTo(GeoTuple4D_F32<?> target) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER))
			return;

		worker.setTarget(target);	
		worker.start();
	}


	private class Offboard2Worker implements Runnable {

		private final msg_set_position_target_local_ned cmd 		= new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue wq = WorkQueue.getInstance();

		// current state
		private final Vector4D_F32 pos_current = new Vector4D_F32();
		private final Vector4D_F32 vel_current = new Vector4D_F32();
		private final Vector4D_F32 acc_current = new Vector4D_F32();

		// Worker targets
		private final GeoTuple4D_F32<Point4D_F32> pos = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		private final GeoTuple4D_F32<Point4D_F32> vel = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		private final GeoTuple3D_F32<Point3D_F32> acc = new Point3D_F32(Float.NaN, Float.NaN, Float.NaN);

		private int type_mask = 0;

		private float acceptance_radius = RADIUS_ACCEPT;
		private float acceptance_yaw    = YAW_ACCEPT;

		private int     offboard_worker = 0;
		private boolean isRunning       = false;
		private boolean offboardEnabled = false;

		private ITargetReached  reached = null;
		private ITimeout         timeout = null;

		// Planners
		private final SingleAxisTrajectory      yawPlanner = new SingleAxisTrajectory();
		private final RapidTrajectoryGenerator  xyzPlanner = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));


		// Timing
		private long t_started = 0;
		private long t_timeout = 0;

		private float t_elapsed = 0;
		private float t_planned_yaw = 0;
		private float t_planned_xyz = 0;

		public Offboard2Worker(IMAVController control) {
			this.control = control;
			this.model   = control.getCurrentModel();
		}

		public void start() {
			start(() -> stopAndLoiter());
		}

		public void start(ITargetReached reached) {
			long timeout = (long) (t_planned_yaw < t_planned_xyz ? t_planned_xyz  : t_planned_yaw ) * 2000;
			start(reached,null,timeout);
		}

		public void start(ITargetReached reached, ITimeout timeout, long timeout_ms) {

			this.reached   = reached;
			this.timeout   = timeout;
			this.t_started   = System.currentTimeMillis();
			this.t_elapsed   = 0;
			this.t_timeout   = timeout_ms;

			if(model.sys.isStatus(Status.MSP_LANDED)) {
				reset();
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}

			if(isRunning)
				return;

			isRunning = true;
			offboard_worker = wq.addCyclicTask("NP", UPDATE_RATE, this);

		}

		public void stopAndLoiter() {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
			stop();
		}

		public void stop() {

			this.isRunning       = false;
			this.offboardEnabled = false;

			wq.removeTask("NP", offboard_worker);
			offboard_worker = 0;

			pos.setTo(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
			vel.setTo(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
			acc.setTo(Float.NaN, Float.NaN, Float.NaN);

			reset();
		}

		public void setAcceptance(float max, float max_yaw) {
			acceptance_radius = max;
			acceptance_yaw    = max_yaw;
		}

		public void setTarget(float yaw) {
			setTarget(yaw,MAX_YAW_VEL);
		}

		public void setTarget(float yaw, float mean_yaw_vel) {

			float estimated_duration;

			updateCurrentState();

			yawPlanner.setInitialState(pos_current.w, vel_current.w, 0);
			yawPlanner.setTargetState(yaw, 0, Double.NaN);

			if(mean_yaw_vel < MAX_YAW_VEL)
				estimated_duration = Math.abs(yaw - pos_current.w)/mean_yaw_vel;
			else
				estimated_duration = Math.abs(yaw - pos_current.w)/MAX_YAW_VEL;

			if(estimated_duration > MIN_YAW_PLANNING_DURATION)
				t_planned_yaw = yawPlanner.generateTrajectory(estimated_duration);
			else
				t_planned_yaw = 0;

			type_mask = 0;
			pos.setTo(Float.NaN, Float.NaN, Float.NaN, yaw);
			vel.setTo(0, 0, 0, 0);
			acc.setTo(0, 0, 0);

		}
		
		public void setTarget(GeoTuple4D_F32<?> pos_target) {
			setTarget(pos_target,null,null);
		}
		
		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target) {
			setTarget(pos_target,vel_target,null);
		}

		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target,GeoTuple3D_F32<?> acc_target) {

			float target_yaw; float estimated_yaw_duration; float estimated_xyz_duration;

			updateCurrentState();

			// Yaw planning

			yawPlanner.setInitialState(pos_current.w, vel_current.w, 0);

			// if target yaw is specified, use it otherwise align with path in XY direction
			if(Float.isFinite(pos_target.w)) 
				target_yaw = pos_target.w;
			else {
				target_yaw = MSP3DUtils.angleXY(pos_target, pos_current);
			}

			estimated_yaw_duration = Math.abs(target_yaw - pos_current.w) * 3 / MAX_YAW_VEL;
			yawPlanner.setTargetState(target_yaw, 0, Double.NaN);

			if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
				t_planned_yaw = yawPlanner.generateTrajectory(estimated_yaw_duration);
			} else
				t_planned_yaw = 0;
			
			System.out.println("Yaw: "+MSPMathUtils.fromRad(target_yaw)+" in "+estimated_yaw_duration+" secs");

			// XYZ planning
			
			acc.setTo(0, 0, 0);
			vel.setTo(0,0,0,0);
			pos.setTo(pos_target.x,pos_target.y,pos_target.z,target_yaw);

			xyzPlanner.setInitialState(pos_current, vel_current,  acc_current);
			if(vel_target!=null && acc_target!=null)
				xyzPlanner.setGoal(pos_target, vel_target, acc_target);
			else if(vel_target!=null)
				xyzPlanner.setGoal(pos_target, vel_target, acc);
			else
				xyzPlanner.setGoal(pos_target, vel, acc);

			estimated_xyz_duration = MSP3DUtils.distance3D(pos_target, pos_current) * 3 / MAX_XYZ_VEL;

			if(estimated_xyz_duration > MIN_XYZ_PLANNING_DURATION) {
				t_planned_xyz = xyzPlanner.generate(estimated_xyz_duration);
			} else
				t_planned_xyz = 0;
			
			System.out.println("XYZ: "+pos_target+" ("+MSP3DUtils.distance3D(pos_target, pos_current)+") in "+estimated_xyz_duration+" secs");

			type_mask = 0;

		}


		@Override
		public void run() {

			// Convert current state
			updateCurrentState();

			t_elapsed = (System.currentTimeMillis() - t_started) / 1000f;

			// check current state and perform action but continue sending setpoints
			if((t_elapsed > t_planned_yaw && t_elapsed > t_planned_xyz &&
					targetReached(pos_current, pos, acceptance_radius, acceptance_yaw) ) && reached!=null) {
				//System.out.println(t_elapsed+":"+t_planned+" -> "+targetReached(pos_current, pos, acceptance_radius, acceptance_yaw));
				reached.action();
				// notify waiting thread
			}

			// check timeout
			if(t_timeout > 0 && (System.currentTimeMillis()-t_started) > t_timeout) {
				stopAndLoiter();
				if(timeout!=null)
					timeout.action();
				control.writeLogMessage(new LogMessage("[msp] Offboard timeout. Switched to HOLD.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				// notify waiting thread
				return;
			}

			// set setpoint synchronized and send to PX4
			synchronized(this) {

				cmd.type_mask    = type_mask;

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.x       = (float)xyzPlanner.getPosition(t_elapsed,0);
					cmd.y       = (float)xyzPlanner.getPosition(t_elapsed,1);
					cmd.z       = (float)xyzPlanner.getPosition(t_elapsed,2);
				} else {
					cmd.x       = pos.x;
					cmd.y       = pos.y;
					cmd.z       = pos.z;
				}

				if(yawPlanner.isPlanned() && t_elapsed <= yawPlanner.getTotalTime())
					cmd.yaw     = (float)yawPlanner.getPosition(t_elapsed);
				else
					cmd.yaw     = pos.w;

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.vx       = (float)xyzPlanner.getVelocity(t_elapsed,0);
					cmd.vy       = (float)xyzPlanner.getVelocity(t_elapsed,1);
					cmd.vz       = (float)xyzPlanner.getVelocity(t_elapsed,2);
				} else {
					cmd.vx       = vel.x;
					cmd.vy       = vel.y;
					cmd.vz       = vel.z;
				}

				if(yawPlanner.isPlanned() && t_elapsed <= yawPlanner.getTotalTime())
					cmd.yaw_rate = (float)yawPlanner.getVelocity(t_elapsed);
				else
					cmd.yaw_rate = 0;

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.afx       = (float)xyzPlanner.getAcceleration(t_elapsed,0);
					cmd.afy       = (float)xyzPlanner.getAcceleration(t_elapsed,1);
					cmd.afz       = (float)xyzPlanner.getAcceleration(t_elapsed,2);
				} else {
					cmd.afx       = acc.x;
					cmd.afy       = acc.y;
					cmd.afz       = acc.z;
				}


				cmd.time_boot_ms = model.sys.t_boot_ms;
				cmd.isValid  = true;

				cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
			}

			if(isRunning) {
				control.sendMAVLinkMessage(cmd);

				if(!offboardEnabled)
					enableOffboard();
			}
		}

		private void reset() {
			reached = null;

			yawPlanner.reset();
			xyzPlanner.reset();

			acceptance_radius = RADIUS_ACCEPT;
			acceptance_yaw    = YAW_ACCEPT;
			t_timeout = 0;
			t_planned_yaw = 0;
		}

		private void updateCurrentState() {
			MSP3DUtils.convertCurrentPosition(model, pos_current);
			MSP3DUtils.convertCurrentSpeed(model, vel_current);
			MSP3DUtils.convertCurrentAcceleration(model, acc_current);
		}


		private boolean targetReached(GeoTuple4D_F32<?> c, GeoTuple4D_F32<?> t, float max, float max_yaw) {

			if(Float.isFinite(t.x) && Float.isFinite(t.y) && Float.isFinite(t.z) && Float.isFinite(max)) {

				float dx = t.x - c.x;
				float dy = t.y - c.y;
				float dz = t.z - c.z;

				if(Math.sqrt(dx*dx+dy*dy+dz*dz) > max) {	
					return false;
				}
			}

			if(Float.isFinite(t.w) && Float.isFinite(max_yaw) && Math.abs(t.w - c.w) > max_yaw) {
				return false;	
			}
			return true;
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

	}

	private interface ITargetReached {
		public void action();
	}

	private interface ITimeout {
		public void action();
	}



}
