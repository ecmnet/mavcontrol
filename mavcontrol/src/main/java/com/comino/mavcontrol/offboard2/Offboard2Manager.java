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
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector4D_F32;

public class Offboard2Manager {

	private static final int   UPDATE_RATE                 	    = 50;					    // offboard update rate in ms
	private static final float RADIUS_ACCEPT                    = 0.2f;                     // Acceptance radius in m
	private static final float YAW_ACCEPT                	    = MSPMathUtils.toRad(1);    // Acceptance yaw in rad

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);    // in rad / s
	private static final float MIN_YAW_DURATION                 = 2f;


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
		worker.start(() -> worker.stopAndLoiter());
	}


	private class Offboard2Worker implements Runnable {

		private final msg_set_position_target_local_ned cmd 		= new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue wq = WorkQueue.getInstance();

		// current state
		private final Vector4D_F32 pos_current = new Vector4D_F32();
		private final Vector4D_F32 vel_current = new Vector4D_F32();

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
		private final SingleAxisTrajectory yawPlanner = new SingleAxisTrajectory();

		private long t_started = 0;
		private long t_timeout = 0;

		private float t_elapsed = 0;
		private float t_planned = 0;

		public Offboard2Worker(IMAVController control) {
			this.control = control;
			this.model   = control.getCurrentModel();
		}
		
		public void start() {
			start(null);
		}
		
		public void start(ITargetReached reached) {
			start(reached,null,(long)t_planned*2000);
		}

		public void start(ITargetReached reached, ITimeout timeout, long timeout_ms) {

			this.reached   = reached;
			this.timeout   = timeout;
			this.t_started   = System.currentTimeMillis();
			this.t_elapsed   = 0;
			this.t_timeout   = timeout_ms;
			
			if(t_planned == 0) {
				control.writeLogMessage(new LogMessage("[msp] No planned offboard trajectory.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
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

			updateCurrentState();

			yawPlanner.setInitialState(pos_current.w, vel_current.w, 0);
			yawPlanner.setTargetState(yaw, 0, Double.NaN);
			
			if(mean_yaw_vel < MAX_YAW_VEL)
			    t_planned = Math.abs(yaw - pos_current.w)/mean_yaw_vel;
			else
				t_planned = Math.abs(yaw - pos_current.w)/MAX_YAW_VEL;
			
			if(t_planned > MIN_YAW_DURATION)
			  yawPlanner.generateTrajectory(t_planned);

			type_mask = 0;
			pos.setTo(Float.NaN, Float.NaN, Float.NaN, yaw);
			vel.setTo(0, 0, 0, 0);
			acc.setTo(0, 0, 0);
			
		}


		public void setTarget(GeoTuple4D_F32<?> pos_target) {
			// TODO: set MASK correctly
			pos.setTo((Point4D_F32) pos_target);
			vel.setTo(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
			acc.setTo(Float.NaN, Float.NaN, Float.NaN);
		}

		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target) {
			// TODO: set MASK correctly
			pos.setTo((Point4D_F32) pos_target);
			vel.setTo((Point4D_F32) vel_target);
			acc.setTo(Float.NaN, Float.NaN, Float.NaN);
		}

		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target, GeoTuple3D_F32<?> acc_target) {
			// TODO: set MASK correctly
			pos.setTo((Point4D_F32) pos_target);
			vel.setTo((Point4D_F32) vel_target);
			acc.setTo((Point3D_F32) acc_target);
		}


		@Override
		public void run() {

			// Convert current state
			updateCurrentState();
			
			t_elapsed = (System.currentTimeMillis() - t_started) / 1000f;

			// check current state and perform action but continue sending setpoints
			if((t_elapsed > t_planned && targetReached(pos_current, pos, acceptance_radius, acceptance_yaw) ) && reached!=null) {
				//System.out.println(t_elapsed+":"+t_planned+" -> "+targetReached(pos_current, pos, acceptance_radius, acceptance_yaw));
				reached.action();
				// notify waiting thread
			}

			// check timeout
			if(t_timeout > 0 && (System.currentTimeMillis()-t_started) > t_timeout) {
				stopAndLoiter();
				if(timeout!=null)
				   timeout.action();
				// notify waiting thread
				return;
			}

			// set setpoint synchronized and send to PX4
			synchronized(this) {

				cmd.type_mask    = type_mask;

				cmd.x       = pos.x;
				cmd.y       = pos.y;
				cmd.z       = pos.z;

				if(yawPlanner.isPlanned() && t_elapsed <= yawPlanner.getTotalTime())
					cmd.yaw     = (float)yawPlanner.getPosition(t_elapsed);
				else
					cmd.yaw     = pos.w;


				cmd.vx       = vel.x;
				cmd.vy       = vel.y;
				cmd.vz       = vel.z;

				// adjust yaw_rate according plan, otherwise keep last planned rate
				if(yawPlanner.isPlanned() && t_elapsed <= yawPlanner.getTotalTime())
					cmd.yaw_rate = (float)yawPlanner.getVelocity(t_elapsed);

				cmd.afx       = acc.x;
				cmd.afy       = acc.y;
				cmd.afz       = acc.z;


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

			acceptance_radius = RADIUS_ACCEPT;
			acceptance_yaw    = YAW_ACCEPT;
			t_timeout = 0;
			t_planned = 0;
		}

		private void updateCurrentState() {
			MSP3DUtils.convertCurrentPosition(model, pos_current);
			MSP3DUtils.convertCurrentSpeed(model, vel_current);
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
