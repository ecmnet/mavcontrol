package com.comino.mavcontrol.offboard2;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.mavlink.MAV_MASK;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.controllib.impl.YawSpeedControl;
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

	private static Offboard2Manager instance;

	private static final int   UPDATE_RATE                 	    = 50;					    // Offboard update rate in [ms]
	private static final int   DEFAULT_TIMEOUT                	= 5000;					    // Default timeout 1s
	private static final float RADIUS_ACCEPT                    = 0.3f;                     // Acceptance radius in [m]
	private static final float YAW_ACCEPT                	    = MSPMathUtils.toRad(0.3);  // Acceptance alignmnet yaw in [rad]

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(45);   // Maxumum speed in [rad/s]
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float YAW_PV							= 0.10f;                    // P factor for yaw speed control

	private static final float MAX_XYZ_VEL                      = 2;                        // Maxumum speed in [m/s]
	private static final float MIN_XYZ_PLANNING_DURATION        = 0.1f;                     // Minumum duration the planner ist used in [s]



	private final Offboard2Worker   worker;
	private final DataModel         model;
	private final IMAVController    control;

	// TODO: start_and_wait
	//       target update in flight
	// 


	public static Offboard2Manager getInstance(IMAVController control) {
		if(instance==null) 
			instance = new Offboard2Manager(control);
		return instance;
	}

	public static Offboard2Manager getInstance() {
		return instance;
	}

	private Offboard2Manager(IMAVController control) {
		this.control = control;
		this.model   = control.getCurrentModel();
		this.worker  = new Offboard2Worker(control);
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

	public void moveTo(float x, float y, float z, float w) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		Point4D_F32 p = new Point4D_F32(x,y,z,w);

		worker.setTarget(p);	
		worker.start();
	}
	
	public void abort() {
		if(!model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;
		worker.stopAndLoiter();
	}


	private class Offboard2Worker implements Runnable {

		private final msg_set_position_target_local_ned cmd 		= new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue wq = WorkQueue.getInstance();

		// current state
		private final GeoTuple4D_F32<Vector4D_F32> pos_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> vel_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> acc_current  = new Vector4D_F32();
		private final GeoTuple4D_F32<Vector4D_F32> pos_setpoint = new Vector4D_F32();

		// Worker targets
		private final GeoTuple4D_F32<Point4D_F32> pos = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		private final GeoTuple4D_F32<Point4D_F32> vel = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		private final GeoTuple3D_F32<Point3D_F32> acc = new Point3D_F32(Float.NaN, Float.NaN, Float.NaN);

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

		// Controllers
		private final YawSpeedControl           yawControl = new YawSpeedControl(YAW_PV,0,MAX_YAW_VEL);

		// Timing
		private long t_started = 0;
		private long t_timeout = 0;

		private float t_elapsed = 0;
		private float t_elapsed_last = 0;
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
			if(timeout < DEFAULT_TIMEOUT) timeout = DEFAULT_TIMEOUT;
			start(reached,null,timeout);
		}

		public void start(ITargetReached reached, ITimeout timeout, long timeout_ms) {

			if(model.sys.isStatus(Status.MSP_LANDED)) {
				reset();
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}

			this.reached   = reached;
			this.timeout   = timeout;
			this.t_started   = System.currentTimeMillis();
			this.t_elapsed   = 0;
			this.t_timeout   = timeout_ms;

			this.t_elapsed_last   = t_started;

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
			
			model.slam.clearFlags();

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

			reset();
			
			yaw = normAngle(yaw);  pos_current.w = normAngle(pos_current.w);

			//System.err.print(yaw+"/"+pos_current.w);
			
			if(yaw >= Math.PI && pos_current.w < 0)
				yaw = (yaw-(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
			
			if(yaw <= 0 && pos_current.w > Math.PI)
				yaw = ((float)MSPMathUtils.PI2+yaw) % (float)MSPMathUtils.PI2;
			
			//System.err.println(" ==> "+yaw+"/"+pos_current.w);
				
			yawPlanner.setInitialState(pos_current.w, vel_current.w, 0);
			yawPlanner.setTargetState(yaw, 0, Double.NaN);

			if(mean_yaw_vel < MAX_YAW_VEL)
				estimated_duration = Math.abs(yaw -pos_current.w)/mean_yaw_vel;
			else
				estimated_duration = Math.abs(yaw -pos_current.w)/MAX_YAW_VEL;

			if(estimated_duration > MIN_YAW_PLANNING_DURATION) {
				if(estimated_duration < 2)
					estimated_duration = 2;
				t_planned_yaw = yawPlanner.generateTrajectory(estimated_duration);
				System.out.println("\tYaw: "+MSPMathUtils.fromRad(yaw)+" in "+estimated_duration+" secs");
			}
			else {
				t_planned_yaw = 0;
				System.out.println("\tYaw without planner: "+MSPMathUtils.fromRad(yaw)+" : "+MSPMathUtils.fromRad(normAngleAbs(yaw, pos_current.w))+" in "+estimated_duration+" secs");
			}

			pos.setTo(Float.NaN,Float.NaN,Float.NaN, yaw);
			vel.setTo(0, 0, 0, Float.NaN);
			acc.setTo(0, 0, 0);

		}

		public void setTarget(GeoTuple4D_F32<?> pos_target) {
			setTarget(pos_target,null,null);
		}

		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target) {
			setTarget(pos_target,vel_target,null);
		}

		public void setTarget(GeoTuple4D_F32<?> pos_target, GeoTuple4D_F32<?> vel_target,GeoTuple3D_F32<?> acc_target) {

			float estimated_yaw_duration = 0; float estimated_xyz_duration = 0;

			reset();

			if(!MSP3DUtils.replaceNaN3D(pos_target, pos_setpoint))
			  MSP3DUtils.replaceNaN3D(pos_target, pos_current);	

			acc.setTo(0, 0, 0);
			vel.setTo(0,0,0,0);
			pos.setTo(pos_target.x,pos_target.y,pos_target.z,pos_current.w);

			// Yaw planning if target yaw available

			if(Float.isFinite(pos_target.w)) {
				
				
				pos_target.w = normAngle(pos_target.w);  pos_current.w = normAngle(pos_current.w);
				
				if(pos_current.w  < 0)
					pos_target.w = (pos_target.w-2*(float)Math.PI) % (2*(float)Math.PI);

				yawPlanner.setInitialState(pos_current.w, vel_current.w, 0);
				yawPlanner.setTargetState(normAngle(pos_target.w), 0, Double.NaN);

				estimated_yaw_duration = Math.abs(pos_target.w - pos_current.w)/MAX_YAW_VEL;

				if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
					if(estimated_yaw_duration < 3)
						estimated_yaw_duration = 3;
					t_planned_yaw = yawPlanner.generateTrajectory(estimated_yaw_duration);
					System.out.println("\tYaw: "+MSPMathUtils.fromRad(pos_target.w)+" in "+estimated_yaw_duration+" secs");
				}
				else {
					yawPlanner.reset();
					System.out.println("\tYaw without planner: "+MSPMathUtils.fromRad(normAngleAbs(pos_target.w, pos_current.w))+" in "+estimated_yaw_duration+" secs");
				}
				pos.w = pos_target.w;
			} else {
				yawControl.reset();
			}

			// XYZ planning

			xyzPlanner.setInitialState(pos_current, vel_current,  acc_current);
			if(vel_target!=null && acc_target!=null)
				xyzPlanner.setGoal(pos_target, vel_target, acc_target);
			else if(vel_target!=null)
				xyzPlanner.setGoal(pos_target, vel_target, acc);
			else
				xyzPlanner.setGoal(pos_target, vel, acc);

			estimated_xyz_duration = MSP3DUtils.distance3D(pos_target, pos_current) * 8.0f / MAX_XYZ_VEL;

			if(estimated_xyz_duration > MIN_XYZ_PLANNING_DURATION) {
				if(estimated_xyz_duration < 3)
					estimated_xyz_duration = 3;
				t_planned_xyz = xyzPlanner.generate(estimated_xyz_duration);
			} else
				xyzPlanner.reset();

			System.out.println("\tXYZ: "+pos_target+" ("+MSP3DUtils.distance3D(pos_target, pos_current)+") in "+estimated_xyz_duration+" secs");

		}
		
		public boolean isPlannerActive() {
			return yawPlanner.isPlanned() || xyzPlanner.isPlanned();
		}


		@Override
		public void run() {

			// Convert current state
			updateCurrentState();

			t_elapsed_last = t_elapsed;
			t_elapsed = (System.currentTimeMillis() - t_started) / 1000f;

			// check current state and perform action 

			if((yawPlanner.isPlanned() || xyzPlanner.isPlanned()) &&
					t_elapsed > t_planned_yaw && t_elapsed > t_planned_xyz &&	
					targetReached(pos_current, pos, acceptance_radius, acceptance_yaw)) {
				
				model.slam.setFlag(Slam.OFFBOARD_FLAG_REACHED, true);

				//System.out.println(t_elapsed+":"+t_planned+" -> "+targetReached(pos_current, pos, acceptance_radius, acceptance_yaw));

				if(reached!=null) reached.action();
				stopAndLoiter();

				updateTrajectoryModel(t_elapsed);
				// notify waiting thread
				return;
			}

			// check timeout
			if(t_timeout > 0 && (System.currentTimeMillis()-t_started) > t_timeout) {
				model.slam.setFlag(Slam.OFFBOARD_FLAG_TIMEOUT, true);
				stopAndLoiter();
				if(timeout!=null)
					timeout.action();
				updateTrajectoryModel(t_elapsed);
				control.writeLogMessage(new LogMessage("[msp] Offboard timeout. Switched to HOLD.", MAV_SEVERITY.MAV_SEVERITY_ERROR));

				// notify waiting thread
				return;
			}

			// set setpoint synchronized and send to PX4
			synchronized(this) {

				cmd.type_mask    = 0;
				model.slam.clearFlags();

				if(Float.isNaN(pos.x) || Float.isNaN(pos.y) || Float.isNaN(pos.z)) {
					pos.x = pos_current.x; 
					pos.y = pos_current.y;
					pos.z = pos_current.z;
				}

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, true);
					cmd.x       = (float)xyzPlanner.getPosition(t_elapsed,0);
					cmd.y       = (float)xyzPlanner.getPosition(t_elapsed,1);
					cmd.z       = (float)xyzPlanner.getPosition(t_elapsed,2);
				} else {
					if(Float.isFinite(pos.x) && Float.isFinite(pos.y) && Float.isFinite(pos.z)) 
					  model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_DIRECT, true);
					cmd.x       = pos.x;
					cmd.y       = pos.y;
					cmd.z       = pos.z;
				}

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.vx       = (float)xyzPlanner.getVelocity(t_elapsed,0);
					cmd.vy       = (float)xyzPlanner.getVelocity(t_elapsed,1);
					cmd.vz       = (float)xyzPlanner.getVelocity(t_elapsed,2);
				} else {
					cmd.vx       = vel.x;
					cmd.vy       = vel.y;
					cmd.vz       = vel.z;   
				}

				if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
					cmd.afx       = (float)xyzPlanner.getAcceleration(t_elapsed,0);
					cmd.afy       = (float)xyzPlanner.getAcceleration(t_elapsed,1);
					cmd.afz       = (float)xyzPlanner.getAcceleration(t_elapsed,2);
				} else {
					cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_ACCELERATION_IGNORE;
					cmd.afx       = acc.x;
					cmd.afy       = acc.y;
					cmd.afz       = acc.z;
				}

				if(yawPlanner.isPlanned()) {	

					// Yaw controlled by planner
					if(t_elapsed <= yawPlanner.getTotalTime()) {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_PLANNER, true);
						cmd.yaw_rate = (float)yawPlanner.getVelocity(t_elapsed);
						cmd.yaw      = (float)yawPlanner.getPosition(t_elapsed);
					} else {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
						cmd.yaw_rate = 0;
						cmd.yaw      = pos.w;
					}

				} else {

					// Yaw control aligns to path based on velocity direction or setpoint
					if(xyzPlanner.isPlanned() && t_elapsed <= xyzPlanner.getTotalTime()) {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_CONTROL, true);
						pos.w = MSP3DUtils.angleXY(cmd.vx,cmd.vy);
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
						// TODO: depend max_yaw_speed on vehicle velocity or acceleration in case of moving
						//       maybe also on planned trajectory time (e.g. yaw completed in a third of the time
						cmd.yaw_rate = yawControl.update(MSPMathUtils.normAngle(pos.w - pos_current.w), t_elapsed - t_elapsed_last,MAX_YAW_VEL);
					} else {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
						cmd.yaw      = pos.w;
					}

				}


				if(isRunning) {
					cmd.time_boot_ms = model.sys.t_boot_ms;
					cmd.isValid  = true;
					cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
					control.sendMAVLinkMessage(cmd);

					if(!offboardEnabled)
						enableOffboard();


					if(t_elapsed < xyzPlanner.getTotalTime())
						updateTrajectoryModel(t_elapsed);

				}
			}
		}

		private void reset() {

			reached = null;

			yawPlanner.reset();
			xyzPlanner.reset();
			yawControl.reset();

			acceptance_radius = RADIUS_ACCEPT;
			acceptance_yaw    = YAW_ACCEPT;
			t_timeout         = DEFAULT_TIMEOUT;
			t_elapsed_last   = 0;
			t_planned_yaw = 0;
			t_planned_xyz = 0;

			updateCurrentState(); 

		}

		private void updateCurrentState() {

			MSP3DUtils.convertCurrentPosition(model, pos_current);
			MSP3DUtils.convertCurrentSpeed(model, vel_current);
			MSP3DUtils.convertCurrentAcceleration(model, acc_current);
			MSP3DUtils.convertTargetState(model, pos_setpoint);

		}

		private float normAngle(float a) {
			return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
		}

		private float normAngle(float a, float b) {
			return normAngle(b-a);
		}

		private float normAngleAbs(float a, float b) {
			return (float)Math.abs(normAngle(a,b));
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

			if(Float.isFinite(t.w) && Float.isFinite(max_yaw) && normAngleAbs(t.w,c.w) > max_yaw) {
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


		private void updateTrajectoryModel(float elapsed_time) {

			if(!model.slam.isFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER)) {
				model.traj.clear();
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
