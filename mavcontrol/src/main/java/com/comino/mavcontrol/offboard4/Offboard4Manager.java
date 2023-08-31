package com.comino.mavcontrol.offboard4;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.mavlink.MAV_MASK;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.offboard3.action.ITargetReached;
import com.comino.mavcontrol.offboard3.action.ITimeout;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;

public class Offboard4Manager implements IOffboardControl {

	private static Offboard4Manager instance;

	private static final int     UPDATE_RATE                 	    = 50;					    // Offboard update rate in [ms]
	private static final float   SECTION_TIME                 	    = 1.0f;					    // Flight time of one trajectory [s]
	private static final float   RADIUS_ACCEPT                      = 0.2f;                     // Acceptance radius in [m]

	private final MAVOctoMap3D      map;
	private final Offboard4Worker   worker;


	public static Offboard4Manager getInstance(IMAVController control,MAVOctoMap3D map) {
		if(instance==null) 
			instance = new Offboard4Manager(control,map);
		return instance;
	}

	public static Offboard4Manager getInstance() {
		return instance;
	}

	private Offboard4Manager(IMAVController control, MAVOctoMap3D map) {
		this.map     = map;
		this.worker  = new Offboard4Worker(control);
		System.out.println("Offboard4Manager instantiated");
	}

	public void planAndExecute(Vector4D_F32 target) {
		worker.setGlobalTarget(target);
		worker.start();
	}

	public void abort() {
		worker.stop();
	}


	@Override
	public void rotate(float radians, ITargetReached action) {
		// TODO Auto-generated method stub

	}

	@Override
	public void rotateBy(float radians, ITargetReached action) {
		// TODO Auto-generated method stub

	}

	@Override
	public void executePlan(Offboard3Plan plan, ITargetReached action) {
		// TODO Auto-generated method stub

	}

	@Override
	public void setMaxVelocity(float velocity_max_ms) {
		// TODO Auto-generated method stub

	}

	@Override
	public void setTimeoutAction(ITimeout timeout) {
		// TODO Auto-generated method stub

	}

	@Override
	public void moveTo(float x, float y, float z, float w, ITargetReached action) {
		this.moveTo(x, y, z, w, action,0.2f);

	}

	@Override
	public void moveTo(float x, float y, float z, float w, ITargetReached action, float acceptance_radius_m) {
		final Vector4D_F32 target = new Vector4D_F32(x,y,z,w);
		worker.setGlobalTarget(target);
		worker.start();

	}

	@Override
	public void moveTo(float x, float y, float z, float w) {
		this.moveTo(x, y, z, w, null,0.2f);
	}

	@Override
	public boolean isPlanned() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void getProjectedPositionAt(float time, GeoTuple4D_F32<?> pos) {
		// TODO Auto-generated method stub

	}


	private class Offboard4Worker implements Runnable {

		private final double NANO_TO_SEC = 1 / 1e9;

		private final msg_set_position_target_local_ned cmd = new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue                 wq;

		private int     offboard_worker    = 0;
		private boolean isRunning          = false;
		private boolean offboardEnabled    = false;

		private long    t_section_start    = 0;
		private double  tf                 = 0;

		private boolean  newTargetSet      = false;

		// current state
		private final Offboard3Current      current;
		private final GeoTuple4D_F32<?>     global_target;
		private final GeoTuple4D_F32<?>     global_start;
		private final Offboard4EDF2DPlanner planner;


		// Current trajectory to be executed
		private RapidTrajectoryGenerator  xyzExecutor;


		public Offboard4Worker(IMAVController control) {

			this.control        = control;
			this.model          = control.getCurrentModel();
			this.current        = new Offboard3Current(model);
			this.global_target  = new Vector4D_F32();
			this.global_start   = new Vector4D_F32();
			this.planner        = new Offboard4EDF2DPlanner(control, map);

			this.wq          = WorkQueue.getInstance();

		}

		public void start() {

			if(model.sys.isStatus(Status.MSP_LANDED)) {
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}

			if(!model.sys.isStatus(Status.MSP_CONNECTED) || isRunning)
				return;

			if(!wq.isInQueue("NP", offboard_worker)) {
				isRunning = true; 
				offboard_worker = wq.addCyclicTask("NP", UPDATE_RATE, this);
			}
		}

		public void stop() {
			this.xyzExecutor     = null;
			this.model.traj.clear();
			this.model.obs.clear();
			model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, false);
			if(offboard_worker != 0)
				this.wq.removeTask("NP", offboard_worker);
			this.isRunning = false;
			switchToLoiter();

		}

		public void setGlobalTarget(Vector4D_F32 target) {
			newTargetSet = true;
			current.update();

			//MSP3DUtils.replaceNaN3D(target, current.pos());

			target.z = -1.5f;

			model.obs.x = target.x;
			model.obs.y = target.y;
			model.obs.z = target.z;
			this.global_target.setTo(target.x,target.y,target.z,target.w);	
			this.global_start.setTo(current.pos().x, current.pos().y, current.pos().z, current.pos().w);

		}

		@Override
		public void run() {


			if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
				this.stop(); 
				control.writeLogMessage(new LogMessage("[msp] Offboard externally stopped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				return;
			}

			current.update();

			if(isTargetReached() || isTimeOut()) {
				stop();
				System.out.println();
				return;
			}
			
			float time = Math.max(SECTION_TIME, 1f/(0.3f + 2.0f * current.vel().norm()));

			tf = (System.nanoTime() - t_section_start) * NANO_TO_SEC;

			if(newTargetSet || tf > time) {
				model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, true);

				// 		map.updateESDF(current.pos());
				xyzExecutor = planner.planDirectPath(global_start,global_target, 0.5f, time);
				updateTrajectoryModel(tf);
				newTargetSet = false;
				t_section_start = System.nanoTime();	
				return;

			}
			executeCurrentTrajectory(tf);
			updateTrajectoryModel(tf);

		}

		private boolean isTargetReached() {	
			//	final float distance =  current.pos().distance(current_target);
			final float distance = MSP3DUtils.distance2D( current.pos(), global_target);
			return distance < RADIUS_ACCEPT;
		}

		private boolean isTimeOut() {
			return false;
		}


		private void executeCurrentTrajectory(double tf) {


			model.slam.clearFlags();
			synchronized(this) {

				cmd.type_mask        = 0;
				cmd.target_system    = 1;
				cmd.target_component = 1;

				// do not control yaw currently
				cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;

				cmd.x       = (float)xyzExecutor.getPosition(tf,0);
				cmd.y       = (float)xyzExecutor.getPosition(tf,1);
				//				cmd.z       = (float)xyzExecutor.getPosition(tf,2);
				cmd.z = -1.5f;


				cmd.vx       = (float)xyzExecutor.getVelocity(tf,0);
				cmd.vy       = (float)xyzExecutor.getVelocity(tf,1);
				cmd.vz       = (float)xyzExecutor.getVelocity(tf,2);


				cmd.afx      = (float)xyzExecutor.getAcceleration(tf,0);
				cmd.afy      = (float)xyzExecutor.getAcceleration(tf,1);
				cmd.afz      = (float)xyzExecutor.getAcceleration(tf,2);	

				cmd.vz = cmd.afz = 0;

				if(isRunning) {

					if(!offboardEnabled ) {
						enableOffboard();
					}

					model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, true);
					cmd.time_boot_ms = model.sys.t_boot_ms;
					cmd.isValid  = true;
					cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
					control.sendMAVLinkMessage(cmd);

				}

			}

		}

		public void switchToLoiter() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd,result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) 
					control.writeLogMessage(new LogMessage("Switching to hold failed. Continue offboard",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				else
					offboardEnabled = false;
			},MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
		}

		private void enableOffboard() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					stop();
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					offboardEnabled = false;

				} else {
					offboardEnabled = true;
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
		}


		private void updateTrajectoryModel(double elapsed_time) {

			if(!model.slam.isFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER)) {
				model.traj.clear();
				// Clear trajectory in MAVGCL
				control.sendMAVLinkMessage(new msg_msp_trajectory(2,1));
				return;
			}

			model.traj.ls = xyzExecutor.getTotalTime();
			model.traj.fs = (float)elapsed_time;
			model.traj.ax = (float)xyzExecutor.getAxisParamAlpha(0);
			model.traj.ay = (float)xyzExecutor.getAxisParamAlpha(1);
			model.traj.az = (float)xyzExecutor.getAxisParamAlpha(2);


			model.traj.bx = (float)xyzExecutor.getAxisParamBeta(0);
			model.traj.by = (float)xyzExecutor.getAxisParamBeta(1);
			model.traj.bz = (float)xyzExecutor.getAxisParamBeta(2);


			model.traj.gx = (float)xyzExecutor.getAxisParamGamma(0);
			model.traj.gy = (float)xyzExecutor.getAxisParamGamma(1);
			model.traj.gz = (float)xyzExecutor.getAxisParamGamma(2);

			model.traj.sx = (float)xyzExecutor.getInitialPosition(0);
			model.traj.sy = (float)xyzExecutor.getInitialPosition(1);
			model.traj.sz = (float)xyzExecutor.getInitialPosition(2);

			model.traj.svx = (float)xyzExecutor.getInitialVelocity(0);
			model.traj.svy = (float)xyzExecutor.getInitialVelocity(1);
			model.traj.svz = (float)xyzExecutor.getInitialVelocity(2);

			model.traj.sax = (float)xyzExecutor.getInitialAcceleration(0);
			model.traj.say = (float)xyzExecutor.getInitialAcceleration(1);
			model.traj.saz = (float)xyzExecutor.getInitialAcceleration(2);

			model.traj.tms = DataModel.getSynchronizedPX4Time_us();

		}




	}



}

