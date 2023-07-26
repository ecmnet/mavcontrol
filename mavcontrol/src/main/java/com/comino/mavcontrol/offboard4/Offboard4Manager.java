package com.comino.mavcontrol.offboard4;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class Offboard4Manager {

	private static Offboard4Manager instance;

	private static final int     UPDATE_RATE                 	    = 20;					    // Offboard update rate in [ms]
	private static final double  SECTION_TIME                 	    = 1.0;					    // Flight time of one trajectory [s]
	private static final float   RADIUS_ACCEPT                      = 0.1f;                     // Acceptance radius in [m]

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
	}
	
	public void planAndExecute(Vector4D_F32 target) {
		worker.setTarget(target);
		worker.start();
	}
	
	public void abort() {
		worker.stop();
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

		// current state
		private final Offboard3Current      current;
		private final GeoTuple4D_F32<?>     current_target;
		private final Offboard4EDF2DPlanner planner;


		// Current trajectory to be executed
		private RapidTrajectoryGenerator  xyzExecutor;


		public Offboard4Worker(IMAVController control) {

			this.control        = control;
			this.model          = control.getCurrentModel();
			this.current        = new Offboard3Current(model);
			this.current_target = new Vector4D_F32();
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

			if(!wq.isInQueue("LP", offboard_worker)) {
				isRunning = true; 
				offboard_worker = wq.addCyclicTask("HP", UPDATE_RATE, this);
			}
		}

		public void stop() {
			this.isRunning       = false;
			this.xyzExecutor     = null;
			this.wq.removeTask("LP", offboard_worker);
			switchToLoiter();

		}
		
		public void setTarget(Vector4D_F32 target) {
			xyzExecutor = null;
			this.current_target.setTo(target.x,target.y,target.z,target.w);
		}

		@Override
		public void run() {

			if(!isRunning)
				return;

			if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
				this.stop(); 
				control.writeLogMessage(new LogMessage("[msp] Offboard externally stopped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				return;
			}
			
			if(isTargetReached() || isTimeOut()) {
				stop();
				return;
			}

			tf = (System.nanoTime() - t_section_start) * NANO_TO_SEC;
			
			System.out.println("TF: "+tf);

			if(xyzExecutor != null && tf < SECTION_TIME) {
				executeCurrentTrajectory(tf);
			} else {
				current.update();
				xyzExecutor = planner.planDirectPath(current_target, SECTION_TIME);
				t_section_start = System.nanoTime();
				
			}
		}
		
		private boolean isTargetReached() {	
			return current.pos().distance(current_target) < RADIUS_ACCEPT;
		}
		
		private boolean isTimeOut() {
			return false;
		}


		private void executeCurrentTrajectory(double tf) {
			
			if(!offboardEnabled ) {
				enableOffboard();
			}

			model.slam.clearFlags();
			synchronized(this) {

				cmd.type_mask        = 0;
				cmd.target_system    = 1;
				cmd.target_component = 1;
				
				cmd.x       = (float)xyzExecutor.getPosition(tf,0);
				cmd.y       = (float)xyzExecutor.getPosition(tf,1);
				cmd.z       = (float)xyzExecutor.getPosition(tf,2);
				
				cmd.vx       = (float)xyzExecutor.getVelocity(tf,0);
				cmd.vy       = (float)xyzExecutor.getVelocity(tf,1);
				cmd.vz       = (float)xyzExecutor.getVelocity(tf,2);
				
				cmd.afx      = (float)xyzExecutor.getAcceleration(tf,0);
				cmd.afy      = (float)xyzExecutor.getAcceleration(tf,1);
				cmd.afz      = (float)xyzExecutor.getAcceleration(tf,2);	
				
				if(isRunning) {
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
					offboardEnabled = true;;
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
		}




	}

	
}

