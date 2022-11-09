/****************************************************************************
 *
 *   Copyright (c) 2017,2022 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.mavcontrol.offboard;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.mavlink.MAV_MASK;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.autopilot.actions.SequencerActionFactory;
import com.comino.mavcontrol.controllib.IYawSpeedControl;
import com.comino.mavcontrol.controllib.impl.YawSpeedControl;
import com.comino.mavcontrol.ekf2utils.EKF2ResetCheck;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector2D_F32;
import georegression.struct.point.Vector4D_F32;

public class OffboardManager implements Runnable {

	// Offboard modes

	public static final int MODE_INIT                               = 0;
	public static final int MODE_IDLE 		   					    = 1;
	public static final int MODE_LOITER	 		   					= 2;
	public static final int MODE_LOCAL_SPEED                		= 3;
	public static final int MODE_TRAJECTORY                         = 5;
	public static final int MODE_LAND                               = 6;

	// PX4 speed control locks

	private static final int  LOCK_NONE								= 1;
	private static final int  LOCK_Z                                = 2;
	private static final int  LOCK_XY                               = 3;
	private static final int  LOCK_XYZ                              = 4;


	private static final int  UPDATE_RATE                 			= 40;					  // offboard update rate in ms

	private static final float MAX_YAW_SPEED                		= MSPMathUtils.toRad(60); // Max YawSpeed rad/s
	private static final float MIN_YAW_SPEED                        = MSPMathUtils.toRad(10); // Min yawSpeed rad/s
	private static final float RAMP_YAW_SPEED                       = MSPMathUtils.toRad(30); // Ramp up Speed for yaw turning
	private static final float MAX_TURN_SLOPE                       = MSPMathUtils.toRad(85); // Max slope to turn into papth direction

	private static final float MAX_SPEED							= 0.75f;			      // Max speed m/s
	private static final float MAX_SPEED_SIM						= 5.00f;			      // Max speed m/s
	private static final float MIN_SPEED							= 0f;					  // Min speed m/s

	private static final float LAND_MODE_ALT                        = 0.10f;                  // rel. altitude to switch to PX4 landing 

	private static final float LAND_MODE_MIN_SPEED                  = 0.20f;                  // Minimum landing speed (SP Z) in offboard phase

	private static final int   MIN_TRAJ_TIME             		    = 6;				      // Minimum time a single trajectory is planned for
	private static final int   TRAJ_TIMESTEPS                       = 10;                     // Steps to increase trajectory length

	private static final int   RC_DEADBAND             				= 10;				      // RC XY deadband for safety check

	private static final int SETPOINT_TIMEOUT_MS         			= 180000;

	private static final float YAW_PV								= 0.10f;                  // P factor for yaw speed control
	private static final float YAW_P								= 0.40f;                  // P factor for yaw position control
	private static final float PXY_PV								= 2.00f;                  // P factor for XY adjustment fiducial control
	private static final float PXY_MAX                              = 0.30f;                  // Maximum speed fiducial adjustment

	private static final float YAW_ACCEPT                	    	= MSPMathUtils.toRad(0.3);// Acceptance yaw deviation


	private MSPLogger 				logger							= null;
	private DataModel 			    model							= null;
	private IMAVController          control      			       	= null;

	private ITargetAction           action_listener     	        = null;		// CB target reached

	private long                    sent_count                      = 0;

	// ControlLib
	private IYawSpeedControl        yawSpeedControl                  = null;	

	private boolean					enabled					  		= false;
	private int						mode					  		= MODE_LOITER;		     // Offboard mode
	private int                     old_mode                        = MODE_IDLE;

	private final Vector4D_F32		target					  		= new Vector4D_F32();	 // target state
	private final Vector4D_F32		target_speed					= new Vector4D_F32();	 // target state
	private final Vector4D_F32		last_target					  	= new Vector4D_F32();	 // target state

	private final Vector4D_F32		current					  		= new Vector4D_F32();	 // current state incl. yaw
	private final Vector4D_F32		current_sp					  	= new Vector4D_F32();	 // current state incl. yaw
	private final Vector4D_F32      start                     		= new Vector4D_F32();    // state, when setpoint was set incl. yaw
	private final Vector4D_F32		cmd			  	            	= new Vector4D_F32();    // vehicle command state (coordinates/speeds)


	private final msg_set_position_target_local_ned pos_cmd   		= new msg_set_position_target_local_ned(1,1);
	private final msg_set_position_target_local_ned speed_cmd 		= new msg_set_position_target_local_ned(1,1);

	private float      max_speed                                    = MAX_SPEED;
	private float      ekf2_min_rng                                 = 0;
	private float      traj_length_s                                      = 0;

	private long traj_eta  = 0;
	private long traj_sta  = 0;

	private float	 	acceptance_radius_std						= 0.1f;
	private float	 	acceptance_radius				        	= 0;
	private boolean    	already_fired			    				= false;
	private boolean    	valid_setpoint                   			= false;
	private boolean    	new_setpoint                   	 			= false;

	private boolean             is_fiducial                          = false;
	private final  Vector2D_F32 fiducial_delta			    	    = new Vector2D_F32(0,0);	 // Fiducial delta when detected


	private long        setpoint_tms                        		= 0;
	private long        setpoint_timeout                       		= SETPOINT_TIMEOUT_MS;
	private long	    last_update_tms                             = 0;

	private final RapidTrajectoryGenerator traj                     = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

	private final Point3D_F64 traj_acc							 	= new Point3D_F64(0,0,0);
	private final Point3D_F64 traj_vel								= new Point3D_F64(0,0,0);
	private final Point3D_F64 traj_pos								= new Point3D_F64(0,0,0);
	private final Point3D_F64 debug								    = new Point3D_F64(0,0,0);

	private boolean check_acceptance_radius                         = false;  // Do not consider acceptance radiua but time only

	private final LocalMap3D map;
	private final EKF2ResetCheck ekf2_reset_check;

	public OffboardManager(IMAVController control,  EKF2ResetCheck ekf2_reset_check, LocalMap3D map, PX4Parameters params) {

		this.control        = control;
		this.map            = map;
		this.model          = control.getCurrentModel();
		this.logger         = MSPLogger.getInstance();

		this.ekf2_reset_check = ekf2_reset_check;

		this.target.setTo(Float.NaN,Float.NaN,Float.NaN,Float.NaN);

		MSPConfig config	= MSPConfig.getInstance();

		acceptance_radius_std = config.getFloatProperty("autopilot_acceptance_radius", String.valueOf(acceptance_radius_std));

		System.out.println("Autopilot: acceptance radius: "+acceptance_radius_std+" m");

		max_speed = config.getFloatProperty("autopilot_max_speed", String.valueOf(max_speed));
		if(control.isSimulation())
			max_speed = MAX_SPEED_SIM;

		this.yawSpeedControl   = new YawSpeedControl(YAW_PV,0,MAX_YAW_SPEED);

		MSP3DUtils.setNaN(target);

		// set to manual mode when armed/disarmed
		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			model.slam.clear(); model.traj.clear();
			model.slam.tms = DataModel.getSynchronizedPX4Time_us();
			updateTrajectoryModel(traj_length_s,-1);

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL, 0 );
		});

		// Get some PX4 parameters used for offboard control
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED)) {	

				float tmp    = params.getParamValue("MPC_XY_VEL_MAX", max_speed);
				if(tmp < max_speed) {
					max_speed = tmp;
					System.out.println("Set maximum autopilot XY speed to "+max_speed+" m/s");
				}

				ekf2_min_rng = params.getParamValue("EKF2_MIN_RNG", 0);
				if(ekf2_min_rng > 0)
					System.out.println("Use EKF2_MIN_RNG of "+ekf2_min_rng+"m for precision landing");

			}
		});

	}

	public void start() {
		start(MODE_LOITER);
	}

	public void start_wait(long timeout) {
		start_wait(MODE_LOITER, false, timeout);
	}

	public void startTrajectory(Vector4D_F32 tgt) {
		target.setTo(tgt);
		new_setpoint = true;
		valid_setpoint = true;
		check_acceptance_radius = true;
		changeStateTo(MODE_TRAJECTORY);
		if(!enabled) {
			enabled = true;
			Thread t = new Thread(this);
			t.setPriority(Thread.MAX_PRIORITY);
			t.start();
		}


	}

	public void start(int m) {
		changeStateTo(m);
		setpoint_timeout = SETPOINT_TIMEOUT_MS;
		if(!enabled) {
			enabled = true;
			Thread t = new Thread(this);
			t.setPriority(Thread.MAX_PRIORITY);
			t.start();
		}
	}

	public boolean start_wait(int m, boolean wait, long timeout) {
		long tstart = System.currentTimeMillis();
		if(!valid_setpoint) {
			MSP3DUtils.convertCurrentPosition(model, current);
			setTarget(current);
		}
		check_acceptance_radius = wait;
		start(m);
		if(!model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
			control.writeLogMessage(new LogMessage("[msp] Try to switch to offboard.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			while(sent_count < 5) {
				try { Thread.sleep(UPDATE_RATE); } catch (InterruptedException e) { }
				if((System.currentTimeMillis() - tstart) > 500) {
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed (Send timeout).", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					enabled = false;
					return false;
				}
			}

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					stop();
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
							MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
							MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
		}
		synchronized(this) {
			//	if(!already_fired) {
			try { 	wait(timeout); } catch (InterruptedException e) { }
			if((System.currentTimeMillis() - tstart) >= timeout) {
				System.out.println("Offboard wait timeout");
				return false;
			}
		}
		return true;
	}

	public void abort() {
		abort(true);
	}

	public void abort(boolean loiter) {

		if(!enabled)
			return;

		already_fired = false;
		if(action_listener!=null && loiter) {
			changeStateTo(MODE_LOITER);
			logger.writeLocalMsg("[msp] Offboard action aborted. Loitering.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		} 

		synchronized(this) {
			notify();
		}
	}


	public void stop() {
		valid_setpoint = false;
		model.slam.flags = Slam.OFFBOARD_FLAG_NONE;
		enabled = false;
		synchronized(this) {
			notify();
		}
	}

	public void setTarget(Vector4D_F32 t) {
		this.setTarget(t.x,t.y,t.z,t.w,0);
	}

	public void setTarget(Vector4D_F32 t, float ar) {
		this.setTarget(t.x,t.y,t.z,t.w,ar);
	}

	public void setTarget(float x, float y, float z, float yaw, float ar) {	
		if(!MSP3DUtils.isNaN(target)) 
			current_sp.setTo(target);

		target.setTo(x,y,z,yaw);
		acceptance_radius = ar > 0 ? ar : acceptance_radius_std;

		valid_setpoint = true;
		new_setpoint   = true;
		already_fired  = false;
		setpoint_tms   = System.currentTimeMillis();

	}

	public void setSpeed(Vector4D_F32 t) {
		synchronized(this) {
			target_speed.setTo(t);
			valid_setpoint = true;
			new_setpoint = true;
			already_fired = false;
			setpoint_tms = System.currentTimeMillis();
		}
	}

	public void enforceCurrentAsTarget() {
		this.setTarget(Float.NaN,Float.NaN,Float.NaN,Float.NaN,0);
	}

	public void updateTarget(Vector4D_F32 t) {

		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT))
			return;

		this.mode = MODE_TRAJECTORY;
		target.setTo(t);
		valid_setpoint = true;
		already_fired = false;
		setpoint_tms = System.currentTimeMillis();
	}

	public Vector4D_F32 getCurrentTarget() {
		return target;
	}


	public void finalize() {
		this.action_listener  = null;
		synchronized(this) {
			notify();
		}
	}


	public boolean isEnabled() {
		return enabled;
	}

	public int getMode() {
		return mode;
	}

	public void registerYawSpeedControl(IYawSpeedControl control) {
		this.yawSpeedControl = control;

	}

	public void registerActionListener(ITargetAction listener) {
		this.action_listener = listener;
	}

	public void removeActionListener() {
		this.action_listener = null;
	}

	public ITargetAction getActionListener() {
		return action_listener;
	}

	public boolean hasTarget() {
		return valid_setpoint;
	}

	public boolean isFired() {
		return already_fired;
	}

	public void reSendCurrentSetpoint() {
		// resend current setpoint for EKF2 check
		
		if(!valid_setpoint || !enabled)
			return;
		
		switch(mode) {
		case MODE_LOITER:
			sendPositionControlToVehice(cmd, MAV_FRAME.MAV_FRAME_LOCAL_NED,true);
			break;
		case MODE_TRAJECTORY:
			sendTrajectoryControlToVehice(traj_pos,traj_vel,traj_acc,0);
			break;
		}	
	}

	@Override
	public void run() {

		long watch_tms = System.currentTimeMillis();
		long current_tms = System.currentTimeMillis();

		double traj_tim = 0;


		float delta_sec  = 0;

		float tmp        = 0;
		float max        = 0;

		Polar3D_F32 path = new Polar3D_F32(); // planned direct path
		Polar3D_F32 spd  = new Polar3D_F32(); // current speed
		Polar3D_F32 ctl  = new Polar3D_F32(); // speed control

		float d_yaw = 0, d_yaw_target = 0, yaw_diff = 0;
		int lock = LOCK_NONE;

		already_fired = false; if(!new_setpoint) valid_setpoint = false;

		logger.writeLocalMsg("[msp] Offboard manager started",MAV_SEVERITY.MAV_SEVERITY_DEBUG);

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.OFFBOARD_UPDATER, true);

		last_update_tms = System.currentTimeMillis();

		MSP3DUtils.convertTargetState(model, current_sp);

		yawSpeedControl.reset();

		while(enabled) {


			current_tms = System.currentTimeMillis();

			if(old_mode != mode) {
				if(MSP3DUtils.isNaN(target)) {
					MSP3DUtils.convertCurrentPosition(model, target);
					logger.writeLocalMsg("[msp] Offboard: Using current as target",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				}
				//	logger.writeLocalMsg("[msp] Offboard: Switched to "+mode_string[mode],MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				old_mode = mode;
			}

			if(model.sys.isStatus(Status.MSP_RC_ATTACHED) && !safety_check()) {
				logger.writeLocalMsg("[msp] Offboard: Disabled ",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				enabled = false;
				continue;
			}

			if(valid_setpoint && (System.currentTimeMillis()-watch_tms ) > setpoint_timeout ) {
				valid_setpoint = false; mode = MODE_LOITER;
				// TODO: Switch to PX4 HOLD or other action if this occurs multiple times

				if(model.sys.nav_state == Status.NAVIGATION_STATE_OFFBOARD)
					logger.writeLocalMsg("[msp] Setpoint not reached. Loitering.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			}

			MSP3DUtils.convertCurrentPosition(model, current);
			MSP3DUtils.convertCurrentSpeed(model, spd);

			// safety: if no valid setpoint, use current as target
			if(!valid_setpoint && mode != MODE_IDLE) {
				target.setTo(current);
				new_setpoint = true;
				valid_setpoint = true;
				changeStateTo(MODE_LOITER);
				continue;
			}


			// a new setpoint was provided
			if(new_setpoint) {

				ekf2_reset_check.reset(false);

				switch(mode) {
				case MODE_TRAJECTORY:
					MSP3DUtils.replaceNaN3D(target, current);

					MSP3DUtils.convertCurrentPosition(model, current);

					if(Float.isNaN(target.z))
						target.z = current.z;

					// TODO: Too simple for long distances: Max speed will not be reached
					traj_length_s = MSP3DUtils.distance3D(target, current) * (float)Math.PI / ( max_speed );
					traj_length_s = traj_length_s < MIN_TRAJ_TIME ? MIN_TRAJ_TIME : traj_length_s;

					traj_eta = doTrajectoryPlanning(current_tms, traj_length_s);

					// Increase trajectory length by 10% as long as not feasible (max 100 %)
					// Note: This is not the only possible measure: If a subsequent target exists,
					//       the speed of the target state could be modified as well
					int count = 0;
					while(traj_eta < 0 && count++ < TRAJ_TIMESTEPS) {
						traj_length_s = traj_length_s * (1.0f+1.0f/TRAJ_TIMESTEPS);
						traj_eta = doTrajectoryPlanning(current_tms, traj_length_s);
					}

					if(traj_eta < 0) {
						control.writeLogMessage(new LogMessage("[msp] No valid trajectory generated. Loitering.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
						check_acceptance_radius = false;
						fireAction(model, path.value);
						mode = MODE_LOITER;
						target.setTo(current);
						continue;

					}
					break;

				case MODE_LOITER:
					MSP3DUtils.replaceNaN3D(target, current);
				}

				if(mode==MODE_TRAJECTORY || mode == MODE_LOITER) {

					// Safety: handle NaN targets for position
					//	MSP3DUtils.replaceNaN(target, current_sp);
					// if still not valid use current
					MSP3DUtils.replaceNaN3D(target, current);			

				}

				new_setpoint = false;
				start.setTo(current);
				ctl.set(spd);

			}

			delta_sec = (System.currentTimeMillis() - last_update_tms ) / 1000.0f;
			last_update_tms = System.currentTimeMillis();

			switch(mode) {

			case MODE_INIT:

				is_fiducial = false; 
				model.slam.flags = Slam.OFFBOARD_FLAG_NONE;
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_LOITER_SETPOINT_TYPE);
				break;

			case MODE_IDLE:

				is_fiducial = false; 
				model.slam.flags = Slam.OFFBOARD_FLAG_NONE;
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_IDLE_SETPOINT_TYPE);
				break;

			case MODE_LOITER:	// Loiter at current position, yaw controlled

				
				watch_tms = System.currentTimeMillis();

				if(Float.isNaN(target.w)) {
					target.setW(MSPMathUtils.normAngle(model.attitude.y));
				}

				// Compensate drift via fiducial
				// TODO: To be tested

				//				if(control.isSimulation())
				//					StandardActionFactory.simulateFiducial(control,0.1f);

				//				if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) { 
				//                // Control XY loitering by fiducial if visible 
				//					
				//					fiducial_delta.setTo(target.x - model.vision.px,target.y - model.vision.py);
				//					if(!is_fiducial) {
				//						logger.writeLocalMsg("[msp] Drift compensation active",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				//						is_fiducial = true;
				//					}
				//
				//				} else {
				//					if(is_fiducial) {
				//						target.setTo(model.vision.px+fiducial_delta.x,model.vision.py+fiducial_delta.y,target.z,target.w);
				//						logger.writeLocalMsg("[msp] Drift compensation deactivated",MAV_SEVERITY.MAV_SEVERITY_INFO);
				//					fiducial_delta.setTo(0,0);
				//					is_fiducial = false; 
				//					}
				//				} else {
				//					fiducial_delta.setTo(0,0);
				//				}

				// Control XY loitering by original target
				yaw_diff = MSPMathUtils.normAngle(target.w - current.w);

				if(valid_setpoint && Math.abs(yaw_diff) < YAW_ACCEPT) {
					d_yaw=0;
					fireAction(model, path.value);
				}

				if(!already_fired) {

					d_yaw_target = yaw_diff * YAW_P;
					if(d_yaw_target > 0) {
						d_yaw = d_yaw + (RAMP_YAW_SPEED * delta_sec);
						if(d_yaw > d_yaw_target)
							d_yaw = d_yaw_target;
					} else if(d_yaw_target < 0) {
						d_yaw = d_yaw - (RAMP_YAW_SPEED * delta_sec );
						if(d_yaw < d_yaw_target)
							d_yaw = d_yaw_target;
					}


					if(Math.abs(d_yaw)>MAX_YAW_SPEED)
						d_yaw = MAX_YAW_SPEED * Math.signum(d_yaw_target);

					// limit min yaw speed if action not fired yet
					if(Math.abs(d_yaw)< MIN_YAW_SPEED)
						d_yaw = MIN_YAW_SPEED * Math.signum(d_yaw_target);

				} else {
					d_yaw = 0;
				}

				if(is_fiducial && !(Float.isNaN(model.vision.px) || Float.isNaN(model.vision.py)))
					cmd.setTo(model.vision.px+fiducial_delta.x,model.vision.py+fiducial_delta.y,target.z, target.w+d_yaw);
				else
					cmd.setTo(target.x,target.y,target.z, target.w+d_yaw);


				//				if(control.isSimulation()) {
				//					// in SITL: Send corrected setpoints and report offsets
				//					model.debug.x = (float)(reset_offset.x);
				//					model.debug.y = (float)(reset_offset.y);
				//					model.debug.z = (float)(reset_offset.z);
				//
				//					cmd.x = cmd.x + reset_offset.x;
				//					cmd.y = cmd.y + reset_offset.y;
				//					cmd.z = cmd.z + reset_offset.z;
				//
				//				} else {
				//					// on vehicle: Send original setpoints and report corrected setpoint
				//					model.debug.x = (float)(cmd.x + reset_offset.x);
				//					model.debug.y = (float)(cmd.y + reset_offset.y);
				//					model.debug.z = (float)(cmd.z + reset_offset.z);
				//
				//				}

				sendPositionControlToVehice(cmd, MAV_FRAME.MAV_FRAME_LOCAL_NED, false);
				updateSLAMModel(target,null);

				break;

			case MODE_LOCAL_SPEED: 	// Direct speed control via Joystick
				// TODO: Use PX4 for this directly

				is_fiducial = false; 

				
				path.set(target_speed.x, target_speed.y, target_speed.z);

				lock = LOCK_NONE;

				if(Float.isNaN(target_speed.z))
					lock = LOCK_Z;

				if(Float.isNaN(target_speed.x) && Float.isNaN(target_speed.y))
					lock = LOCK_XY;

				if(Float.isNaN(target_speed.x) && Float.isNaN(target_speed.y) && Float.isNaN(target_speed.z))
					lock = LOCK_XYZ;

				//TODO: Take current speed into account and control acceleration

				cmd.setTo(target_speed);

				//TODO: check external constraints (breaking, emergency stop)

				sendSpeedControlToVehice(cmd, target, MAV_FRAME.MAV_FRAME_LOCAL_NED,lock);


				if((System.currentTimeMillis()- setpoint_tms) > 1000)
					valid_setpoint = false;
				else
					watch_tms = System.currentTimeMillis();

				updateSLAMModel(null,spd);

				break;

			case MODE_TRAJECTORY:	

				watch_tms = current_tms;

				is_fiducial = false; 
				path.set(target, current);
				if((path.value < acceptance_radius && check_acceptance_radius) || current_tms > traj_eta) {
					if(current_tms < traj_eta) {
						fireAction(model, path.value);
					} else {
						check_acceptance_radius = false;
						// use current target aas target for LOITER
						valid_setpoint = true;
						fireAction(model, path.value);
						changeStateTo(MODE_LOITER);	
						updateTrajectoryModel(traj_length_s,-1);
						logger.writeLocalMsg("[msp] Target reached.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
						continue;

					}
				}

				
				traj_tim = (current_tms-traj_sta)/1000d;
				traj.getState(traj_tim, traj_pos, traj_vel, traj_acc);

				if(!Float.isNaN(target.w))
					yaw_diff = MSPMathUtils.normAngle(target.w - current.w);
				else
					yaw_diff = MSPMathUtils.normAngle(MSP3DUtils.angleXY(traj_vel) - current.w);

				if(Math.abs(MSP3DUtils.angleXZ(target, current)) > MAX_TURN_SLOPE)
					yaw_diff = 0;


				// In SITL correct trajectory by reset_offset
				// TODO: How to simulate GPS glitch

				//				if(control.isSimulation()) {
				//					// in SITL: Send corrected setpoints and report offsets
				//					model.debug.x = (float)(reset_offset.x);
				//					model.debug.y = (float)(reset_offset.y);
				//					model.debug.z = (float)(reset_offset.z);
				//
				//					traj_pos.x = traj_pos.x + reset_offset.x;
				//					traj_pos.y = traj_pos.y + reset_offset.y;
				//					traj_pos.z = traj_pos.z + reset_offset.z;
				//
				//				} else {
				//					// on vehicle: Send original setpoints and report corrected setpoint
				//					model.debug.x = (float)(traj_pos.x + reset_offset.x);
				//					model.debug.y = (float)(traj_pos.y + reset_offset.y);
				//					model.debug.z = (float)(traj_pos.z + reset_offset.z);
				//				}

				sendTrajectoryControlToVehice(traj_pos,traj_vel,traj_acc,yawSpeedControl.update(yaw_diff, delta_sec));

				updateTrajectoryModel(traj_length_s, (float)traj_tim);
				updateSLAMModel(target,path);

				break;


			case MODE_LAND:    	// Performs an altitude controlled landing using precision lock for pos and yaw if available
				// NOT USED; Sequencer instead

				is_fiducial = false; 

				ctl.clear(); 
				valid_setpoint = true;
				watch_tms = System.currentTimeMillis();

				if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED))
					target.setTo(model.vision.px,model.vision.py,current.z,model.vision.pw);
				else
					target.setTo(current.x,current.y,current.z,current.w);

				// TODO: Safetychecks: Pitch/Roll => gain height and loiter

				yaw_diff = MSPMathUtils.normAngle(target.w - current.w);

				path.set(target, current); 
				ctl.set(target, current);

				if(Float.isFinite(model.hud.at))
					tmp =  Math.abs(model.hud.al - model.hud.at) - ekf2_min_rng;
				else
					tmp = model.hud.al - ekf2_min_rng;


				// Calculate max XY adjustment speed depending on the height and current z-speed
				// but minimum max speed is 0.10m/s
				max = 3.0f * path .value * model.state.l_vz / tmp ;
				if(max < MIN_SPEED) max = MIN_SPEED;


				// No XY adjustment anymore if height < 0.07m or no fiducial 
				if(tmp < LAND_MODE_ALT || MSP3DUtils.hasNaN(target)) {
					ctl.value = 0; 
					ctl.get(cmd);
					lock = LOCK_XY;
				}
				else {

					last_target.setTo(target);

					// Simple P controller to adjust XY according to fiducial
					ctl.value =  ctl.value * PXY_PV;

					// Maximum adjustment speed is determined max speed or absolute value of 0.3m/s
					if(ctl.value > max) ctl.value = max;
					if(ctl.value > PXY_MAX) ctl.value = PXY_MAX;

					ctl.get(cmd);
					lock = LOCK_NONE;
				}

				cmd.z = tmp / 2.75f; // Eventually adjust to 2.5 (increases z speed)

				// between 2.0 and 0.2m try to turn into fiducial orientation
				if(tmp > 0.2 && tmp < 2f ) {
					cmd.w = yawSpeedControl.update(yaw_diff, delta_sec);
				}
				else {
					cmd.w = 0;
				}


				// Check minimum landing speed
				if(cmd.z < LAND_MODE_MIN_SPEED) cmd.z = LAND_MODE_MIN_SPEED;

				// Todo: Check abort condition and abort landing if necessary (gain height of 1m and loiter)
				//       e.g. if landing target not reachable, or current z-speed is upwards

				// Once in turnmode, stay there
				if(tmp < LAND_MODE_ALT) {
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0, Float.NaN);		
					stop();
					logger.writeLocalMsg("[msp] Accurracy: "+String.format("% #.2fm [%#.1f°]",path.value, MSPMathUtils.fromRadSigned(yaw_diff)
							),MAV_SEVERITY.MAV_SEVERITY_DEBUG);
					continue;
				} else {
					
					sendSpeedControlToVehice(cmd, current_sp, MAV_FRAME.MAV_FRAME_LOCAL_NED, lock);
					updateSLAMModel(target,path);
				}

				break;

			}

			try { Thread.sleep(UPDATE_RATE); 	} catch (InterruptedException e) { }

		}

		sent_count = 0;


		yawSpeedControl.reset();

		action_listener = null;
		abort();

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.OFFBOARD_UPDATER, false);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.INTERACTIVE, false);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_AVOIDANCE, false);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP, false);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.RTL, false);


		logger.writeLocalMsg("[msp] Offboard manager stopped",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		already_fired = false; valid_setpoint = false; new_setpoint = false; is_fiducial = false;

	}

	private void changeStateTo(int mode) {
		yawSpeedControl.reset();
		this.mode = mode;

	}

	private void sendTypeControlToVehice(int mask) {

		pos_cmd.target_component = 1;
		pos_cmd.target_system    = 1;
		pos_cmd.type_mask        = mask;

		pos_cmd.x   = Float.NaN;
		pos_cmd.y   = Float.NaN;
		pos_cmd.z   = Float.NaN;


		if(!control.sendMAVLinkMessage(pos_cmd))
			enabled = false;

	}


	private void sendPositionControlToVehice(Vector4D_F32 target, int frame, boolean ignore_yaw) {

		pos_cmd.target_component = 1;
		pos_cmd.target_system    = 1;


		if(control.isSimulation()) {
			pos_cmd.x   = target.x + model.est.l_x_reset;
			pos_cmd.y   = target.y + model.est.l_y_reset;
			pos_cmd.z   = target.z + model.est.l_z_reset;
		} else {
			pos_cmd.x   = target.x;
			pos_cmd.y   = target.y;
			pos_cmd.z   = target.z;
		}
		
		pos_cmd.vx  = 0;
		pos_cmd.vy  = 0;
		pos_cmd.vz  = 0;
		pos_cmd.afx  = 0;
		pos_cmd.afy  = 0;
		pos_cmd.afz  = 0;


		if(Float.isInfinite(target.w) || ignore_yaw) {
			pos_cmd.type_mask  = pos_cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
			pos_cmd.yaw = model.attitude.y;
		} else
			pos_cmd.yaw = MSPMathUtils.normAngle(target.w);

		pos_cmd.coordinate_frame = frame;
		pos_cmd.time_boot_ms = model.sys.t_boot_ms;


		if(!control.sendMAVLinkMessage(pos_cmd))
			enabled = false;

	}

	private void sendSpeedControlToVehice(Vector4D_F32 target, Vector4D_F32 lock, int frame, int lock_mode) {

		speed_cmd.target_component = 1;
		speed_cmd.target_system    = 1;
		speed_cmd.time_boot_ms     = model.sys.t_boot_ms;

		switch(lock_mode) {


		case LOCK_NONE:
			speed_cmd.type_mask    = MAV_MASK.MASK_POSITION_IGNORE | MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
			MAV_MASK.MASK_YAW_IGNORE;

			speed_cmd.vx       = target.x;
			speed_cmd.vy       = target.y;
			speed_cmd.vz       = target.z;

			break;

		case LOCK_Z:
			speed_cmd.type_mask    = MAV_MASK.MASK_POSITION_IGNORE_ZLOCK | MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
			MAV_MASK.MASK_YAW_IGNORE ;

			speed_cmd.vx       = target.x;
			speed_cmd.vy       = target.y;
			speed_cmd.vz       = 0;

			speed_cmd.z        = lock.z;

			break;	

		case LOCK_XY:

			speed_cmd.type_mask    = MAV_MASK.MASK_POSITION_IGNORE_XYLOCK | MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
			MAV_MASK.MASK_YAW_IGNORE ;

			speed_cmd.vx       = 0;
			speed_cmd.vy       = 0;
			speed_cmd.vz       = target.z;

			speed_cmd.x        = lock.x;
			speed_cmd.y        = lock.y;

			break;

		case LOCK_XYZ:

			speed_cmd.type_mask    = MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
			MAV_MASK.MASK_YAW_IGNORE ;

			speed_cmd.vx       = 0;
			speed_cmd.vy       = 0;
			speed_cmd.vz       = 0;

			speed_cmd.x        = lock.x;
			speed_cmd.y        = lock.y;
			speed_cmd.z        = lock.z;

			break;
		}

		speed_cmd.yaw_rate = target.w;


		speed_cmd.coordinate_frame = frame;

		speed_cmd.time_boot_ms = model.sys.t_boot_ms;
		if(!control.sendMAVLinkMessage(speed_cmd))
			enabled = false;
		else
			sent_count++;

	}


	private void sendTrajectoryControlToVehice(Point3D_F64 pos, Point3D_F64 vel, Point3D_F64 acc, float w) {

		//		speed_cmd.type_mask    = MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_YAW_IGNORE;
		speed_cmd.type_mask    = MAV_MASK.MASK_YAW_IGNORE;

		if(control.isSimulation()) {
			speed_cmd.x       = (float)pos.x + model.est.l_x_reset;
			speed_cmd.y       = (float)pos.y + model.est.l_y_reset;
			speed_cmd.z       = (float)pos.z + model.est.l_z_reset;
		}	 else {

			speed_cmd.x       = (float)pos.x;
			speed_cmd.y       = (float)pos.y;
			speed_cmd.z       = (float)pos.z;
		}

		speed_cmd.vx       = (float)vel.x;
		speed_cmd.vy       = (float)vel.y;
		speed_cmd.vz       = (float)vel.z;

		speed_cmd.afx       = (float)acc.x;
		speed_cmd.afy       = (float)acc.y;
		speed_cmd.afz       = (float)acc.z;

		speed_cmd.time_boot_ms = model.sys.t_boot_ms;
		speed_cmd.isValid  = true;

		if(Float.isInfinite(w)) {
			speed_cmd.type_mask  = speed_cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
			speed_cmd.yaw = model.attitude.y;
		} else
			speed_cmd.yaw_rate = MSPMathUtils.normAngle(w);

		speed_cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
		control.sendMAVLinkMessage(speed_cmd);

	}

	private void fireAction(DataModel model,float delta) {
		if(already_fired)
			return;

		synchronized(this) {
			already_fired = true;
			notify();
		}
		if(action_listener!= null)
			action_listener.action(model, delta);
	}

	private void updateSLAMModel(Vector4D_F32 target, Polar3D_F32 path ) {

		model.slam.tms = DataModel.getSynchronizedPX4Time_us();

		if(valid_setpoint && path!=null && target!=null) {
			model.slam.px = target.getX();
			model.slam.py = target.getY();
			model.slam.pz = target.getZ();
			model.slam.pd = path.angle_xy;
			model.slam.di = path.value;
			model.slam.pv = model.state.getXYSpeed();
			return;
		}

		if(valid_setpoint && path!=null && target==null) {
			model.slam.px = Float.NaN;
			model.slam.py = Float.NaN;
			model.slam.pz = Float.NaN;
			model.slam.pd = path.angle_xy;
			model.slam.di = Float.NaN;
			model.slam.pv = model.state.getXYSpeed();
			return;
		}

		if(valid_setpoint && path==null && target!=null) {
			model.slam.px = target.getX();
			model.slam.py = target.getY();
			model.slam.pz = target.getZ();
			model.slam.pv = 0;
			return;
		}

		model.slam.px = Float.NaN;
		model.slam.py = Float.NaN;
		model.slam.pz = Float.NaN;
		model.slam.pd = Float.NaN;
		model.slam.di = Float.NaN;
		model.slam.pv = 0;

	}
	private boolean safety_check() {

		if(Math.abs(model.rc.s1 -1500) > RC_DEADBAND || Math.abs(model.rc.s2 -1500) > RC_DEADBAND) {
			abort();
			logger.writeLocalMsg("[msp] OffboardUpdater stopped: RC",MAV_SEVERITY.MAV_SEVERITY_INFO);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL, 0 );
			return false;
		}

		return true;
	}

	public float getETA() {
		return (float)traj_length_s;
	}

	public long doTrajectoryPlanning( long tms, float d_time) {

		if(!traj.generate(d_time, model, target, null)) {
			return -1;
		}

		//		if(doCollisionCheck(d_time) > 0)
		//			return - 1;

		System.out.println("Generate trajectory: "+String.format("%#.1fs with costs of %#.2f", d_time, traj.getCost()*100));

		traj_sta = tms;
		traj_eta = (long)(d_time * 1000f) + traj_sta;

		// Send trajectory to MAVGCL
		updateTrajectoryModel(d_time, 0);

		return traj_eta;

	}

	private double doCollisionCheck(float total_time) {

		Point3D_F64 speed    = new Point3D_F64();
		Point3D_F64 position = new Point3D_F64();
		double delta_t = 0; double speed_n = 0; double p=0;

		while(delta_t < total_time) {
			traj.getVelocity(delta_t, speed);
			speed_n = speed.norm();
			if(speed_n < 0.01)
				return -1;
			delta_t = delta_t + 0.1 / speed_n;

			//		  if(delta_t > 5)
			//			  map.update(position);

			traj.getPosition(delta_t, position);
			if(map.check(position) > 0.5) {
				System.err.println("Collsion detected in "+delta_t+" secs at "+position);
				return -1;
			}

		}

		return -1;
	}

	private void updateTrajectoryModel(float length, float current) {

		// TODO: Consider reset_offsets also in model to plot them correctly

		if(length > 0) {

			model.traj.ls = length;
			model.traj.fs = current;
			model.traj.ax = (float)traj.getAxisParamAlpha(0);
			model.traj.ay = (float)traj.getAxisParamAlpha(1);
			model.traj.az = (float)traj.getAxisParamAlpha(2);


			model.traj.bx = (float)traj.getAxisParamBeta(0);
			model.traj.by = (float)traj.getAxisParamBeta(1);
			model.traj.bz = (float)traj.getAxisParamBeta(2);


			model.traj.gx = (float)traj.getAxisParamGamma(0);
			model.traj.gy = (float)traj.getAxisParamGamma(1);
			model.traj.gz = (float)traj.getAxisParamGamma(2);

			model.traj.sx = (float)traj.getInitialPosition(0);
			model.traj.sy = (float)traj.getInitialPosition(1);
			model.traj.sz = (float)traj.getInitialPosition(2);

			model.traj.svx = (float)traj.getInitialVelocity(0);
			model.traj.svy = (float)traj.getInitialVelocity(1);
			model.traj.svz = (float)traj.getInitialVelocity(2);

			model.traj.tms = DataModel.getSynchronizedPX4Time_us();

		} else {

			model.traj.clear();

		}

	}



}
