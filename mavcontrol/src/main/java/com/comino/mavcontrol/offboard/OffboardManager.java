/****************************************************************************
 *
 *   Copyright (c) 2017,2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import org.mavlink.messages.lquac.msg_debug_vect;
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
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.controllib.IYawSpeedControl;
import com.comino.mavcontrol.controllib.IConstraints;
import com.comino.mavcontrol.controllib.ISpeedControl;
import com.comino.mavcontrol.controllib.impl.ContraintControl;
import com.comino.mavcontrol.controllib.impl.SimpleXYZSpeedControl;
import com.comino.mavcontrol.controllib.impl.YawSpeedControl;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;

public class OffboardManager implements Runnable {

	// Offboard modes

	public static final int MODE_INIT                               = 0;
	public static final int MODE_IDLE 		   					    = 1;
	public static final int MODE_LOITER	 		   					= 2;
	public static final int MODE_LOCAL_SPEED                		= 3;
	public static final int MODE_SPEED_POSITION	 	    			= 4;
	public static final int MODE_BEZIER                             = 5;
	public static final int MODE_LAND                               = 6;
	
	// PX4 speed control locks

	private static final int  LOCK_NONE								= 1;
	private static final int  LOCK_Z                                = 2;
	private static final int  LOCK_XY                               = 3;
	private static final int  LOCK_XYZ                              = 4;
	private static final int  LOCK_XYZY                             = 5;


	private static final int  UPDATE_RATE                 			= 50;					  // offboard update rate in ms

	private static final float MAX_YAW_SPEED                		= MSPMathUtils.toRad(30); // Max YawSpeed rad/s
	private static final float MIN_YAW_SPEED                        = MSPMathUtils.toRad(4);  // Min yawSpeed rad/s
	private static final float RAMP_YAW_SPEED                       = MSPMathUtils.toRad(30); // Ramp up Speed for yaw turning
	private static final float MAX_TURN_SPEED               		= 0.3f;   	              // Max speed that allow turning before start in m/s
	private static final float MIN_TURN_DISTANCE              		= 0.6f;   	              // Min distance to path that allow turning

	private static final float MAX_SPEED							= 0.5f;					  // Max speed m/s
	private static final float MIN_SPEED							= 0.1f;					  // Min speed m/s
	
	private static final float LAND_MODE_ALT                        = 0.10f;                  // rel. altitude to switch to PX4 landing 
																							  // Note: Relative to offset of 12cm => 12cm
	
	private static final float LAND_MODE_MIN_SPEED                  = 0.20f;                  // Minimum landing speed in offboard phase



	private static final int   RC_DEADBAND             				= 10;				      // RC XY deadband for safety check

	private static final int SETPOINT_TIMEOUT_MS         			= 75000;

	private static final float YAW_PV								= 0.10f;                  // P factor for yaw speed control
	private static final float YAW_P								= 0.40f;                  // P factor for yaw position control
	private static final float PXY_PV								= 0.90f;                  // P factor for XY adjustment precision landing

	private static final float YAW_ACCEPT                	    	= MSPMathUtils.toRad(0.3);// Acceptance yaw deviation


	private MSPLogger 				logger							= null;
	private DataModel 			    model							= null;
	private IMAVController          control      			       	= null;

	private ITargetAction           action_listener     	        = null;		// CB target reached

	private long                    sent_count                      = 0;

	// ControlLib
	private IYawSpeedControl        yawSpeedControl                  = null;
	private ISpeedControl           speedControl                     = null;		
	private IConstraints            constraintControl                = null;		

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

	private final msg_set_position_target_local_ned pos_cmd   		= new msg_set_position_target_local_ned(1,2);
	private final msg_set_position_target_local_ned speed_cmd 		= new msg_set_position_target_local_ned(1,2);

	private final msg_debug_vect  debug                             = new msg_debug_vect(1,2);

	private float      max_speed                                    = MAX_SPEED;
	private float      min_speed                                    = MIN_SPEED;

	private float	 	acceptance_radius_pos						= 0.10f;
	private float	 	acceptance_radius_pos_out					= MIN_TURN_DISTANCE;
	private boolean    	already_fired			    				= false;
	private boolean    	valid_setpoint                   			= false;
	private boolean    	new_setpoint                   	 			= false;

	private long        setpoint_tms                        		= 0;
	private long        setpoint_timeout                       		= SETPOINT_TIMEOUT_MS;
	private long        trajectory_start_tms                        = 0;
	private long	    last_update_tms                             = 0;
	
	private float       ekf2_min_rng;

	public OffboardManager(IMAVController control, PX4Parameters params) {
		
		this.control        = control;
		this.model          = control.getCurrentModel();
		this.logger         = MSPLogger.getInstance();


		MSPConfig config	= MSPConfig.getInstance();

		acceptance_radius_pos = config.getFloatProperty("autopilot_acceptance_radius", String.valueOf(acceptance_radius_pos));
		System.out.println("Autopilot: acceptance radius: "+acceptance_radius_pos+" m");

		max_speed = config.getFloatProperty("autopilot_max_speed", String.valueOf(max_speed));
		System.out.println("Autopilot: MSP maximum speed: "+max_speed+" m/s");
		
		this.constraintControl = new ContraintControl();
		this.speedControl      = new SimpleXYZSpeedControl(model, min_speed, max_speed);
		this.yawSpeedControl   = new YawSpeedControl(YAW_PV,0,MAX_YAW_SPEED);
		
		this.ekf2_min_rng = params.getParamValue("EKF_MIN_RNG");
		if(Float.isNaN(ekf2_min_rng))
			this.ekf2_min_rng = 0;
		else
		    System.out.println("Autopilot: Use LidarRange when landed: "+ekf2_min_rng+"m");

		MSP3DUtils.setNaN(target);

		// set to manual mode when armed/disarmed
		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			model.slam.clear();
			model.slam.tms = model.sys.getSynchronizedPX4Time_us();

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL, 0 );
		});

	}

	public void start() {
		start(MODE_LOITER);
	}

	public void start_wait(long timeout) {
		start_wait(MODE_LOITER, timeout);
	}

	public void start(int m) {
		changeStateTo(m);
		setpoint_timeout = SETPOINT_TIMEOUT_MS;
		if(!enabled) {
			enabled = true;
			Thread t = new Thread(this);
			t.start();
		}
	}

	public void setMaxSpeed(double speed_limit) {
		if(max_speed > speed_limit) {
			max_speed = (float)speed_limit;
			System.out.println("Autopilot: maximum speed limited to PX4 speed: "+max_speed+" m/s");
		} 
	}

	public boolean start_wait(int m, long timeout) {
		long tstart = System.currentTimeMillis();
		if(!valid_setpoint) {
			MSP3DUtils.convertCurrentState(model, current);
			setTarget(current);
		}
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
			//	}
		}
		return true;
	}

	public void abort() {
		if(!enabled)
			return;
		already_fired = false;
		if(action_listener!=null)
			changeStateTo(MODE_LOITER);
		synchronized(this) {
			notify();
		}
		logger.writeLocalMsg("[msp] Offboard action aborted. Loitering.",MAV_SEVERITY.MAV_SEVERITY_INFO);
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
		this.setTarget(t.x,t.y,t.z,t.w);
	}

	public void setTarget(float x, float y, float z, float yaw) {
		synchronized(this) {
			if(!MSP3DUtils.isNaN(target))
				current_sp.set(target);
			target.set(x,y,z,yaw);
			valid_setpoint = true;
			new_setpoint = true;
			already_fired = false;
			setpoint_tms = System.currentTimeMillis();
		}
	}

	public void setSpeed(Vector4D_F32 t) {
		synchronized(this) {
			target_speed.set(t);
			valid_setpoint = true;
			new_setpoint = true;
			already_fired = false;
			setpoint_tms = System.currentTimeMillis();
		}
	}

	public void enforceCurrentAsTarget() {
		this.setTarget(Float.NaN,Float.NaN,Float.NaN,Float.NaN);
	}

	public void updateTarget(Vector4D_F32 t) {

		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT))
			return;

		this.mode = MODE_SPEED_POSITION;
		target.set(t);
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
	
	public void registerContraintControl(IConstraints control) {
		this.constraintControl = control;
		
	}
	
	public void registerSpeedControl(ISpeedControl control) {
		this.speedControl = control;
		
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

	@Override
	public void run() {

		long watch_tms = System.currentTimeMillis();

		float delta_sec  = 0;
		float eta_sec    = 0;
		float ela_sec    = 0;

		float tmp        = 0;
		float max        = 0;

		Polar3D_F32 path = new Polar3D_F32(); // planned direct path
		Polar3D_F32 way  = new Polar3D_F32(); // travelled direct path
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

			if(old_mode != mode) {
				if(MSP3DUtils.isNaN(target)) {
					MSP3DUtils.convertCurrentState(model, target);
					logger.writeLocalMsg("[msp] Offboard: Using current as target",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				}
				//	logger.writeLocalMsg("[msp] Offboard: Switched to "+mode_string[mode],MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				old_mode = mode;
			}

			if(model.sys.isStatus(Status.MSP_RC_ATTACHED) && !safety_check()) {
				enabled = false;
				continue;
			}

			if(valid_setpoint && (System.currentTimeMillis()-watch_tms ) > setpoint_timeout ) {
				valid_setpoint = false; mode = MODE_LOITER;

				if(model.sys.nav_state == Status.NAVIGATION_STATE_OFFBOARD)
					logger.writeLocalMsg("[msp] Setpoint not reached. Loitering.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			}

			MSP3DUtils.convertCurrentState(model, current);
			MSP3DUtils.convertCurrentSpeed(model, spd);

			// safety: if no valid setpoint, use current as target
			if(!valid_setpoint && mode != MODE_IDLE) {
				target.set(current);
				new_setpoint = true;
				valid_setpoint = true;
				changeStateTo(MODE_LOITER);
				continue;
			}

			// a new setpoint was provided
			if(new_setpoint) {

				new_setpoint = false;
				speedControl.initialize(spd, path);
				trajectory_start_tms = 0; ela_sec = 0;
				start.set(current);
				ctl.set(spd);

				if(mode==MODE_SPEED_POSITION || mode == MODE_LOITER) {

					// Safety: handle NaN targets for position
					//	MSP3DUtils.replaceNaN(target, current_sp);
					// if still not valid use current
					MSP3DUtils.replaceNaN(target, current);

				}

			}

			delta_sec = (System.currentTimeMillis() - last_update_tms ) / 1000.0f;
			last_update_tms = System.currentTimeMillis();

			switch(mode) {

			case MODE_INIT:
				
				model.slam.flags = Slam.OFFBOARD_FLAG_NONE;
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_LOITER_SETPOINT_TYPE);
				break;
				
			case MODE_IDLE:
				
				model.slam.flags = Slam.OFFBOARD_FLAG_NONE;
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_IDLE_SETPOINT_TYPE);
				break;
				
			case MODE_LOITER:	// Loiter at current position, yaw controlled
				
				model.slam.flags = Slam.OFFBOARD_FLAG_HOLD;
				watch_tms = System.currentTimeMillis();

				if(Float.isNaN(target.w)) {
					target.setW(MSPMathUtils.normAngle(model.attitude.y));
				}

				yaw_diff = MSPMathUtils.normAngle(target.w - current.w);

				if(valid_setpoint && Math.abs(yaw_diff) < YAW_ACCEPT) {
					d_yaw=0;
					fireAction(model, path.value);
				}

				if(!already_fired) {

					// simple P controller for yaw with ramp up
					// TODO: Still required?
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


					// limit min yaw speed if action not fired yet
					if(Math.abs(d_yaw)< MIN_YAW_SPEED)
						d_yaw = MIN_YAW_SPEED * Math.signum(d_yaw_target);

					if(Math.abs(d_yaw)>MAX_YAW_SPEED)
						d_yaw = MAX_YAW_SPEED * Math.signum(d_yaw_target);

					cmd.set(target.x,target.y,target.z, current.w + d_yaw);

				} else {

					cmd.set(target.x,target.y,target.z, target.w);
				}

				sendPositionControlToVehice(cmd, MAV_FRAME.MAV_FRAME_LOCAL_NED);

				updateMSPModel(target,null);

				break;

			case MODE_LOCAL_SPEED: 	// Direct speed control
				
				model.slam.flags = Slam.OFFBOARD_FLAG_SPEED;
				path.set(target_speed.x, target_speed.y, target_speed.z);

				lock = LOCK_NONE;

				if(Float.isNaN(target_speed.z))
					lock = LOCK_Z;

				if(Float.isNaN(target_speed.x) && Float.isNaN(target_speed.y))
					lock = LOCK_XY;

				if(Float.isNaN(target_speed.x) && Float.isNaN(target_speed.y) && Float.isNaN(target_speed.z))
					lock = LOCK_XYZ;

				//TODO: Take current speed into account and control acceleration

				cmd.set(target_speed);

				//TODO: check external constraints (breaking, emergency stop)

				sendSpeedControlToVehice(cmd, target, MAV_FRAME.MAV_FRAME_LOCAL_NED,lock);


				if((System.currentTimeMillis()- setpoint_tms) > 1000)
					valid_setpoint = false;
				else
					watch_tms = System.currentTimeMillis();

				updateMSPModel(null,spd);

				break;

			case MODE_SPEED_POSITION:  // Speed controlled position adjustment

				watch_tms = System.currentTimeMillis();
				path.set(target, current);
				way.set(current, start);

				if(trajectory_start_tms > 0)
					ela_sec = (System.currentTimeMillis() - trajectory_start_tms) / 1000f;

				eta_sec = (path.value - acceptance_radius_pos / 2f ) / spd.value;

				// target reached?
				if(path.value < acceptance_radius_pos && valid_setpoint && !already_fired
						// TODO: Triggers when radius is reached => middle never hit (especially: altitude)
						// same for POSITION Mode
						// How to do that?
						) {

					// clear everything
					trajectory_start_tms = 0;
					//					path.clear(); ctl.clear();

					if(Float.isNaN(target.w) && valid_setpoint) {
						fireAction(model, path.value);
						target.setW(model.attitude.y);
						continue;
					} else {
						changeStateTo(MODE_LOITER);
						continue;
					}
				}

				// Yaw difference - do not consider path direction if slope steeper than 45°
				if(( path.angle_xz >  0.78f || path.angle_xz < -0.78 ))
					if(!Float.isNaN(target.w))
						// only if target attitude was given
						yaw_diff = MSPMathUtils.normAngle(target.w - current.w);
					else
						yaw_diff = 0;
				else {
					yaw_diff = MSPMathUtils.normAngle(ctl.angle_xy - current.w);
				}

				// if vehicle is not moving or close to target and turn angle > 60° => turn before moving
				if( Math.abs(yaw_diff) > Math.PI/3 &&
						ctl.value < MAX_TURN_SPEED && 
						way.value < acceptance_radius_pos_out && 
						path.value > acceptance_radius_pos_out) {

					model.slam.flags = Slam.OFFBOARD_FLAG_TURN;
					//path.value > MIN_TURN_DISTANCE) {
					// reduce XY speeds
					ctl.value = 0;
					//  Lock XYZ Position while turning
					lock = LOCK_XYZ;

				} else {
					// external speed control via control callback
					speedControl.update(delta_sec, ela_sec, eta_sec, spd, path, ctl);
					// check external constraints
					constraintControl.update(delta_sec, spd, path, ctl);
					lock = LOCK_NONE;
				}

				if(ctl.value > 0 && trajectory_start_tms == 0)
					trajectory_start_tms = System.currentTimeMillis();

				if(path.value > acceptance_radius_pos_out) {
					cmd.w = yawSpeedControl.update(yaw_diff, delta_sec);
				} else
					// if near the target do not control yaw anymore
					cmd.w = 0;

				// get Cartesian speeds from polar
				ctl.get(cmd);

				sendSpeedControlToVehice(cmd, current_sp, MAV_FRAME.MAV_FRAME_LOCAL_NED, lock);

				updateMSPModel(target,path);

				break;

			case MODE_BEZIER:	// Not implemented yet

				logger.writeLocalMsg("[msp] Offboard manager bezier mode not supported",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				changeStateTo(MODE_LOITER);

				break;


			case MODE_LAND:    	// Performs an altitude controlled landing using precision lock for pos and yaw if available
				
				ctl.clear(); 
				valid_setpoint = true;
				watch_tms = System.currentTimeMillis();

				target.set(model.vision.px,model.vision.py,model.state.l_z,model.vision.pw);
				
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
				if(max < 0.10f) max = 0.10f;


				// No XY adjustment anymore if height < 0.07m or no fiducial 
				if(tmp < LAND_MODE_ALT || MSP3DUtils.hasNaN(target)) {
					ctl.value = 0; 
					ctl.get(cmd);
					lock = LOCK_XY;
				}
				else {

					last_target.set(target);

					// Simple P controller to adjust XY according to fiducial
					ctl.value =  ctl.value * PXY_PV;

					// Maximum adjustment speed is determined max speed or absolute value of 0.3m/s
					if(ctl.value > max) ctl.value = max;
					if(ctl.value > 0.4) ctl.value = 0.4f;

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


				debug.x = (float)ctl.value;
				debug.y = (float)yaw_diff;
				debug.z = (float)tmp;

				control.sendMAVLinkMessage(debug);

				// Once in turnmode, stay there
				if(tmp < LAND_MODE_ALT) {
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0,Float.NaN );		
						stop();
						logger.writeLocalMsg("[msp] Accurracy: "+String.format("% #.2fm [%#.1f°]",path.value, MSPMathUtils.fromRad2(yaw_diff)
								),MAV_SEVERITY.MAV_SEVERITY_DEBUG);
						continue;
				} else {
					model.slam.flags = Slam.OFFBOARD_FLAG_LAND;
					sendSpeedControlToVehice(cmd, current_sp, MAV_FRAME.MAV_FRAME_LOCAL_NED, lock);
					updateMSPModel(target,path);
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
		already_fired = false; valid_setpoint = false; new_setpoint = false;

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


	private void sendPositionControlToVehice(Vector4D_F32 target, int frame) {

		pos_cmd.target_component = 1;
		pos_cmd.target_system    = 1;
		
		pos_cmd.type_mask        = MAV_MASK.MASK_VELOCITY_IGNORE | MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
	                    		   MAV_MASK.MASK_YAW_RATE_IGNORE ;
			
		pos_cmd.x   = target.x;
		pos_cmd.y   = target.y;
		pos_cmd.z   = target.z;
		
		if(Float.isInfinite(target.w)) {
			pos_cmd.type_mask  = pos_cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
			pos_cmd.yaw = model.attitude.y;
		} else
			pos_cmd.yaw = MSPMathUtils.normAngle(target.w);

		pos_cmd.coordinate_frame = frame;


		if(!control.sendMAVLinkMessage(pos_cmd))
			enabled = false;

	}

	private void sendSpeedControlToVehice(Vector4D_F32 target, Vector4D_F32 lock, int frame, int lock_mode) {

		speed_cmd.target_component = 1;
		speed_cmd.target_system    = 1;
		speed_cmd.time_boot_ms     = model.sys.t_boot_ms;

		checkAbsoluteSpeeds(target);

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


		if(!control.sendMAVLinkMessage(speed_cmd))
			enabled = false;
		else
			sent_count++;

	}

	private void checkAbsoluteSpeeds(Vector4D_F32 s) {

		if(s.x >  max_speed )  s.x =  max_speed;
		if(s.y >  max_speed )  s.y =  max_speed;
		if(s.z >  max_speed )  s.z =  max_speed;
		if(s.x < -max_speed )  s.x = -max_speed;
		if(s.y < -max_speed )  s.y = -max_speed;
		if(s.z < -max_speed )  s.z = -max_speed;

		if(s.w >   MAX_YAW_SPEED )  s.w =   MAX_YAW_SPEED;
		if(s.w <  -MAX_YAW_SPEED )  s.w =  -MAX_YAW_SPEED;

	}

	private void fireAction(DataModel model,float delta) {
		if(already_fired)

			return;
		if(action_listener!= null)
			action_listener.action(model, delta);
		synchronized(this) {
			already_fired = true;
			notify();
		}

	}

	private void updateMSPModel(Vector4D_F32 target, Polar3D_F32 path ) {

		model.slam.tms = model.sys.getSynchronizedPX4Time_us();

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


}
