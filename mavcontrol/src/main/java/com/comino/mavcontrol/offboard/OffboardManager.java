/****************************************************************************
 *
 *   Copyright (c) 2017,2019 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard.control.DefaultConstraintListener;
import com.comino.mavcontrol.offboard.control.DefaultControlListener;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F32;

public class OffboardManager implements Runnable {

	// Offboard modes

	public static final int MODE_INIT                               = 0;
	public static final int MODE_IDLE 		   					    = 1;
	public static final int MODE_LOITER	 		   					= 2;
	public static final int MODE_LOCAL_SPEED                		= 3;
	public static final int MODE_SPEED_POSITION	 	    			= 4;
	public static final int MODE_BEZIER                             = 5;

	private static final String[] mode_string                       = { "INIT", "IDLE", "LOITER","SPEED","SPEEDPOS","BEZIER" };

	private static final int  LOCK_NONE								= 0;
	private static final int  LOCK_Z                                = 1;
	private static final int  LOCK_XY                               = 2;
	private static final int  LOCK_XYZ                              = 3;

	private static final int  UPDATE_RATE                 			= 50;					  // offboard update rate in ms
	private static final long OFFBOARD_INIT_DELAY                   = 2*UPDATE_RATE;		  // initial delay


	private static final float MAX_YAW_SPEED                		= MSPMathUtils.toRad(45); // Max YawSpeed rad/s
	private static final float MIN_YAW_SPEED                        = MSPMathUtils.toRad(4);  // Min yawSpeed rad/s
	private static final float RAMP_YAW_SPEED                       = MSPMathUtils.toRad(30); // Ramp up Speed for yaw turning
	private static final float MAX_TURN_SPEED               		= 0.3f;   	              // Max speed that allow turning before start in m/s
	private static final float MIN_TURN_DISTANCE              		= 0.3f;   	              // Min distance to path target that allow z-Locking
	private static final float MAX_SPEED							= 1.0f;					  // Max speed m/s

	private static final int   RC_DEADBAND             				= 20;				      // RC XY deadband for safety check
	private static final int   RC_LAND_CHANNEL						= 8;                      // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD            		= 2010;		              // RC channel 8 landing threshold


	private static final int SETPOINT_TIMEOUT_MS         			= 75000;

	private static final float YAW_PV								= 0.10f;                  // P factor for yaw speed control
	private static final float YAW_P								= 0.40f;                  // P factor for yaw position control

	private static final float YAW_ACCEPT                	    	= MSPMathUtils.toRad(0.3);// Acceptance yaw deviation



	//

	private MSPLogger 				 logger							= null;
	private DataModel 				 model							= null;
	private IMAVController         	 control      			       	= null;

	private IOffboardTargetAction        action_listener     	    = null;		// CB target reached
	private IOffboardExternalControl     ext_control_listener       = null;		// CB external angle+speed control in MODE_SPEED_POSITION
	private IOffboardExternalConstraints ext_constraints_listener   = null;		// CB Constrains in MODE_SPEED_POSITION

	private boolean					enabled					  		= false;
	private int						mode					  		= MODE_LOITER;		     // Offboard mode
	private int                     old_mode                        = MODE_IDLE;

	private final Vector4D_F32		target					  		= new Vector4D_F32();	 // target state (coordinates/speeds) incl. yaw
	private final Vector4D_F32		current					  		= new Vector4D_F32();	 // current state incl. yaw
	private final Vector4D_F32		current_sp					  	= new Vector4D_F32();	 // current state incl. yaw
	private final Vector4D_F32      start                     		= new Vector4D_F32();    // state, when setpoint was set incl. yaw
	private final Vector4D_F32		cmd			  	            	= new Vector4D_F32();    // vehicle command state (coordinates/speeds)

	private final msg_set_position_target_local_ned pos_cmd   		= new msg_set_position_target_local_ned(1,2);
	private final msg_set_position_target_local_ned speed_cmd 		= new msg_set_position_target_local_ned(1,2);
	private final msg_debug_vect  debug                             = new msg_debug_vect(1,2);

	private float      max_speed                                    = MAX_SPEED;

	private float	 	acceptance_radius_pos						= 0.10f;
	private boolean    	already_fired			    				= false;
	private boolean    	valid_setpoint                   			= false;
	private boolean    	new_setpoint                   	 			= false;

	private long        setpoint_tms                        		= 0;
	private long        setpoint_timeout                       		= SETPOINT_TIMEOUT_MS;
	private long        trajectory_start_tms                        = 0;
	private long	    last_update_tms                             = 0;

	public OffboardManager(IMAVController control) {
		this.control        = control;
		this.model          = control.getCurrentModel();
		this.logger         = MSPLogger.getInstance();

		this.ext_constraints_listener = new DefaultConstraintListener();
		this.ext_control_listener  = new DefaultControlListener();
		//	this.ext_control_listener  = new TimebasedControlListener();

		MSPConfig config	= MSPConfig.getInstance();

		acceptance_radius_pos = config.getFloatProperty("autopilot_acceptance_radius", String.valueOf(acceptance_radius_pos));
		System.out.println("Autopilot: acceptance radius: "+acceptance_radius_pos+" m");

		max_speed = config.getFloatProperty("autopilot_max_speed", String.valueOf(max_speed));
		System.out.println("Autopilot: maximum speed: "+max_speed+" m/s");

		MSP3DUtils.setNaN(target);

		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			model.slam.clear();
			model.slam.tms = model.sys.getSynchronizedPX4Time_us();

			// set to manual mode when armed/disarmed
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
		mode = m;
		setpoint_timeout = SETPOINT_TIMEOUT_MS;
		if(!enabled) {
			enabled = true;
			new Thread(this).start();
			System.out.println("Offboard updater started..");
			try { Thread.sleep(OFFBOARD_INIT_DELAY); } catch (InterruptedException e) { }
		}

	}

	public boolean start_wait(int m, long timeout) {
		start(m);
		synchronized(this) {
			if(!already_fired) {
				long tstart = System.currentTimeMillis();
				try { 	wait(timeout); } catch (InterruptedException e) { }
				if((System.currentTimeMillis() - tstart) >= timeout)
					return false;
			}
		}
		return true;
	}

	public void abort() {
		already_fired = false;
		if(action_listener!=null)
			mode = MODE_LOITER;
		synchronized(this) {
			notify();
		}
	}


	public void stop() {
		//	Thread.dumpStack();
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


	public void registerExternalControlListener(IOffboardExternalControl control_listener) {
		this.ext_control_listener = control_listener;
	}

	public void registerExternalConstraintsListener(IOffboardExternalConstraints constraints) {
		this.ext_constraints_listener = constraints;
	}

	public boolean isEnabled() {
		return enabled;
	}

	public int getMode() {
		return mode;
	}

	public void registerActionListener(IOffboardTargetAction listener) {
		this.action_listener = listener;
	}

	public void removeActionListener() {
		this.action_listener = null;
	}

	public IOffboardTargetAction getActionListener() {
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

		while(enabled) {

			if(old_mode != mode) {
				old_mode = mode;
				logger.writeLocalMsg("[msp] Offboard: Switched to "+mode_string[mode],MAV_SEVERITY.MAV_SEVERITY_DEBUG);
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
			path.set(target, current);

			// safety: if no valid setpoint, use current as target
			if(!valid_setpoint && mode != MODE_IDLE && mode<7 ) {
				target.set(current_sp);
				new_setpoint = true;
				valid_setpoint = true;
				mode = MODE_LOITER;
				continue;
			}

			// a new setpoint was provided
			if(new_setpoint) {

				new_setpoint = false;
				ext_control_listener.initialize(spd, path);
				trajectory_start_tms = 0; ela_sec = 0;
				start.set(current);
				ctl.set(spd);

				// Safety: handle NaN targets for position
				MSP3DUtils.replaceNaN(target, current_sp);
				// if still not valid use current
				MSP3DUtils.replaceNaN(target, current);

			}

			delta_sec = (System.currentTimeMillis() - last_update_tms ) / 1000.0f;
			last_update_tms = System.currentTimeMillis();

			switch(mode) {

			case MODE_INIT:
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_LOITER_SETPOINT_TYPE);
				break;
			case MODE_IDLE:
				watch_tms = System.currentTimeMillis();
				sendTypeControlToVehice(MAV_MASK.MASK_IDLE_SETPOINT_TYPE);
				break;
			case MODE_LOITER:
				watch_tms = System.currentTimeMillis();

				if(Float.isNaN(target.w)) {
					target.setW(MSPMathUtils.normAngle2(model.attitude.y));
				}

				yaw_diff = MSPMathUtils.normAngle2(target.w - current.w);

				if(valid_setpoint && Math.abs(yaw_diff) < YAW_ACCEPT) {
					d_yaw=0;
					fireAction(model, path.value);
				}

				if(!already_fired) {

					//  simple P controller for yaw with ramp up
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
				toModel(target,null);
				break;

			case MODE_LOCAL_SPEED:

				ctl.set(target.x, target.y, target.z);
				path.angle_xy = ctl.angle_xy;

				//TODO: check external constraints,

				//				// check external constraints
				ext_constraints_listener.get(delta_sec, spd, path, ctl);

				ctl.get(cmd);

				// manual yaw control
				cmd.w = target.w;

				// use Z-Lock for XY control
				// TODO: Check up/down control speed -> remove Z-Lock

				sendSpeedControlToVehice(cmd, current_sp, MAV_FRAME.MAV_FRAME_LOCAL_NED,LOCK_Z);
				//toModel(spd.value,target,current);

				if((System.currentTimeMillis()- setpoint_tms) > 1000)
					valid_setpoint = false;
				else
					watch_tms = System.currentTimeMillis();
				break;

			case MODE_SPEED_POSITION:

				watch_tms = System.currentTimeMillis();

				if(trajectory_start_tms > 0)
					ela_sec = (System.currentTimeMillis() - trajectory_start_tms) / 1000f;

				eta_sec = (path.value - acceptance_radius_pos / 2f ) / spd.value;

				// external speed control via control callback ?
				ext_control_listener.determineSpeedAnDirection(delta_sec, ela_sec, eta_sec, spd, path, ctl);


				// check external constraints
				ext_constraints_listener.get(delta_sec, spd, path, ctl);

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
							mode = MODE_LOITER;
							continue;
						}
				}

				// Yaw difference - do not consider path direction if slope steeper than 45°
				if(( path.angle_xz >  0.78f || path.angle_xz < -0.78 ))
					if(!Float.isNaN(target.w))
						// only of target attitude was given
						yaw_diff = MSPMathUtils.normAngle2(target.w - current.w);
					else
						yaw_diff = 0;
				else
					yaw_diff = MSPMathUtils.normAngle2(ctl.angle_xy - current.w);

				// if vehicle is not moving or close to target and turn angle > 60° => turn before moving
				if( Math.abs(yaw_diff) > Math.PI/3 &&
						ctl.value < MAX_TURN_SPEED  &&  path.value > MIN_TURN_DISTANCE) {
					// reduce XY speeds
					ctl.value = 0;
					//  Lock XYZ Position while turning
					lock = LOCK_XYZ;
				} else
					lock = LOCK_NONE;

				if(ctl.value > 0 && trajectory_start_tms == 0)
					trajectory_start_tms = System.currentTimeMillis();

				//  simple P controller for yaw
				d_yaw_target = yaw_diff / delta_sec * YAW_PV;

				// ramp up/down yaw speed
				if(d_yaw_target > 0) {
					d_yaw = d_yaw + (RAMP_YAW_SPEED*delta_sec);
					if(d_yaw > d_yaw_target)
						d_yaw = d_yaw_target;
				} else if(d_yaw_target < 0)  {
					d_yaw = d_yaw - (RAMP_YAW_SPEED*delta_sec);
					if(d_yaw < d_yaw_target)
						d_yaw = d_yaw_target;
				}
				cmd.w = d_yaw;

				// do not go below min yaw speed
				if(Math.abs(cmd.w)< MIN_YAW_SPEED) cmd.w = MIN_YAW_SPEED * Math.signum(cmd.w);

				// get Cartesian speeds from polar
				ctl.get(cmd);

				debug.x = yaw_diff;
				debug.y = ctl.value;
				debug.z = cmd.z;

				sendSpeedControlToVehice(cmd, current_sp, MAV_FRAME.MAV_FRAME_LOCAL_NED, lock);

				toModel(target,path);

				break;

			case MODE_BEZIER:

				logger.writeLocalMsg("[msp] Offboard manager bezier mode not supported",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				mode = MODE_LOITER;

				break;

			}

			try { Thread.sleep(UPDATE_RATE); 	} catch (InterruptedException e) { }
		}

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

	private void sendTypeControlToVehice(int mask) {

		pos_cmd.target_component = 1;
		pos_cmd.target_system    = 1;
		pos_cmd.type_mask        = mask;

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
		pos_cmd.yaw = Float.isNaN(target.w)? model.attitude.y : MSPMathUtils.normAngle(target.w);

		pos_cmd.coordinate_frame = frame;


		if(!control.sendMAVLinkMessage(pos_cmd))
			enabled = false;

	}

	private void sendSpeedControlToVehice(Vector4D_F32 target, Vector4D_F32 lock, int frame, int lock_mode) {

		speed_cmd.target_component = 1;
		speed_cmd.target_system    = 1;

		switch(lock_mode) {
		case LOCK_NONE:
			speed_cmd.type_mask    = MAV_MASK.MASK_POSITION_IGNORE | MAV_MASK.MASK_ACCELERATION_IGNORE | MAV_MASK.MASK_FORCE_IGNORE |
			MAV_MASK.MASK_YAW_IGNORE ;

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
			speed_cmd.y        = lock.z;

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


		// safety constraints
		checkAbsoluteSpeeds(target);


		speed_cmd.yaw_rate = target.w;

		speed_cmd.coordinate_frame = frame;


		if(!control.sendMAVLinkMessage(speed_cmd))
			enabled = false;

		control.sendMAVLinkMessage(debug);

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

	private void toModel(Vector4D_F32 target, Polar3D_F32 path ) {
		if(valid_setpoint && path!=null) {
			model.slam.px = target.getX();
			model.slam.py = target.getY();
			model.slam.pz = target.getZ();
			model.slam.pd = path.angle_xy;
			model.slam.di = path.value;
			model.slam.pv = model.state.getSpeed();
		} else {
			model.slam.px = Float.NaN;
			model.slam.py = Float.NaN;
			model.slam.pz = Float.NaN;
			model.slam.pd = Float.NaN;
			model.slam.di = 0;
			model.slam.pv = 0;
		}
		model.slam.tms = model.sys.getSynchronizedPX4Time_us();

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

		// TODO: Move to Autopilot -->

		// Safety: Channel 8 (Mid) triggers landing mode of PX4
		if(Math.abs(model.rc.get(RC_LAND_CHANNEL) - RC_LAND_THRESHOLD) < RC_DEADBAND) {
			logger.writeLocalMsg("[msp] Emergency landing triggered",MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 2, 0.05f );
			return false;
		}
		return true;

	}

}
