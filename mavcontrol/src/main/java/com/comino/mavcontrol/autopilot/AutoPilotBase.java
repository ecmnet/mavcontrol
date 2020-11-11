package com.comino.mavcontrol.autopilot;

import java.util.LinkedList;
import java.util.ListIterator;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import org.mavlink.messages.MAV_BATTERY_CHARGE_STATE;

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

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_micro_slam;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.autopilot.tests.PlannerTest;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavcontrol.struct.ISeqAction;
import com.comino.mavcontrol.struct.SeqItem;
import com.comino.mavmap.map.map2D.ILocalMap;
import com.comino.mavmap.map.map2D.filter.ILocalMapFilter;
import com.comino.mavmap.map.map2D.filter.impl.ForgetMapFilter;
import com.comino.mavmap.map.map2D.impl.LocalMap2DRaycast;
import com.comino.mavmap.map.map2D.store.LocaMap2DStorage;
import com.comino.mavodometry.estimators.ITargetListener;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.legacy.ExecutorService;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F32;


/**
 * @author ecmnet
 *
 */
/**
 * @author ecmnet
 *
 */
/**
 * @author ecmnet
 *
 */
public abstract class AutoPilotBase implements Runnable, ITargetListener {

	/* TEST ONLY */
	private PlannerTest planner = null;

	protected static final int   CERTAINITY_THRESHOLD  = 100;
	protected static final float WINDOWSIZE       	   = 3.0f;

	protected static final float MAX_REL_DELTA_HEIGHT  = 0.10f;
	protected static final float MAX_TAKEOFF_VZ        = 0.1f;

	private static final int   RC_DEADBAND             = 20;				      // RC Deadband
	private static final int   RC_LAND_CHANNEL		   = 8;                       // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD       = 2010;		              // RC channel 8 landing threshold

	private static AutoPilotBase  autopilot    = null;

	protected DataModel                     model    = null;
	protected MSPLogger                     logger   = null;
	protected IMAVController                control  = null;
	protected ILocalMap				        map      = null;
	protected OffboardManager               offboard = null;
	protected PX4Parameters                 params   = null;

	protected boolean			           mapForget = false;
	protected boolean                      flowCheck = false;

	protected boolean                      isRunning = false;
	protected boolean               emergencyLanding = false;

	protected LinkedList<SeqItem>           sequence = null;
	protected LinkedList<SeqItem>           appended = null;

	protected final Vector4D_F32             takeoff = new Vector4D_F32();
	protected  long                       takeoff_ms = 0;

	protected ILocalMapFilter              mapFilter = null;

	private final Vector4D_F32            body_speed = new Vector4D_F32();
	private final Vector4D_F32            ned_speed  = new Vector4D_F32();

	private final msg_msp_micro_slam            slam = new msg_msp_micro_slam(2,1);


	private Future<?> future;


	public static AutoPilotBase getInstance(String clazz, IMAVController control,MSPConfig config) {
		if(autopilot == null)
			try {
				autopilot =(AutoPilotBase)Class.forName(clazz).getDeclaredConstructor(IMAVController.class,MSPConfig.class).newInstance(control,config);
			} catch (Exception e) {
				e.printStackTrace();
			}
		return autopilot;
	}

	public static AutoPilotBase getInstance() {
		return autopilot;
	}


	public AutoPilotBase(IMAVController control, MSPConfig config) {

		/* TEST ONLY */
		this.planner = new PlannerTest(control,config);

		String instanceName = this.getClass().getSimpleName();

		System.out.println(instanceName+" instantiated");

		this.control  = control;
		this.model    = control.getCurrentModel();
		this.logger   = MSPLogger.getInstance();
		this.params   = PX4Parameters.getInstance();
		this.sequence = new LinkedList<SeqItem>();
		this.appended = new LinkedList<SeqItem>();
		
		this.offboard = new OffboardManager(control);

		this.map      = new LocalMap2DRaycast(model,WINDOWSIZE,CERTAINITY_THRESHOLD);
		this.mapForget = config.getBoolProperty("autopilot_forget_map", "true");
		System.out.println(instanceName+": Map forget enabled: "+mapForget);
		if(mapForget)
			registerMapFilter(new ForgetMapFilter());

		this.flowCheck = config.getBoolProperty("autopilot_flow_check", "true") & !control.isSimulation();
		System.out.println(instanceName+": FlowCheck enabled: "+flowCheck);

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE,
				config.getBoolProperty("autopilot_takeoff_procedure", "false"));

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,
				config.getBoolProperty("autopilot_precision_lock", "false"));
		
		// Register actions

		registerTakeoff();
		registerLanding();
		registerDisarm();
		registerLowBattery();
		
		// Limit offboard max speed to PX4 speed limit
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED)) {	
				final ParameterAttributes  speed_limit_param = params.getParam("MPC_XY_VEL_MAX");
				offboard.setMaxSpeed(speed_limit_param.value);
			}
		});



	}
	
	protected void registerLowBattery() {
		control.getStatusManager().addListener(StatusManager.TYPE_BATTERY, MAV_BATTERY_CHARGE_STATE.MAV_BATTERY_CHARGE_STATE_LOW, (n) -> {
			logger.writeLocalMsg("[msp] Battery low procedure triggered",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			
			// Different actions depending on the current mode, e.g.
			// Shutdown MSP, Switch off SLAM, RTL, Landing, etc
			
		});
		
		control.getStatusManager().addListener(StatusManager.TYPE_BATTERY, MAV_BATTERY_CHARGE_STATE.MAV_BATTERY_CHARGE_STATE_CRITICAL, (n) -> {
			logger.writeLocalMsg("[msp] Battery critical procedure triggered",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
			
			// Different actions depending on the current mode, e.g.
			// Shutdown MSP, Switch off SLAM, RTL, Landing, etc
			
		});
	}

	protected void registerDisarm() {

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_ARMED, StatusManager.EDGE_FALLING, (n) -> {

			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			takeoff_ms = 0; emergencyLanding = false;
			if(future!=null) future.cancel(true);

			if(offboard.isEnabled()) {
				offboard.abort(); offboard.stop();
				control.writeLogMessage(new LogMessage("[msp] Switched to manual mode.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
						MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL, 0 );
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.RTL, false);

				control.sendMAVLinkMessage(new msg_debug_vect(1,2));
			}
		});

	}

	protected void registerLanding() {

		// Abort any sequence if PX4 landing is triggered
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_LAND, StatusManager.EDGE_RISING, (n) -> {
			abortSequence();
			if(future!=null) future.cancel(true);
		});

	}

	protected void registerTakeoff() {

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_TAKEOFF, StatusManager.EDGE_RISING, (n) -> {

			final ParameterAttributes  takeoff_alt_param   = params.getParam("MIS_TAKEOFF_ALT");
			final ParameterAttributes  takeoff_speed_param = params.getParam("MPC_TKO_SPEED");

			// calculate maximum takeoff time
			final int max_tko_time_ms = (int)(takeoff_alt_param.value / takeoff_speed_param.value ) * 1000 + 15000;

			control.writeLogMessage(new LogMessage("[msp] Takeoff proecdure initiated.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

			long takeoff_start_tms = System.currentTimeMillis();
			double delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;


			// Phase 1: Wait for height is in range
			while(delta_height > MAX_REL_DELTA_HEIGHT) {
				try { Thread.sleep(50); } catch(Exception e) { }
				delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;
				if((System.currentTimeMillis() - takeoff_start_tms) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					return;
				}
			}

			// Phase 2: Wait for VZ is small enough to switch to offboard
			while(Math.abs(model.state.l_vz) > MAX_TAKEOFF_VZ) {
				try { Thread.sleep(50); } catch(Exception e) { }
				if((System.currentTimeMillis() - takeoff_start_tms) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					return;
				}
			}


			// Phase 3: Switch to offboard in MODE_INIT
			offboard.start();
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					offboard.stop();
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
							MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
							MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );

			control.writeLogMessage(new LogMessage("[msp] Setting takeoff position.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			this.takeoff.set(model.state.l_x,model.state.l_y,model.state.l_z,0);

			try { Thread.sleep(200); } catch(Exception e) { }
			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE))
				this.takeoffCompletedAction();

		});

	}


	protected void takeoffCompletedAction() {

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP, true);
		control.writeLogMessage(new LogMessage("[msp] Obstacle survey executed.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
		rotate(45,() -> {
			control.writeLogMessage(new LogMessage("[msp] Takeoff procedure completed.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			return true;
		});

	}

	protected void addToSequence(SeqItem item) {
		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
			sequence.add(item);
		else
			appended.add(item);
	}

	//	protected void appendToRunningSequence(SeqItem item) {
	//		appended.add(item);
	//	}

	protected void clearSequence() {
		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			sequence.clear(); appended.clear();
			model.slam.wpcount = 0;
		}
	}

	protected void abortSequence() {
		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			sequence.clear();  appended.clear();
			model.slam.wpcount = 0;
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
			offboard.abort();
			while(!future.isDone());
		}
	}

	protected void executeSequence(SeqItem item, ISeqAction completedAction) {
		sequence.clear();  appended.clear();
		sequence.add(item);
		executeSequence();
	}

	protected void executeSequence() {
		executeSequence((ISeqAction)null);
	}

	protected void executeSequence(ISeqAction completedAction) {


		if(!offboard.isEnabled()) {
			logger.writeLocalMsg("[msp] Offboard not enabled.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			sequence.clear();
			return;
		}

		if(model.sys.isStatus(Status.MSP_LANDED)) {
			control.writeLogMessage(new LogMessage("[msp] Not executed. On ground/No offboard.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
			sequence.clear();  appended.clear();
			return;
		}

		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			control.writeLogMessage(new LogMessage("[msp] Sequence already in execution.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return;
		}
		if(sequence.isEmpty()) {
			control.writeLogMessage(new LogMessage("[msp] No valid sequence.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
			return;
		}

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, true);

		future = ExecutorService.get().submit(() -> {
			int i=0;

			try { Thread.sleep(50); } catch (InterruptedException e) { }

			//		final ListIterator<SeqItem> i = sequence.listIterator();
			while(!sequence.isEmpty() && model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
				model.slam.wpcount  = ++i;
				control.writeLogMessage(new LogMessage("[msp] Step "+i+ " executed.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				SeqItem item = sequence.poll();
				if(item.hasTarget()) {
					if(item.getControlListener()!=null)
						offboard.registerExternalControlListener(item.getControlListener());
					offboard.setTarget(item.getTarget(model));
					if(!offboard.start_wait(OffboardManager.MODE_SPEED_POSITION, item.getTimeout_ms())) {
						model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
						control.writeLogMessage(new LogMessage("[msp] Sequence timeout occurred.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
						break;
					}
					if(!offboard.isEnabled()) {
						model.slam.wpcount = 0;
						sequence.clear();
						control.writeLogMessage(new LogMessage("[msp] Sequence aborted.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
						return;
					}
				}

				// Execute action
				if(!item.executeAction()) {
					control.writeLogMessage(new LogMessage("[msp] Sequence action request abort.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					break;
				}
				// Extend sequence by appended items
				if(!appended.isEmpty()) {
					control.writeLogMessage(new LogMessage("[msp] "+appended.size()+" added to sequence", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					appended.forEach((n) -> { sequence.add(n); });
					appended.clear();
				}
			}
			offboard.setTarget(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
			offboard.start(OffboardManager.MODE_LOITER);

			model.slam.wpcount = 0;
			sequence.clear();

			if(completedAction!=null && model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
				completedAction.execute();

			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
				control.writeLogMessage(new LogMessage("[msp] Sequence finished.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			else
				control.writeLogMessage(new LogMessage("[msp] Sequence aborted.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
		});
	}

	protected void start() {
		isRunning = true;
		Thread worker = new Thread(this);
		//worker.setPriority(Thread.MIN_PRIORITY);
		worker.setName("AutoPilot");
		worker.start();
	}

	protected void stop() {
		isRunning = false;
	}

	protected void publishSLAMData() {
		publishSLAMData(null);
	}

	protected void publishSLAMData(Polar3D_F32 obstacle) {

		slam.px = model.slam.px;
		slam.py = model.slam.py;
		slam.pz = model.slam.pz;
		slam.pd = model.slam.pd;
		slam.pv = model.slam.pv;
		slam.md = model.slam.di;
		slam.quality = model.slam.quality;
		slam.fps = model.slam.fps;

		if(obstacle!=null) {

			slam.ox = obstacle.getX()+model.state.l_x;
			slam.oy = obstacle.getY()+model.state.l_y;
			slam.oz = obstacle.getZ()+model.state.l_z;
			if(control.isSimulation())
				model.slam.dm = obstacle.value;
		} else {
			slam.ox = 0; slam.oy = 0; slam.oz = 0;
			if(control.isSimulation())
				model.slam.dm = Float.NaN;
		}

		slam.wpcount = model.slam.wpcount;
		slam.dm = model.slam.dm;
		slam.tms = model.slam.tms;
		control.sendMAVLinkMessage(slam);

	}


	public long getTimeSinceTakeoff() {
		if(takeoff_ms > 0)
			return System.currentTimeMillis() - takeoff_ms;
		return 0;
	}




	@Override
	public abstract void run();

	/*******************************************************************************/
	// Command dispatching


	public void setMode(boolean enable, int mode, float param) {

		switch(mode) {
		case MSP_AUTOCONTROL_MODE.ABORT:
			abort();
			break;
		case MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.INTERACTIVE, false);
			break;
		case MSP_AUTOCONTROL_MODE.INTERACTIVE:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT, false);
			break;
		case MSP_AUTOCONTROL_ACTION.RTL:
			returnToLand(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.SAVE_MAP2D:
			saveMap2D();
			break;
		case MSP_AUTOCONTROL_ACTION.LOAD_MAP2D:
			loadMap2D();
			break;
		case MSP_AUTOCONTROL_ACTION.DEBUG_MODE1:
			setXObstacleForSITL();
			break;
		case MSP_AUTOCONTROL_ACTION.DEBUG_MODE2:
			buildvirtualWall(0.6f);
			break;
		case MSP_AUTOCONTROL_ACTION.ROTATE:
			System.out.println("Turn to "+param);
			turn_to(param);
			break;
		case MSP_AUTOCONTROL_ACTION.LOCK:
			execute_lock(false);
			break;
		case MSP_AUTOCONTROL_MODE.PX4_PLANNER:
			planner.enable(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.TEST_SEQ1:
			if(enable)
				//square();
				northAndBack();
			else
				abortSequence();
			break;
		case MSP_AUTOCONTROL_ACTION.TAKEOFF:
			countDownAndTakeoff(5,enable);
			break;
		case MSP_AUTOCONTROL_ACTION.OFFBOARD_UPDATER:
			offboardPosHold(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.APPLY_MAP_FILTER:
			//	applyMapFilter();
			break;
		}

		model.sys.setAutopilotMode(mode, enable);
	}


	/*******************************************************************************/
	// Standard setpoint setting


	public void setSpeed(boolean enable, float p, float r, float h, float y) {

		if(enable) {
			model.sys.setStatus(Status.MSP_JOY_ATTACHED,true);
			body_speed.set(
					p == 0 && r == 0 ? Float.NaN : MSPMathUtils.expo(p,0.3f) * 2f,
							p == 0 && r == 0 ? Float.NaN : MSPMathUtils.expo(r,0.3f) * 2f,
									h == 0 ? Float.NaN : h,
											y == 0 ? Float.NaN : y);

			if(!offboard.isEnabled() || offboard.getMode()==OffboardManager.MODE_SPEED_POSITION) {
				//abort any sequence if sticks moved
				if(!MSP3DUtils.isNaN(body_speed)) {
					sequence.clear(); offboard.abort();
				}
				return;
			}

			// If sticks in initial position switch to LOITER mode
			// TODO: Should be done in OffboardManager as breaking should be controlled
			if(MSP3DUtils.isNaN(body_speed)) {
				offboard.enforceCurrentAsTarget();
				offboard.start(OffboardManager.MODE_LOITER);
			} else {
				MSP3DUtils.rotateXY(body_speed, ned_speed, -model.attitude.y);
				offboard.setSpeed(ned_speed);
				offboard.start(OffboardManager.MODE_LOCAL_SPEED);
			}
		} else {
			model.sys.setStatus(Status.MSP_JOY_ATTACHED,false);
			logger.writeLocalMsg("[msp] Joystick control disabled",MAV_SEVERITY.MAV_SEVERITY_INFO);
		}
	}


	public void setTarget(float x, float y, float z, float yaw) {
		Vector4D_F32 target = new Vector4D_F32(x,y,z,yaw);
		offboard.setTarget(target);
		offboard.start(OffboardManager.MODE_SPEED_POSITION);

	}

	public void setTarget(float x, float y, float z) {
		Vector4D_F32 target = new Vector4D_F32(x,y,z,Float.NaN);
		offboard.finalize();
		offboard.setTarget(target);
		offboard.start(OffboardManager.MODE_SPEED_POSITION);

	}

	public void moveto(float x, float y, float z, float yaw) {

		final Vector4D_F32 target = new Vector4D_F32(x,y,z,yaw);
		if(flowCheck && !model.sys.isSensorAvailable(Status.MSP_PIX4FLOW_AVAILABILITY)) {
			logger.writeLocalMsg("[msp] Aborting. No Flow available.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			return;
		}

		if(planner.isStarted()) {
			planner.setTarget(target);
			return;
		}

		abortSequence();

		offboard.registerActionListener( (m,d) -> {
			offboard.start(OffboardManager.MODE_LOITER);
			logger.writeLocalMsg("[msp] Target reached.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		});
		offboard.setTarget(target);
		offboard.start(OffboardManager.MODE_SPEED_POSITION);


	}

	@Override
	public boolean update(Point3D_F64 point, Point3D_F64 body) {
		return false;
	}

	/*******************************************************************************/
	// Standard actions

	public void registerMapFilter(ILocalMapFilter filter) {
		System.out.println("registering MapFilter "+filter.getClass().getSimpleName());
		this.mapFilter = filter;
	}


	/**
	 * AutopilotAction: Offboard position hold
	 * @param enable
	 */
	public void offboardPosHold(boolean enable) {
		if(enable) {
			offboard.start();
			if(!model.sys.isStatus(Status.MSP_LANDED) && !model.sys.isStatus(Status.MSP_RC_ATTACHED)) {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
					if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
						offboard.stop();
						control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+") (manual).", MAV_SEVERITY.MAV_SEVERITY_WARNING));
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
								MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
								MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
					}
				}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
			}
		} else {
			if(model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
						MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL, 0 );
			}
			offboard.stop();
		}
	}


	/**
	 * AutopilotAction: Aborts current AutoPilot sequence
	 */
	public void abort() {
		abortSequence();
		clearAutopilotActions();
		model.sys.autopilot &= 0b11000000000000000000000000000001;
		if(model.sys.isStatus(Status.MSP_RC_ATTACHED)) {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL, 0 );
			control.writeLogMessage(new LogMessage("[msp] Autopilot disabled. Return control to pilot.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		} else {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
			control.writeLogMessage(new LogMessage("[msp] Autopilot disabled. Switched to hold mode.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		}
		offboard.stop();
	}

	/**
	 * AutopilotAction: Rotates forth and back by a given angle and executes completed action
	 * @param deg
	 * @param completedAction
	 */
	public void rotate(float deg, ISeqAction completedAction) {
		clearSequence();
		float rad = MSPMathUtils.toRad(deg);
		addToSequence(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad));
		addToSequence(new SeqItem(Float.NaN,Float.NaN,Float.NaN,-2*rad));
		addToSequence(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad));
		executeSequence(completedAction);
	}

	/**
	 * AutopilotAction: Turn to a given angle
	 * @param deg
	 */
	public void turn_to(float deg) {
		clearSequence();
		float rad = MSPMathUtils.toRad(deg);
		addToSequence(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad,ISeqAction.ABS));
		executeSequence();
	}

	/**
	 * AutopilotAction: Execute lock
	 */
	public void execute_lock(boolean land) {

		if(control.isSimulation()) {
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
			model.vision.px = model.state.l_x + 0.3f;
			model.vision.py = model.state.l_y + 0.3f;
		}

		final boolean is_offboard = model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD);

		if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED) && !model.sys.isStatus(Status.MSP_LANDED)) {
			ExecutorService.get().submit(() -> {
				control.writeLogMessage(new LogMessage("[msp] Executing lock procedure.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				offboard.setTarget(model.vision.px, model.vision.py, model.target_state.l_z, model.attitude.y);
				offboard.start_wait(OffboardManager.MODE_ADJUST_XY, 10000);
				if(land) {
					control.writeLogMessage(new LogMessage("[msp] Executing lock finalized. Landing.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 2, 0.05f );
					return;
				}
				if(!is_offboard) {
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
							MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
							MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL, 0 );
					offboard.stop();
				}
				control.writeLogMessage(new LogMessage("[msp] Executing lock finalized.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			});
		} else
			control.writeLogMessage(new LogMessage("[msp] Executing lock procedure refused.", MAV_SEVERITY.MAV_SEVERITY_WARNING));

	}

	/**
	 * AutopilotAction: Return to takoff location and land vehicle with fiducial support
	 * @param enable
	 */
	public void returnToLand(boolean enable) {

		Vector4D_F32 landing_preparation = takeoff.copy();
		landing_preparation.z = -0.8f;

		// requires CMD_RC_OVERRIDE set to 0 in SITL; for real vehicle set to 1 (3?) as long as RC is used

		abortSequence();

		if(!enable) {
			logger.writeLocalMsg("[msp] Return to launch aborted.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			return;
		}

		if((takeoff.x == 0 && takeoff.y == 0) || takeoff.isNaN()) {
			logger.writeLocalMsg("[msp] No valid takeoff ccordinates. Landing.",MAV_SEVERITY.MAV_SEVERITY_INFO);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 1f, 0, 0, 0.05f );
			return;
		}

		logger.writeLocalMsg("[msp] Return to launch.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		addToSequence(new SeqItem(takeoff,ISeqAction.ABS, null,0));
		addToSequence(new SeqItem(landing_preparation,ISeqAction.ABS, null,0));
		addToSequence(new SeqItem(Float.NaN,Float.NaN, Float.NaN, 0, ISeqAction.ABS, () -> {
			if(!model.vision.isStatus(Vision.FIDUCIAL_ACTIVE)) {
				logger.writeLocalMsg("[msp] Auto-Landing refused. No fiducial.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
				clearAutopilotActions();
			} else {
				logger.writeLocalMsg("[msp] Perform precision landing.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				// Move according to precision_offset
				landing_preparation.plusIP(new Vector4D_F32(model.vision.px, model.vision.py,0,0));

				// TODO: Should not turn to target
				addToSequence(new SeqItem(landing_preparation,ISeqAction.ABS, null,0));
				addToSequence(new SeqItem(Float.NaN,Float.NaN, Float.NaN, 0, ISeqAction.ABS, () -> {
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 1f, 0, 0, 0.05f );
					return true;
				},200));
			}
			return true;
		},200));
		executeSequence();
	}

	/**
	 * AutopilotAction: Count down and takeoff
	 * @param seconds
	 * @param enable
	 */
	public void countDownAndTakeoff(int seconds, boolean enable) {
		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			logger.writeLocalMsg("[msp] CountDown not initiated. Not armed.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			return;
		}

		if(enable ) {
			logger.writeLocalMsg("[msp] CountDown initiated.",MAV_SEVERITY.MAV_SEVERITY_INFO);
			takeoff_ms = System.currentTimeMillis() + seconds*1000;
			future = ExecutorService.get().schedule(() -> {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_TAKEOFF, -1, 0, 0, Float.NaN, Float.NaN, Float.NaN,Float.NaN);
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			}, seconds,TimeUnit.SECONDS);

		} else {
			if(future!=null) future.cancel(true);
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,0 );
			logger.writeLocalMsg("[msp] CountDown aborted.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
		}
	}

	/**
	 * AutopilotAction: stops and turns to given angle
	 * @param targetAngle
	 */
	public void emergency_stop_and_turn(float targetAngle) {
		abortSequence();
		clearAutopilotActions();
		offboard.finalize();
		offboard.setTarget(model.state.l_x, model.state.l_y, model.state.l_z, targetAngle);
		offboard.start(OffboardManager.MODE_LOITER);
		logger.writeLocalMsg("[msp] Emergency breaking",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
	}

	//*******************************************************************************/
	// Safety

	/**
	 * performs autopilot safety checks:
	 * 1. Check RC channel 8 for emergency landing
	 */
	protected boolean safetyChecks() {

		// Safety: Channel 8 (Mid) triggers landing mode of PX4
		if(Math.abs(model.rc.get(RC_LAND_CHANNEL) - RC_LAND_THRESHOLD) < RC_DEADBAND && !emergencyLanding) {
			emergencyLanding = true;
			logger.writeLocalMsg("[msp] Emergency landing triggered by RC",MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
			if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED))
				execute_lock(true);
			else
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 2, 0.05f );
			return false;
		}
		return true;
	}



	protected void clearAutopilotActions() {
		model.sys.autopilot &= 0b11000000000000000111111111111111;
		offboard.removeActionListener();
		control.sendMAVLinkMessage(new msg_msp_micro_slam(2,1));
	}


	//*******************************************************************************/
	// Map management

	public void resetMap() {
		logger.writeLocalMsg("[msp] reset local map",MAV_SEVERITY.MAV_SEVERITY_NOTICE);
		map.reset();
		map.toDataModel(false);
	}

	public void saveMap2D() {
		LocaMap2DStorage store = new LocaMap2DStorage(map,model.state.g_lat, model.state.g_lon);
		store.write();
		logger.writeLocalMsg("[msp] Map for this home position stored.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
	}

	public void loadMap2D() {
		LocaMap2DStorage store = new LocaMap2DStorage(map, model.state.g_lat, model.state.g_lon);
		if(store.locateAndRead()) {
			logger.writeLocalMsg("[msp] Map for this home position loaded.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			map.setDataModel(control.getCurrentModel()); map.toDataModel(false); map.setIsLoaded(true);
		}
		else
			logger.writeLocalMsg("[msp] No Map for this home position found.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
	}

	public ILocalMap getMap2D() {
		return map;
	}


	/*******************************************************************************/
	// SITL testing


	public void square() {
		clearSequence();
		//		addToSequence(new SeqItem(Float.NaN, Float.NaN, -1.0f, Float.NaN , SeqItem.ABS));
		addToSequence(new SeqItem(0.5f     , 0.5f     , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(Float.NaN, -1f      , -1.5f, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(-1f      , Float.NaN, Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(Float.NaN, 1f       , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(1f       , Float.NaN, 1.5f, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(-0.5f    , -0.5f    , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		addToSequence(new SeqItem(Float.NaN, Float.NaN, Float.NaN,0         , ISeqAction.ABS));
		executeSequence();
	}

	public void northAndBack() {
		clearSequence();
		if(control.isSimulation()) {

			for(int i=1;i<10;i++)
				addToSequence(new SeqItem((float)(Math.random()*4-2),
						(float)(Math.random()*4-2),
						(float)(-Math.random()*0.5+0.2),
						Float.NaN, ISeqAction.REL, null,0));
			addToSequence(new SeqItem( 0.5f    ,       0.5f  , -2.0f, (float)(Math.PI), ISeqAction.ABS,null,0));
		} else {
			addToSequence(new SeqItem( 1f       , Float.NaN  , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
			addToSequence(new SeqItem(-1f       , Float.NaN  , Float.NaN, Float.NaN, ISeqAction.REL,null,0));		}
		executeSequence();
	}


	public void buildvirtualWall(float distance_m) {
		if(map==null)
			return;
		control.writeLogMessage(new LogMessage("[msp] Build virtual wall.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

		Point3D_F64   pos          = new Point3D_F64();
		Point3D_F64   wall         = new Point3D_F64();
		pos.setZ(model.state.l_z);
		pos.x = model.state.l_x + Math.cos(model.attitude.y) * distance_m;
		pos.y = model.state.l_y + Math.sin(model.attitude.y) * distance_m;


		wall.setZ(model.state.l_z);
		for(int k=-5; k<6; k++) {
			wall.x = pos.x + Math.sin(-model.attitude.y) * 0.05f * k;
			wall.y = pos.y + Math.cos(-model.attitude.y) * 0.05f * k;
			for(int i=0;i<100;i++) map.update(model.state.l_x, model.state.l_y,wall);
		}
	}


	//**********

	public void setCircleObstacleForSITL() {
		if(map==null)
			return;
		map.reset();
		Vector3D_F32   pos          = new Vector3D_F32();
		System.err.println("SITL -> set example obstacle map");
		pos.x = 0.5f + model.state.l_x;
		pos.y = 0.4f + model.state.l_y;
		pos.z = 1.0f + model.state.l_z;
		map.update(pos); map.update(pos); map.update(pos);
		map.update(pos); map.update(pos); map.update(pos);
		map.update(pos); map.update(pos); map.update(pos);
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.45f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.50f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.55f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.60f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.65f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
		pos.y = 0.70f + model.state.l_y;
		map.update(pos); map.update(pos); map.update(pos);
	}

	public void setXObstacleForSITL() {
		if(map==null)
			return;
		map.reset();

		Point3D_F64   pos          = new Point3D_F64();
		System.err.println("SITL -> set X example obstacle map");
		this.mapForget = false;

		pos.y = 2.25f;
		pos.z =  model.state.l_z;
		for(int i = 0; i < 40;i++) {
			pos.x = -1.25f + i *0.05f;
			for(int z=0;z<100;z++)
				map.update(model.state.l_x, model.state.l_y,pos);
		}

		pos.y = 3.75f ;
		pos.z =  model.state.l_z;
		for(int i = 0; i < 30;i++) {
			pos.x = -1.25f + i *0.05f ;
			for(int z=0;z<100;z++)
				map.update(model.state.l_x, model.state.l_y,pos);
		}

		for(int i = 0; i < 30;i++) {
			pos.x = 1.25f + i *0.05f ;
			for(int z=0;z<100;z++)
				map.update(model.state.l_x, model.state.l_y,pos);
		}

		pos.x = 2.0f ;
		for(int i = 0; i < 25;i++) {
			pos.y = -1 + i *0.05f;
			for(int z=0;z<100;z++)
				map.update(model.state.l_x, model.state.l_y,pos);
		}



	}

	public void setYObstacleForSITL() {
		float x,y;
		if(map==null)
			return;
		map.reset();
		Vector3D_F32   pos          = new Vector3D_F32();
		System.err.println("SITL -> set example obstacle map");
		pos.z = 1.0f + model.state.l_z;

		pos.y = 4f + model.state.l_y;
		pos.x = -0.15f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x = -0.10f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x = -0.05f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x =  0.00f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x = 0.05f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x = 0.10f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);
		pos.x = 0.15f + model.state.l_x;
		map.update(pos); map.update(pos); map.update(pos);

		float dotx, doty ; float[] r = new float[2];

		for(int j=0; j< 30; j++) {

			dotx = (float)((Math.random()*15-5f));
			doty = (float)((Math.random()*15-5f));

			MSPMathUtils.rotateRad(r, dotx, doty, (float)Math.random() * 6.28f);

			for(int i=0; i< 40; i++) {

				x =  (float)Math.random()*.8f - 0.4f + r[0];
				y =  (float)Math.random()*.8f - 0.4f + r[1];

				pos.x = x;
				pos.y = y ;
				map.update(pos); map.update(pos); map.update(pos);

			}
		}

	}


}
