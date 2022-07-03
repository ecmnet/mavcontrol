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

package com.comino.mavcontrol.autopilot;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_micro_grid;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.autopilot.actions.SafetyCheckHandler;
import com.comino.mavcontrol.autopilot.actions.StandardActionFactory;
import com.comino.mavcontrol.autopilot.actions.TakeOffHandler;
import com.comino.mavcontrol.autopilot.tests.PlannerTest;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavcontrol.sequencer.ISeqAction;
import com.comino.mavcontrol.sequencer.Sequencer;
import com.comino.mavcontrol.struct.SeqItem;
import com.comino.mavmap.map.map3D.Map3DSpacialInfo;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavmap.map.map3D.store.LocaMap3DStorage;
import com.comino.mavmap.test.MapTestFactory;
import com.comino.mavodometry.estimators.ITargetListener;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;


public abstract class AutoPilotBase implements Runnable, ITargetListener {

	/* TEST ONLY */
	private PlannerTest planner = null;

	protected static final int   CERTAINITY_THRESHOLD          = 100;
	protected static final float WINDOWSIZE       	           = 3.0f;

	protected static final float MAX_REL_DELTA_HEIGHT          = 0.10f;
	protected static final long  MAP_RETENTION_TIME_MS         = 3000;
	protected static final long  MAP_RETENTION_TIME_NO_SLAM_MS = 30000;

	private static AutoPilotBase  autopilot    = null;

	protected final WorkQueue wq;

	protected DataModel                     model    = null;
	protected MSPLogger                     logger   = null;
	protected IMAVController                control  = null;
	protected LocalMap3D				    map      = null;
	protected OffboardManager               offboard = null;
	protected PX4Parameters                 params   = null;


	protected TakeOffHandler         takeoff_handler = null;
	protected SafetyCheckHandler safetycheck_handler = null;

	protected boolean			           mapForget = false;
	protected boolean                      flowCheck = false;
	protected boolean              publish_microgrid = true;

	protected boolean                      isRunning = false;

	protected Sequencer                    sequencer = null;

	private final Vector4D_F32            body_speed = new Vector4D_F32();
	private final Vector4D_F32            ned_speed  = new Vector4D_F32();


	//	private Future<?> future;
	private int future;


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

		wq = WorkQueue.getInstance();

		String instanceName = this.getClass().getSimpleName();

		System.out.println(instanceName+" instantiated");

		this.control   = control;
		this.model     = control.getCurrentModel();
		this.logger    = MSPLogger.getInstance();
		this.params    = PX4Parameters.getInstance();
		
		this.publish_microgrid = config.getBoolProperty(MSPParams.PUBLISH_MICROGRID, "true");
		System.out.println("[vis] Publishing microGrid enabled: "+publish_microgrid);

		this.mapForget = config.getBoolProperty(MSPParams.AUTOPILOT_FORGET_MAP, "true");
		System.out.println(instanceName+": Map forget enabled: "+mapForget);

		Map3DSpacialInfo map_info = new Map3DSpacialInfo(0.10f,20.0f,20.0f,5.0f);
		this.map      = new LocalMap3D(map_info,mapForget,publish_microgrid);
		this.model.grid.setResolution(map_info.getCellSize());

		this.offboard  = new OffboardManager(control, map, params);
		this.sequencer = new Sequencer(offboard,logger,model,control);


		this.flowCheck = config.getBoolProperty(MSPParams.AUTOPILOT_FLOW_CHECK, "true") & !control.isSimulation();
		System.out.println(instanceName+": FlowCheck enabled: "+flowCheck);

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE,
				config.getBoolProperty(MSPParams.AUTOPILOT_TAKEOFF_PROC, "false"));

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,
				config.getBoolProperty(MSPParams.AUTOPILOT_PRECISION_LOCK, "false"));


		this.takeoff_handler = new TakeOffHandler(control, offboard,() -> takeoffCompletedAction());
		this.safetycheck_handler = new SafetyCheckHandler(control);

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_SERVICES,Status.MSP_SLAM_AVAILABILITY, (n) -> {
			if(n.isSensorAvailable(Status.MSP_SLAM_AVAILABILITY)) {	
				System.out.println("SLAM available -> reset MAP");
				resetMap();
			}
		});

		registerLanding();

		registerDisarm();

		registerArm();



	}

	protected void registerArm() {

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
           resetMap();
           map.setOrigin(model.state.l_x, model.state.l_y, 0);

		});

	}



	protected void registerDisarm() {

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_ARMED, StatusManager.EDGE_FALLING, (n) -> {

			takeoff_handler.abort("Disarmed");
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			wq.removeTask("LP",future);

			if(offboard.isEnabled()) {
				offboard.abort(); offboard.stop();
				control.writeLogMessage(new LogMessage("[msp] Disarmed. Switch to manual mode.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.RTL, false);

			}
			//			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
			//					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
			//					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL,MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

		});

	}

	public int getAutopilotStatus() {
		return offboard.getMode();
	}

	protected void registerLanding() {

		// Abort any sequence if PX4 landing is triggered
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_LAND, StatusManager.EDGE_RISING, (n) -> {
			if(!model.sys.isStatus(Status.MSP_LANDED)) { // Workaround; Landing mode triggered on ground sometimes
				sequencer.abort();
				takeoff_handler.abort("Landing");
				//			if(future!=null) future.cancel(true);
				wq.removeTask("LP",future);
			} else {
				// should not occur
				System.out.println("[DEBUG] NavState 'LandMode' activated by PX4 on ground for unknown reasons: "+System.currentTimeMillis());
			}
		});

	}


	protected void takeoffCompletedAction() {

		//		if(control.isSimulation()) {
		//			try { Thread.sleep(1000); } catch(Exception e) { }
		//			precisionLand(true);
		//			return;
		//		}

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP, true);
		control.writeLogMessage(new LogMessage("[msp] Obstacle survey executed.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
		rotate(45,() -> {
			control.writeLogMessage(new LogMessage("[msp] Takeoff procedure completed.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			return true;
		});

	}

	protected void start(int cycle_ms) {

		wq.addCyclicTask("NP", cycle_ms, this);
		
		if(publish_microgrid)
		  wq.addCyclicTask("NP",20, new MapToModelTransfer());

		//		wq.addCyclicTask("NP",100, () -> {
		//			
		//			if(mapForget && MAP_RETENTION_TIME_MS > 0)
		//				 map.forget(System.currentTimeMillis() - MAP_RETENTION_TIME_MS  );
		//			
		//			
		//		});
	}


	protected void publishSLAMData() {
		transferObstacleToModel(null);
	}

	protected void transferObstacleToModel(Polar3D_F32 obstacle) {

		if(obstacle!=null) {
			model.slam.ox = obstacle.getX()+model.state.l_x;
			model.slam.oy = obstacle.getY()+model.state.l_y;
			model.slam.oz = obstacle.getZ()+model.state.l_z;
			if(control.isSimulation())
				model.slam.dm = obstacle.value;
		} else {
			model.slam.ox = 0; model.slam.ox = 0; model.slam.ox = 0;
			if(control.isSimulation())
				model.slam.dm = Float.NaN;
		}
	}


	public long getTimeSinceTakeoff() {
		long tms = takeoff_handler.getPlannedTakeoffTime();
		if(tms > 0)
			return System.currentTimeMillis() - tms;
		return 0;
	}

	public void invalidate_map_transfer() {
		
		if(!publish_microgrid)
			return;
		
		map.getMapItems().forEachRemaining((p) -> {
			model.grid.add(map.getMapInfo().encodeMapPoint(p, p.probability));
		});
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
		case MSP_AUTOCONTROL_MODE.FCUM:
			if(enable) {
				wq.addSingleTask("LP", 500, () -> {
					control.writeLogMessage(new LogMessage("[msp] FCUM mode entered.", MAV_SEVERITY.MAV_SEVERITY_INFO));
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,1 );
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
							MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
							MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL, 0 );
				});
			} else {
				if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FCUM)) {
					wq.addSingleTask("LP", 500, () -> {
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 );
						control.writeLogMessage(new LogMessage("[msp] FCUM mode left.", MAV_SEVERITY.MAV_SEVERITY_INFO));
					});
				}
			}

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
			MapTestFactory.buildWall(map, model, 1, (float)(Math.random()*2 - 1.0));
			break;
		case MSP_AUTOCONTROL_ACTION.DEBUG_MODE2:
			control.writeLogMessage(new LogMessage("[msp] Build virtual wall.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			MapTestFactory.buildWall(map, model, 1.5f, 0.5f);
			break;
		case MSP_AUTOCONTROL_ACTION.ROTATE:
			StandardActionFactory.turn_to(sequencer, param);
			break;
		case MSP_AUTOCONTROL_ACTION.LAND:
			precisionLand(enable);
			break;
		case MSP_AUTOCONTROL_MODE.PX4_PLANNER:
			planner.enable(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.TEST_SEQ1:
			if(control.isSimulation())
				// SequenceTestFactory.randomSequence(sequencer);
				if(control.isSimulation()) {
					StandardActionFactory.square(sequencer, 2);
				}
				else
					logger.writeLocalMsg("[msp] Only available in simulation environment",MAV_SEVERITY.MAV_SEVERITY_INFO);
			break;
		case MSP_AUTOCONTROL_ACTION.TAKEOFF:
			if(enable)
				takeoff_handler.initiateTakeoff(control.isSimulation() ? 1 : 5);
			else {
				takeoff_handler.abort("Abort");
			}
			//	countDownAndTakeoff(5,enable);
			break;
		case MSP_AUTOCONTROL_ACTION.OFFBOARD_UPDATER:
			offboardPosHold(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.APPLY_MAP_FILTER:

			break;
		}

		model.sys.setAutopilotMode(mode, enable);
	}


	/*******************************************************************************/
	// Standard setpoint setting


	public void setSpeed(boolean enable, float p, float r, float h, float y) {

		if(enable) {
			model.sys.setStatus(Status.MSP_JOY_ATTACHED,true);
			body_speed.setTo(
					p == 0 && r == 0 ? Float.NaN : MSPMathUtils.expo(p,0.3f) * 2f,
							p == 0 && r == 0 ? Float.NaN : MSPMathUtils.expo(r,0.3f) * 2f,
									h == 0 ? Float.NaN : h,
											y == 0 ? Float.NaN : y);

			if(!offboard.isEnabled() || offboard.getMode()==OffboardManager.MODE_TRAJECTORY) {
				//abort any sequence if sticks moved
				if(!MSP3DUtils.isNaN(body_speed)) {
					sequencer.clear(); offboard.abort();
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

	public LocalMap3D getMap() {
		return map;
	}


	public void setTarget(float x, float y, float z, float yaw) {
		Vector4D_F32 target = new Vector4D_F32(x,y,z,yaw);
		offboard.setTarget(target);
		offboard.startTrajectory(target);

	}

	public void setTarget(float x, float y, float z) {
		Vector4D_F32 target = new Vector4D_F32(x,y,z,Float.NaN);
		offboard.finalize();
		offboard.setTarget(target);
		offboard.startTrajectory(target);

	}

	public void moveto(float x, float y, float z, float yaw) {

		final Vector4D_F32 target = new Vector4D_F32(x,y,z,yaw);
		//		if(flowCheck && !model.sys.isSensorAvailable(Status.MSP_PIX4FLOW_AVAILABILITY)) {
		//			logger.writeLocalMsg("[msp] Aborting. No Flow available.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
		//			return;
		//		}
		//
		//		if(planner.isStarted()) {
		//			planner.setTarget(target);
		//			return;
		//		}
		//
		//		sequencer.abort();
		//
		//		offboard.registerActionListener( (m,d) -> {
		//			offboard.start(OffboardManager.MODE_LOITER);
		//			logger.writeLocalMsg("[msp] Target reached.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		//		});
		//		offboard.setTarget(target);
		//		offboard.start(OffboardManager.MODE_SPEED_POSITION);

		offboard.startTrajectory(target);


	}

	@Override
	public boolean update(Point3D_F64 point, Point3D_F64 body) {
		return false;
	}

	/*******************************************************************************/
	// Standard actions


	/**
	 * AutopilotAction: Offboard position hold
	 * @param enable
	 */
	public void offboardPosHold(boolean enable) {
		if(enable) {
			offboard.start();
			if(!model.sys.isStatus(Status.MSP_LANDED) && (!model.sys.isStatus(Status.MSP_RC_ATTACHED) || model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FCUM)) ) {
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
		} else 
			abort();
	}


	/**
	 * AutopilotAction: Aborts current AutoPilot sequence
	 */
	public void abort() {

		sequencer.abort();
		clearAutopilotActions();

		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FCUM))
			model.sys.autopilot &= 0b11000000000000000000000000000001;

		if(model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
			if(model.sys.isStatus(Status.MSP_RC_ATTACHED)) {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
						MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL, 0 );
				control.writeLogMessage(new LogMessage("[msp] Autopilot disabled.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			} else {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
						MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
				control.writeLogMessage(new LogMessage("[msp] Autopilot disabled. Hold mode.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			}
		}
		offboard.stop();
	}

	/**
	 * AutopilotAction: Rotates forth and back by a given angle and executes completed action
	 * @param deg
	 * @param completedAction
	 */
	public void rotate(float deg, ISeqAction completedAction) {
		sequencer.clear();
		float rad = MSPMathUtils.toRad(deg);
		sequencer.add(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad));
		sequencer.add(new SeqItem(Float.NaN,Float.NaN,Float.NaN,-2*rad));
		sequencer.add(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad));
		sequencer.execute(completedAction);
	}


	/**
	 * AutopilotAction: Execute precision landing
	 */

	public void precisionLand(boolean enable) {

		if(model.sys.isStatus(Status.MSP_LANDED) || !enable) {
			offboard.abort();
			return;
		}

		if(control.isSimulation())
			StandardActionFactory.simulateFiducial(control,2f);

		StandardActionFactory.precisionLanding(sequencer, control);

		// TEST planned landing

		//
		//			if(!model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
		//				control.writeLogMessage(new LogMessage("[msp] Precision landing without lock.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		//				if(!offboard.start_wait(OffboardManager.MODE_LAND, 30000)) {
		//					control.writeLogMessage(new LogMessage("[msp] Precision landing procedure aborted", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		//				}
		//				//	offboardPosHold(true);
		//			} else {
		//
		//				control.writeLogMessage(new LogMessage("[msp] Precision landing triggered.", MAV_SEVERITY.MAV_SEVERITY_INFO));
		//				if(!offboard.start_wait(OffboardManager.MODE_LAND, 30000)) {
		//					control.writeLogMessage(new LogMessage("[msp] Precision landing procedure aborted", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		//				}
		//			}

		//		});

	}


	/**
	 * AutopilotAction: Return to takoff location and land vehicle with fiducial support
	 * @param enable
	 */
	public void returnToLand(boolean enable) {

		//		Vector4D_F32 takeoff = takeoff_handler.getTakeoffPosition();
		//		Vector4D_F32 landing_preparation = takeoff.copy();
		//		landing_preparation.z = -0.8f;
		//		landing_preparation.w = Float.NaN;

		if(control.isSimulation()) {
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, true);
			model.vision.px = takeoff_handler.getTakeoffPosition().x + ((float)Math.random()-0.5f)*2.2f;
			model.vision.py = takeoff_handler.getTakeoffPosition().y + ((float)Math.random()-0.5f)*2.2f;
			model.vision.pw = ((float)Math.random()-0.5f)*12f;
		}

		StandardActionFactory.returnToLand(sequencer, control, takeoff_handler.getTakeoffPosition(), enable);

		// requires CMD_RC_OVERRIDE set to 0 in SITL; for real vehicle set to 1 (3?) as long as RC is used

		//		sequencer.abort();
		//
		//		if(!enable) {
		//			logger.writeLocalMsg("[msp] Return to launch aborted.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
		//			return;
		//		}
		//
		//		if(Float.isNaN(takeoff.x) || Float.isNaN(takeoff.y) || Float.isNaN(takeoff.z)) {
		//			logger.writeLocalMsg("[msp] No valid takeoff ccordinates. Landing.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		//			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0,  Float.NaN );
		//			return;
		//		}
		//
		//		logger.writeLocalMsg("[msp] Return to launch.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		//	
		//		
		//		sequencer.add(new SeqItem(takeoff,ISeqAction.ABS, null,500));
		//		if(!model.vision.isStatus(Vision.FIDUCIAL_ENABLED))
		//			sequencer.add(new SeqItem(landing_preparation,ISeqAction.ABS, null,0));
		//		sequencer.add(new SeqItem(Float.NaN,Float.NaN, Float.NaN, 0, ISeqAction.ABS, () -> {
		//			if(!model.vision.isStatus(Vision.FIDUCIAL_ENABLED)) {
		//				logger.writeLocalMsg("[msp] No precision landing.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
		//				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0, Float.NaN );
		//				clearAutopilotActions();
		//			} else {
		//				precisionLand(true);
		//			}
		//			return true;
		//		},0));
		//		sequencer.execute();
	}

	/**
	 * AutopilotAction: stops and turns to given angle
	 * @param targetAngle
	 */
	public void emergency_stop_and_turn(float targetAngle) {
		sequencer.abort();
		clearAutopilotActions();
		offboard.finalize();
		offboard.setTarget(model.state.l_x, model.state.l_y, model.state.l_z, targetAngle,0);
		offboard.start(OffboardManager.MODE_LOITER);
		logger.writeLocalMsg("[msp] Emergency breaking",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
	}


	protected void clearAutopilotActions() {
		model.sys.autopilot &= 0b11000000000000000111111111111111;
		offboard.removeActionListener();
		model.slam.clear();
	}


	//*******************************************************************************/
	// Map management

	public void resetMap() {
		logger.writeLocalMsg("[msp] reset local map",MAV_SEVERITY.MAV_SEVERITY_NOTICE);
		//	map.clear(model.state.l_x,model.state.l_y, model.state.l_z);
		map.clear();
		msg_msp_micro_grid grid = new msg_msp_micro_grid(2,1);
		grid.count = 0;
		grid.cx = (float)map.getOrigin().x;
		grid.cy = (float)map.getOrigin().y;
		grid.cz = (float)map.getOrigin().z;
		control.sendMAVLinkMessage(grid);

	}

	public void saveMap2D() {
		LocaMap3DStorage store = new LocaMap3DStorage(map,model.state.g_lat, model.state.g_lon);
		store.write();
		logger.writeLocalMsg("[msp] Map for this home position stored.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
	}

	public void loadMap2D() {
		LocaMap3DStorage store = new LocaMap3DStorage(map, model.state.g_lat, model.state.g_lon);
		if(store.locateAndRead()) {
			logger.writeLocalMsg("[msp] Map for this home position loaded.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		}
		else
			logger.writeLocalMsg("[msp] No Map for this home position found.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
	}

	// put map transfer into the WQ
	// TODO: Move this to dispatcher

	private class MapToModelTransfer implements Runnable {

		private final msg_msp_micro_grid  grid     = new msg_msp_micro_grid(2,1);
		private long  map_tms = 0;

		@Override
		public void run() {
			
			if(!publish_microgrid || !model.sys.isStatus(Status.MSP_GCL_CONNECTED)) {
				return;
			}

			if(mapForget && MAP_RETENTION_TIME_MS > 0) {
				if(!control.getStatusManager().get().isSensorAvailable(Status.MSP_SLAM_AVAILABILITY)) {
					map.forget(System.currentTimeMillis() - MAP_RETENTION_TIME_NO_SLAM_MS  );
				}
				else
					map.forget(System.currentTimeMillis() - MAP_RETENTION_TIME_MS  );
			}
			
			model.sys.setSensor(Status.MSP_GRID_AVAILABILITY, true);


			map.getLatestMapItems(map_tms, LocalMap3D.PROBABILITY_THRESHOLD).forEachRemaining((p) -> {			
				model.grid.add(map.getMapInfo().encodeMapPoint(p, p.probability));
			});
			map_tms = System.currentTimeMillis()-10;

			model.grid.count = map.size();

			while(model.grid.hasTransfers()) {
				if(model.grid.toArray(grid.data)) {
					grid.cx  = (float)map.getOrigin().x;
					grid.cy  = (float)map.getOrigin().y;
					grid.cz  = (float)map.getOrigin().z;
					grid.tms = DataModel.getSynchronizedPX4Time_us();
					grid.count = model.grid.count;
					grid.resolution = model.grid.resolution;
					control.sendMAVLinkMessage(grid);
				}
			}


			// TODO: Transfer forget immediately

		}

	}

}
