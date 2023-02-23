/****************************************************************************
 *
 *   Copyright (c) 2017,2023 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_micro_grid;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcontrol.autopilot.actions.OffboardActionFactory;
import com.comino.mavcontrol.autopilot.actions.TakeOffHandler;
import com.comino.mavcontrol.autopilot.actions.TestActionFactory;
import com.comino.mavcontrol.autopilot.safety.SafetyCheckHandler;
import com.comino.mavcontrol.ekf2utils.EKF2ResetCheck;
import com.comino.mavcontrol.offboard3.Offboard3Manager;
import com.comino.mavcontrol.scenario.ScenarioManager;
import com.comino.mavmap.map.map3D.Map3DSpacialInfo;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavmap.map.map3D.store.LocaMap3DStorage;
import com.comino.mavmap.test.MapTestFactory;
import com.comino.mavodometry.estimators.ITargetListener;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;


public abstract class AutoPilotBase implements Runnable, ITargetListener {


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
	protected PX4Parameters                 params   = null;
	protected ScenarioManager     scenario_manager   = null;
	protected Offboard3Manager    offboard_manager   = null;


	protected TakeOffHandler         takeoff_handler = null;
	protected SafetyCheckHandler safetycheck_handler = null;
	protected EKF2ResetCheck        ekf2_reset_check = null;

	protected boolean			           mapForget = false;
	protected boolean                      flowCheck = false;
	protected boolean              publish_microgrid = true;

	protected boolean                      isRunning = false;



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

		this.offboard_manager = Offboard3Manager.getInstance(control);
		this.scenario_manager = ScenarioManager.getInstance(control);
		this.control          = control;
		this.model            = control.getCurrentModel();
		this.logger           = MSPLogger.getInstance();
		this.params           = PX4Parameters.getInstance();

		this.ekf2_reset_check = new EKF2ResetCheck(control);

		this.publish_microgrid = config.getBoolProperty(MSPParams.PUBLISH_MICROGRID, "true");
		System.out.println("[vis] Publishing microGrid enabled: "+publish_microgrid);

		this.mapForget = config.getBoolProperty(MSPParams.AUTOPILOT_FORGET_MAP, "true");
		System.out.println(instanceName+": Map forget enabled: "+mapForget);

		Map3DSpacialInfo map_info = new Map3DSpacialInfo(0.10f,20.0f,20.0f,5.0f);
		this.map      = new LocalMap3D(map_info,mapForget,publish_microgrid);
		this.model.grid.setResolution(map_info.getCellSize());


		this.flowCheck = config.getBoolProperty(MSPParams.AUTOPILOT_FLOW_CHECK, "true") & !control.isSimulation();
		System.out.println(instanceName+": FlowCheck enabled: "+flowCheck);

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE,
				config.getBoolProperty(MSPParams.AUTOPILOT_TAKEOFF_PROC, "false"));

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,
				config.getBoolProperty(MSPParams.AUTOPILOT_PRECISION_LOCK, "false"));


		if(config.getBoolProperty(MSPParams.AUTOPILOT_TAKEOFF_OFFBOARD, "false"))
			this.takeoff_handler = new TakeOffHandler(control, () -> takeoffCompletedAction(),null);
		else
			this.takeoff_handler = new TakeOffHandler(control, null,null);

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

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_STATUS, Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
			ekf2_reset_check.reset(true);
			resetMap(); 
			map.setOrigin(model.state.l_x, model.state.l_y, 0);

		});

	}



	protected void registerDisarm() {

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_STATUS, Status.MSP_ARMED, StatusManager.EDGE_FALLING, (n) -> {

			takeoff_handler.abort("Disarmed");
			wq.removeTask("LP",future);
		

//
//			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
//					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
//					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL, 0 );

		});

	}

	protected void registerLanding() {

		// Abort any sequence if PX4 landing is triggered
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_LAND, StatusManager.EDGE_RISING, (n) -> {
			if(!model.sys.isStatus(Status.MSP_LANDED)) { // Workaround; Landing mode triggered on ground sometimes
				//sequencer.abort();
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
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP, true);

	}

	protected void start(int cycle_ms) {

		wq.addCyclicTask("NP", cycle_ms, this);

		safetycheck_handler.start();

		if(publish_microgrid)
			wq.addCyclicTask("NP",20, new MapToModelTransfer());

	}


	protected void publishSLAMData() {
		transferObstacleToModel(null);
	}

	protected void addEKF2ResetListener(Runnable r) {
		ekf2_reset_check.addListener(r);
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

	public EKF2ResetCheck getEKF2ResetCheck() {
		return this.ekf2_reset_check;
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

	public IMAVController getControl() {
		return control;
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
//		case MSP_AUTOCONTROL_ACTION.RTL:
//			OffboardActionFactory.precision_landing_rotate();
//			break;
		case MSP_AUTOCONTROL_ACTION.SAVE_MAP2D:
			saveMap2D();
			break;
		case MSP_AUTOCONTROL_ACTION.LOAD_MAP2D:
			loadMap2D();
			break;
		case MSP_AUTOCONTROL_ACTION.DEBUG_MODE1:
			if(control.isSimulation())
				TestActionFactory.accTest();
			break;
		case MSP_AUTOCONTROL_ACTION.DEBUG_MODE2:
			control.writeLogMessage(new LogMessage("[msp] Build virtual wall.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			MapTestFactory.buildWall(map, model, 1.5f, 0.5f);
			break;
		case MSP_AUTOCONTROL_ACTION.ROTATE:
			OffboardActionFactory.turn_to(MSPMathUtils.toRad(param));
			break;
//		case MSP_AUTOCONTROL_ACTION.LAND:
//			if(control.isSimulation())
//				TestActionFactory.continuous_planning(control.getCurrentModel(),false);
//			precisionLand(enable);
//			break;
		case MSP_AUTOCONTROL_MODE.OBSTACLE_STOP:
			if(enable)
				control.writeLogMessage(new LogMessage("[msp] Obstacle stop enabled.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			break;
		case MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT:
			if(enable)
				control.writeLogMessage(new LogMessage("[msp] Turn to person enabled.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

			if(control.isSimulation())
				TestActionFactory.test_simulate_person(enable);
			break;
		case MSP_AUTOCONTROL_ACTION.TAKEOFF:
			if(enable)
				takeoff_handler.initiateTakeoff(control.isSimulation() ? 1 : 5);
			else {
				takeoff_handler.abort("Abort");
			}
			//	countDownAndTakeoff(5,enable);
			break;



		case MSP_AUTOCONTROL_MODE.SITL_MODE1:
			if(control.isSimulation())
				TestActionFactory.continuous_planning(control.getCurrentModel(),enable);
			break;

		case MSP_AUTOCONTROL_ACTION.SITL_ACTION1:
			if(control.isSimulation())
				TestActionFactory.setRandomObstacle();
			break;


		case MSP_AUTOCONTROL_ACTION.SITL_ACTION2:
			if(control.isSimulation())
				TestActionFactory.test_circle(control,true);
			break;
		}

		model.sys.setAutopilotMode(mode, enable);
	}


	/*******************************************************************************/
	// Standard setpoint setting


	public LocalMap3D getMap() {
		return map;
	}


	@Override
	public boolean update(Point3D_F64 point, Point3D_F64 body) {
		return false;
	}

	/*******************************************************************************/
	// Standard actions



	/**
	 * AutopilotAction: Aborts current AutoPilot sequence
	 */
	public void abort() {

		offboard_manager.abort();
		scenario_manager.abort();
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
