/****************************************************************************
 *
 *   Copyright (c) 2017-2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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


package com.comino.mavcontrol.commander;

import java.io.IOException;
import java.util.LinkedList;

import org.mavlink.messages.IMAVLinkMessageID;
import org.mavlink.messages.MAV_BATTERY_CHARGE_STATE;
import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_set_home_position;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.autopilot.actions.OffboardActionFactory;
import com.comino.mavcontrol.scenario.ScenarioFactory;
import com.comino.mavcontrol.scenario.ScenarioManager;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;
import com.comino.mavcontrol.scenario.parser.Scenario;
import com.comino.mavcontrol.scenario.parser.ScenarioReader;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavutils.workqueue.WorkQueue;

@SuppressWarnings("unused")
public class MSPCommander  {

	private final IMAVMSPController	control ;
	private final AutoPilotBase     autopilot ;
	private final DataModel         model ;
	private final StatusCheck       status_check;
	private final ScenarioManager 	scenarioManager;
	private final MSPLogger         logger;



	private final WorkQueue wq = WorkQueue.getInstance();
	private final PX4Parameters params;

	public MSPCommander(IMAVMSPController control, MSPConfig config) {


		this.scenarioManager = ScenarioManager.getInstance(control);
		this.control = control;
		this.model   = control.getCurrentModel();
		this.logger  = MSPLogger.getInstance();
		this.params  = PX4Parameters.getInstance();

		this.status_check   = new StatusCheck(control);
		this.status_check.start();

		registerActions();
		registerCommands();
		registerLowBattery();

		System.out.println("Commander initialized");

		//String autopilot_class = config.getProperty(MSPParams.AUTOPILOT_CLASS, "com.comino.mavcontrol.autopilot.BreakingPilot");
		String autopilot_class = config.getProperty(MSPParams.AUTOPILOT_CLASS, "com.comino.mavcontrol.autopilot.SimplePlannerPilot");

		this.autopilot =  AutoPilotBase.getInstance(autopilot_class,control,config);

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.COLLISION_PREVENTION, true);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP,true); 

	}

	public AutoPilotBase getAutopilot() {
		return autopilot;
	}

	private void registerActions() {

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_STATUS, Status.MSP_CONNECTED, StatusManager.EDGE_RISING, (a) -> {
			if(!model.sys.isStatus(Status.MSP_ARMED)) {
				System.out.println("Setting up MAVLINK streams and refresh parameters...");
				
				// Streams removed
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET,-1);	
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_UTM_GLOBAL_POSITION,-1);	
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ESC_STATUS,-1);	
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ESC_INFO,-1);	
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_SCALED_PRESSURE,-1);	
				
				// rate adjusted Note: Interval is in us
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_ESTIMATOR_STATUS,50000);
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_LOCAL_POSITION_NED,16666);
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,16666);
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,50000);
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL,IMAVLinkMessageID.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,50000);

				// Load parameters if required
				if(!params.isLoaded())
					params.requestRefresh(true);
			}
		});
		
		control.getStatusManager().addListener(StatusManager.TYPE_MSP_STATUS, Status.MSP_ARMED, StatusManager.EDGE_FALLING, (n) -> {
			
			model.obs.clear();
			
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, false);

		});

		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED) && !model.sys.isStatus(Status.MSP_ARMED)) {

				params.sendParameter("RTL_DESCEND_ALT", 1.0f);
				params.sendParameter("RTL_RETURN_ALT", 1.0f);
				params.sendParameter("NAV_MC_ALT_RAD", 0.05f);

				if(control.isSimulation()) {
					params.sendParameter("COM_RC_OVERRIDE", 0);
					params.sendParameter("COM_RCL_EXCEPT", 7);
					params.sendParameter("MPC_XY_VEL_P_ACC", 4.5f);
					params.sendParameter("MIS_TAKEOFF_ALT", 1.5f);
					//	}

					// Autotune params
					params.sendParameter("MC_ROLL_P", 5.92f);
					params.sendParameter("MC_ROLLRATE_P", 0.170f);
					params.sendParameter("MC_ROLLRATE_I", 0.217f);
					params.sendParameter("MC_ROLLRATE_D", 0.0036f);

					params.sendParameter("MC_PITCH_P", 5.72f);
					params.sendParameter("MC_PITCHRATE_P", 0.162f);
					params.sendParameter("MC_PITCHRATE_I", 0.228f);
					params.sendParameter("MC_PITCHRATE_D", 0.0037f);

					params.sendParameter("MC_YAW_P", 5.0f);
					params.sendParameter("MC_YAWRATE_P", 0.17f);
					params.sendParameter("MC_YAWRATE_I", 0.17f);

				}

				// Simple check for tethered mode; needs to be better
				if(model.battery.b0 > 14.1 && model.battery.b0  < 14.4) {
					model.sys.bat_type = Status.MSP_BAT_TYPE_TETHERED;
				} else {
					model.sys.bat_type = Status.MSP_BAT_TYPE_BAT;
				}

			}

		});

	}


	private void registerLowBattery() {

		control.getStatusManager().addListener(StatusManager.TYPE_BATTERY, MAV_BATTERY_CHARGE_STATE.MAV_BATTERY_CHARGE_STATE_LOW, (n) -> {
			logger.writeLocalMsg("[msp] Battery low procedure triggered.",MAV_SEVERITY.MAV_SEVERITY_WARNING);

			// Different actions depending on the current mode, e.g.
			// Shutdown MSP, Switch off SLAM, RTL, Landing, etc

		});

		control.getStatusManager().addListener(StatusManager.TYPE_BATTERY, MAV_BATTERY_CHARGE_STATE.MAV_BATTERY_CHARGE_STATE_CRITICAL, (n) -> {
			logger.writeLocalMsg("[msp] Battery critical procedure triggered.",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);

			// Shutdown MSP if vehicle is not armed
			if(!model.sys.isStatus(Status.MSP_ARMED) && model.sys.isStatus(Status.MSP_LANDED))
				logger.writeLocalMsg("[msp] Shutdown simulated",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
			//shutdownCompanion();	

			// Different actions depending on the current mode, e.g.
			// Shutdown MSP, Switch off SLAM, RTL, Landing, etc



		});
	}

	private void registerCommands() {

		// register MSP commands here

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				wq.addSingleTask("LP",() -> {
					msg_msp_command cmd = (msg_msp_command)o;
					switch(cmd.command) {
					case MSP_CMD.MSP_CMD_RESTART:
						restartCompanion();
						break;
					case MSP_CMD.MSP_CMD_OFFBOARD_SETLOCALPOS:
						setOffboardPosition(cmd);
						break;
					case MSP_CMD.MSP_CMD_AUTOMODE:
						setMode((int)(cmd.param1)==MSP_COMPONENT_CTRL.ENABLE,(int)(cmd.param2),cmd.param3);
						break;
					case MSP_CMD.MSP_CMD_EXECUTE_SCENARIO:
						executeScenario("scenario.xml");
						break;
					case MSP_CMD.MSP_CMD_OFFBOARD_SETLOCALVEL:

						break;
					case MSP_CMD.MSP_CMD_MICROSLAM:
						switch((int)cmd.param1) {
						case MSP_COMPONENT_CTRL.RESET:
							autopilot.resetMap(); break;
						}
						break;
					case MSP_CMD.MSP_CMD_CHECK_READY:
						if(status_check.checkFlightReadiness(true))
							control.writeLogMessage(new LogMessage("[msp] Status checks successful.",MAV_SEVERITY.MAV_SEVERITY_NOTICE));
						break;
					case MSP_CMD.MSP_CMD_SET_HOMEPOS:
						setGlobalOrigin(cmd.param1 / 1e7f, cmd.param2 / 1e7f, cmd.param3 / 1e3f );
						break;
					}
				});
			}
		});
	}

	private void setMode(boolean enable, int mode, float param) {

		switch(mode) {
		case MSP_AUTOCONTROL_ACTION.RTL:
			scenarioManager.setMaxVelocity(1.5f);
			scenarioManager.addItems(ScenarioFactory.createRTLScenario(control).getList());
			scenarioManager.start();
			break;
		case MSP_AUTOCONTROL_ACTION.LAND:
			if(model.sys.isStatus(Status.MSP_LANDED) || !enable) 
				return;
			scenarioManager.abort();
			OffboardActionFactory.precision_landing_rotate();
			break;
		default:
			autopilot.setMode(enable,mode,param);
			break;
		}
	}


	private void setGlobalOrigin(double lat, double lon, double altitude) {

		if((params.getParam("SYS_HAS_GPS")!=null && params.getParam("SYS_HAS_GPS").value == 1) || model.sys.isStatus(Status.MSP_GPOS_VALID))
			return;

		// Note: In SITL Set global origin causes BARO failure 
		// TODO: To be investigated in PX4
		if(control.isSimulation()) {
			logger.writeLocalMsg("[msp] Global origin not set (SITL)",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			return;
		}

		final msg_set_home_position  home = new msg_set_home_position(1,1);
		home.target_system = 1;
		home.latitude = (long)(lat * 1e7);
		home.longitude = (long)(lon * 1e7);
		if(altitude < 0)
			home.altitude = (int)(model.hud.ap * 1000f);
		else
			home.altitude = (int)(altitude * 1000);
		home.time_usec = DataModel.getSynchronizedPX4Time_us();
		control.sendMAVLinkMessage(home);
		//		
		//		final msg_set_gps_global_origin gor = new msg_set_gps_global_origin(1,1);
		//		gor.target_system = 1;
		//		gor.latitude = (long)(lat * 1e7);
		//		gor.longitude = (long)(lon * 1e7);
		//		if(altitude < 0)
		//			gor.altitude = (int)(model.hud.ap * 1000f);
		//		else
		//			gor.altitude = (int)(altitude * 1000);
		//		gor.time_usec = DataModel.getSynchronizedPX4Time_us();
		//
		//		control.sendMAVLinkMessage(gor);
		logger.writeLocalMsg("[msp] Setting reference position",MAV_SEVERITY.MAV_SEVERITY_INFO);

	}

	public LocalMap3D getMap() {
		return autopilot.getMap();
	}

	public long getTimeSinceTakeoff() {
		return autopilot.getTimeSinceTakeoff();
	}


	private void restartCompanion() {
		if(model.sys.isStatus(Status.MSP_LANDED) && !model.sys.isStatus(Status.MSP_ARMED)) {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1);
			if(!control.isSimulation()) { 
				wq.addSingleTask("LP",50,() -> {
					//	control.sendShellCommand("reboot");
					executeConsoleCommand("service flightcontrol restart");
				});
			}
		}
		else
			logger.writeLocalMsg("[msp] Restart command rejected.",
					MAV_SEVERITY.MAV_SEVERITY_WARNING);
	}

	private void shutdownCompanion() {

		logger.writeLocalMsg("[msp] Shutdown of MSP companion in 10 seconds.",MAV_SEVERITY.MAV_SEVERITY_INFO);	
		wq.addSingleTask("LP",10000,() -> {
			logger.writeLocalMsg("[msp] Shutdown of MSP companion now!",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);	
			System.out.println("Companion shutdown now!");
			if(!control.isSimulation()) {
				if(!model.sys.isStatus(Status.MSP_ARMED))
					executeConsoleCommand("shutdown -h now");
			} else {
				System.exit(0);
			}
		});
	}


	private void setOffboardPosition(msg_msp_command cmd) {
		//		if(cmd.param3 == 0 || cmd.param3 == Float.NaN)
		//			autopilot.moveto(cmd.param1, cmd.param2, Float.NaN, cmd.param4);
		//		else
		//			autopilot.moveto(cmd.param1, cmd.param2, cmd.param3, cmd.param4);

		OffboardActionFactory.move_to(cmd.param1, cmd.param2, Float.NaN);

	}

	private void executeScenario(String filename) {
		DataModel m = control.getCurrentModel();

		if(m.sys.isStatus(Status.MSP_ARMED) &&
				(m.sys.isNavState(Status.NAVIGATION_STATE_AUTO_TAKEOFF) ||
						m.sys.isNavState(Status.NAVIGATION_STATE_AUTO_PRECLAND) ||
						m.sys.isNavState(Status.NAVIGATION_STATE_AUTO_RTL) ||
						m.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LAND) ||
						m.sys.isNavState(Status.NAVIGATION_STATE_MANUAL))) {
			control.writeLogMessage(new LogMessage("Mode does not allow scenario execution", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return;
		}

		ScenarioReader reader = new ScenarioReader(control);
		Scenario scenario = reader.readScenario(filename);

		LinkedList<AbstractScenarioItem> list = scenario.getList();

		if(scenario.isSITL() && !control.isSimulation()) {
			control.writeLogMessage(new LogMessage("Scenario is SITL only. Not executed.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
			return;
		}

		if(!scenario.hasItems()) {
			control.writeLogMessage(new LogMessage("Scenario has not items. Not executed.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return;
		}

		scenarioManager.setMaxVelocity(scenario.getMaxSpeed());
		scenarioManager.addItems(list);
		scenarioManager.start();

	}




	// -----------------------------------------------------------------------------------------------helper

	private void executeConsoleCommand(String command) {
		try {
			Runtime.getRuntime().exec(command);
		} catch (IOException e) {
			MSPLogger.getInstance().writeLocalMsg("LINUX command '"+command+"' failed: "+e.getMessage(),
					MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
		}
	}
}
