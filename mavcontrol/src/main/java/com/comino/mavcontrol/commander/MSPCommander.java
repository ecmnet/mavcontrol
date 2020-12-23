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
import java.util.concurrent.TimeUnit;

import org.mavlink.messages.MAV_BATTERY_CHARGE_STATE;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_gps_global_origin;
import org.mavlink.messages.lquac.msg_hil_gps;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_set_gps_global_origin;
import org.mavlink.messages.lquac.msg_set_home_position;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavmap.map.map2D.ILocalMap;
import com.comino.mavutils.legacy.ExecutorService;

@SuppressWarnings("unused")
public class MSPCommander  {

	private IMAVMSPController        control 	= null;
	private AutoPilotBase           autopilot 	= null;
	private DataModel                  model 	= null;
	private ILocalMap                	map  	= null;
	private MSPLogger                  logger   = null;

	public MSPCommander(IMAVMSPController control, MSPConfig config) {


		this.control = control;
		this.model   = control.getCurrentModel();
		this.logger  = MSPLogger.getInstance();

		registerCommands();
		registerLowBattery();

		System.out.println("Commander initialized");

		//		String autopilot_class = config.getProperty("autopilot_class", "com.comino.mavcontrol.autopilot.TrajectoryPilot");
		String autopilot_class = config.getProperty("autopilot_class", "com.comino.mavcontrol.autopilot.BreakingPilot");

		this.autopilot =  AutoPilotBase.getInstance(autopilot_class,control,config);
		this.map = autopilot.getMap2D();
	}

	public AutoPilotBase getAutopilot() {
		return autopilot;
	}

	public void setGlobalOrigin(double lat, double lon, double altitude) {


//		if(model.sys.isStatus(Status.MSP_GPOS_VALID) || lat == 0.0 || lon == 0.0 ||
//				model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY)) 
//			return;

//		msg_set_gps_global_origin gor = new msg_set_gps_global_origin(1,2);
//		gor.latitude = (long)(lat * 1e7);
//		gor.longitude = (long)(lon * 1e7);
//		gor.altitude = (int)(altitude * 1000);
//		gor.time_usec = model.sys.getSynchronizedPX4Time_us();
//		
//		control.sendMAVLinkMessage(gor);
		
		msg_gps_global_origin gor = new msg_gps_global_origin(1,2);
		gor.latitude = (long)(lat * 1e7);
		gor.longitude = (long)(lon * 1e7);
		gor.altitude = (int)(altitude * 1000);
		gor.time_usec = DataModel.getSynchronizedPX4Time_us();
		
		control.sendMAVLinkMessage(gor);

		MSPLogger.getInstance().writeLocalMsg("[msp] Try to set reference position",
				MAV_SEVERITY.MAV_SEVERITY_INFO);

		// HIL does not work as this requires GPS active in EKF2. But if GPS is active in EKF2 Vision position
		// is not considered anymore.

		//		ExecutorService.submit(() -> {
		//
		//			long tms = System.currentTimeMillis();
		//
		////			msg_gps_global_origin gor = new msg_gps_global_origin(1,2);
		////			gor.latitude = (long)(lat * 1e7);
		////			gor.longitude = (long)(lon * 1e7);
		////			gor.altitude = (int)(altitude * 1000);
		////			gor.time_usec = tms*1000;
		//
		//
		//			msg_hil_gps gps = new msg_hil_gps(1,1);
		//			gps.lat = (long)(lat * 1e7);
		//			gps.lon = (long)(lon * 1e7);
		//			gps.alt = (int)(altitude * 1000);
		//			gps.satellites_visible = 10;
		//			gps.eph = 30;
		//			gps.epv = 30;
		//			gps.fix_type = 4;
		//			gps.cog = 0;
		//			gps.time_usec = model.sys.getSynchronizedPX4Time_us();
		//			
		//			msg_set_home_position pos = new msg_set_home_position(1,2);
		//			pos.x = 0;
		//			pos.y = 0;
		//			pos.z = 0;
		//			pos.altitude = (int)(altitude * 1000);
		//			pos.time_usec = model.sys.getSynchronizedPX4Time_us();
		//
		//			while(!model.sys.isStatus(Status.MSP_GPOS_VALID)
		//					&& (System.currentTimeMillis() - tms) < MAX_GPOS_SET_MS) {
		//				
		//				if(!control.isConnected())
		//					return;
		//				
		//				control.sendMAVLinkMessage(gps);
		//				control.sendMAVLinkMessage(pos);
		//				
		//				try {
		//					Thread.sleep(200);
		//				} catch (InterruptedException e) { }
		//			}
		//			
		//			if(model.sys.isStatus(Status.MSP_GPOS_VALID)) {
		//				control.sendMAVLinkMessage(pos);
		//				MSPLogger.getInstance().writeLocalMsg("[msp] Reference position set from MAVGCL",
		//						MAV_SEVERITY.MAV_SEVERITY_INFO);
		//			}
		//			else
		//				MSPLogger.getInstance().writeLocalMsg("[msp] Setting reference position failed",
		//						MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		//
		//		}, ExecutorService.LOW );


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

			if(model.sys.isStatus(Status.MSP_RC_ATTACHED) && model.sys.isStatus(Status.MSP_ARMED)) {
				logger.writeLocalMsg("[msp] Switching to ... (Test). Ap Status = "+autopilot.getAutopilotStatus(),MAV_SEVERITY.MAV_SEVERITY_INFO);
			}



			// Different actions depending on the current mode, e.g.
			// Shutdown MSP, Switch off SLAM, RTL, Landing, etc

		});
	}

	private void registerCommands() {

		// register MSP commands here

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				ExecutorService.get().submit(() -> {
					msg_msp_command cmd = (msg_msp_command)o;
					switch(cmd.command) {
					case MSP_CMD.MSP_CMD_RESTART:
						restartCompanion();
						break;
					case MSP_CMD.MSP_CMD_OFFBOARD_SETLOCALPOS:
						setOffboardPosition(cmd);
						break;
					case MSP_CMD.MSP_CMD_SET_HOMEPOS:
						setGlobalOrigin(cmd.param1 / 1e7f, cmd.param2 / 1e7f, cmd.param3 / 1e3f );
						break;
					case MSP_CMD.MSP_CMD_AUTOMODE:
						autopilot.setMode((int)(cmd.param1)==MSP_COMPONENT_CTRL.ENABLE,(int)(cmd.param2),cmd.param3);
						break;
					case MSP_CMD.MSP_CMD_OFFBOARD_SETLOCALVEL:
						autopilot.setSpeed((int)(cmd.param1)==MSP_COMPONENT_CTRL.ENABLE,cmd.param2, cmd.param3, cmd.param4, cmd.param5);
						break;
					case MSP_CMD.MSP_CMD_MICROSLAM:
						switch((int)cmd.param1) {
						case MSP_COMPONENT_CTRL.RESET:
							autopilot.resetMap(); break;
						}
						break;
					}
				});
			}
		});
	}

	public ILocalMap getMap() {
		return map;
	}

	public long getTimeSinceTakeoff() {
		return autopilot.getTimeSinceTakeoff();
	}


	private void restartCompanion() {
		if(model.sys.isStatus(Status.MSP_LANDED) && !model.sys.isStatus(Status.MSP_ARMED)) {
			logger.writeLocalMsg("[msp] Flight control restarted", MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
			control.sendShellCommand("reboot");
			executeConsoleCommand("service flightcontrol restart");
		}
		else
			logger.writeLocalMsg("[msp] Restart command rejected.",
					MAV_SEVERITY.MAV_SEVERITY_WARNING);
	}

	private void shutdownCompanion() {

		logger.writeLocalMsg("[msp] Shutdown of MSP companion in 10 seconds.",MAV_SEVERITY.MAV_SEVERITY_INFO);	
		ExecutorService.get().schedule(() -> {
			logger.writeLocalMsg("[msp] Shutdown of MSP companion now!",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);	
			System.out.println("Companion shutdown now!");
			if(!control.isSimulation() && !model.sys.isStatus(Status.MSP_ARMED)) {
				executeConsoleCommand("shutdown -h now");
			} else {
				System.exit(0);
			}
		}, 10, TimeUnit.SECONDS);
	}


	private void setOffboardPosition(msg_msp_command cmd) {
		if(cmd.param3 == 0 || cmd.param3 == Float.NaN)
			autopilot.moveto(cmd.param1, cmd.param2, Float.NaN, cmd.param4);
		else
			autopilot.moveto(cmd.param1, cmd.param2, cmd.param3, cmd.param4);
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
