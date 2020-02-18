package com.comino.mavcontrol.autopilot;

import org.mavlink.messages.lquac.msg_msp_micro_slam;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;


public class NoPilot extends AutoPilotBase {



	protected NoPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		start();
	}

	public void run() {

		while(isRunning) {

			try { Thread.sleep(50); } catch(Exception s) { }
			publishSLAMData();
		}

	}

	protected void takeoffCompleted() {

	}




	@Override
	public void moveto(float x, float y, float z, float yaw) {

	}

}
