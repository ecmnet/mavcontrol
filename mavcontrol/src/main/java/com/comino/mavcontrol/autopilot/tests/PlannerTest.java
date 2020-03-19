package com.comino.mavcontrol.autopilot.tests;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_COMPONENT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MAV_TYPE;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_heartbeat;
import org.mavlink.messages.lquac.msg_trajectory_representation_bezier;
import org.mavlink.messages.lquac.msg_trajectory_representation_waypoints;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.param.PX4Parameters;

public class PlannerTest implements Runnable {

	private static final msg_heartbeat beat = new msg_heartbeat(1,MAV_COMPONENT.MAV_COMP_ID_OBSTACLE_AVOIDANCE);

	protected DataModel               model    = null;
	protected MSPLogger               logger   = null;
	protected IMAVController          control  = null;

	private boolean is_running;

	public PlannerTest(IMAVController control, MSPConfig config)  {

		this.control  = control;
		this.model    = control.getCurrentModel();
		this.logger   = MSPLogger.getInstance();

		beat.type = MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER;

	}


	public void send() {
		if(!control.isSimulation()) {
			logger.writeLocalMsg("[msp] Not executed - no simulation.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			return;
		}

		System.out.println("Planner send bezier 2");

		msg_trajectory_representation_bezier bez = new msg_trajectory_representation_bezier(1,1);
	//	msg_trajectory_representation_waypoints bez = new msg_trajectory_representation_waypoints(1,1);

		bez.pos_x[0] = 1.0f;
		bez.pos_y[0] = 1.0f;
		bez.pos_z[0] = -1.0f;
		bez.delta[0] = 2;

		bez.pos_x[1] = 2.0f;
		bez.pos_y[1] = 2.0f;
		bez.pos_z[1] = -1.0f;
		bez.delta[1] = 2;

		bez.pos_x[2] = 3.0f;
		bez.pos_y[2] = 1.0f;
		bez.pos_z[2] = -1.0f;
		bez.delta[2] = 2;

		bez.valid_points = 3;
		bez.time_usec = model.sys.getSynchronizedPX4Time_us();
		control.sendMAVLinkMessage(bez);

	}

	public void enable(boolean enable) {
		this.is_running = enable;
		if(enable ) {
			new Thread(this).start();
		}
	}


	public boolean isStarted() {
		return is_running;
	}

	@Override
	public void run() {
		logger.writeLocalMsg("[msp] Planner started.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		PX4Parameters params = PX4Parameters.getInstance();

		params.sendParameter("COM_OBS_AVOID", 1);

		while(is_running) {
			try { Thread.sleep(200); } catch (InterruptedException e) { }


			control.sendMAVLinkMessage(beat);



		}
		System.out.println("Planner stopped");
		params.sendParameter("COM_OBS_AVOID", 0);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PX4_PLANNER, false);
		logger.writeLocalMsg("[msp] Planner stopped.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);

	}

}
