package com.comino.mavcontrol.commander;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavutils.workqueue.WorkQueue;

public class StatusCheck implements Runnable {

	private final DataModel      model;
	private final IMAVController control;

	private final WorkQueue wq = WorkQueue.getInstance();


	private boolean isRunning = false;
	private int     worker_id = 0;


	public StatusCheck(IMAVController control) {

		super();
		this.model   = control.getCurrentModel();
		this.control = control;
	}

	public void start() {
		if (isRunning)
			return;
		System.out.println("StatsCheck started...");
		worker_id = wq.addCyclicTask("LP", 200, this);
		isRunning = true;
	}

	public void stop() {
		if(isRunning) {
			isRunning = false;
			wq.removeTask("LP", worker_id);
		}
	}


	public boolean checkFlightReadiness(boolean logging) {

		boolean is_ready = true;

		if(logging)
			control.writeLogMessage(new LogMessage("[msp] Performing status check...",MAV_SEVERITY.MAV_SEVERITY_DEBUG));


		if (model.sys.isStatus(Status.MSP_CONNECTED) && !model.sys.isStatus(Status.MSP_SITL)
				&& !model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FCUM)) {

			// Checks for MSP driven vehicles


			if (!model.sys.isSensorAvailable(Status.MSP_PIX4FLOW_AVAILABILITY) && !model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] No Flow or Vision sensor available.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if (!model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY) && model.vision.isStatus(Vision.ENABLED)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] Vision not enabled.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if ((Float.isNaN(model.vision.x) || Float.isNaN(model.vision.y) || Float.isNaN(model.vision.z))
					&& model.vision.isStatus(Vision.ENABLED)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] Vision does not provide any data.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if (!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] Distance sensor not available.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if(!model.sys.isStatus(Status.MSP_RC_ATTACHED)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] RC not attached",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

		}

		if (model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY)) {

			if(!model.sys.isStatus(Status.MSP_GPOS_VALID)) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] No global position.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if (model.est.posVertAccuracy > 1.0f) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] GPS vertical accurracy.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

			if (model.est.posHorizAccuracy > 0.5f) {
				if(logging)
					control.writeLogMessage(new LogMessage("[msp] GPS horizontal accurracy.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
				is_ready = false;
			}

		}

		if (!model.sys.isStatus(Status.MSP_GCL_CONNECTED)) {
			if(logging)
				control.writeLogMessage(new LogMessage("[msp] GC not connected.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return false;
		}

		if (!model.sys.isStatus(Status.MSP_LPOS_VALID)) {
			if(logging)
				control.writeLogMessage(new LogMessage("[msp] No local position.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return false;
		}

		if(Float.isNaN(model.hud.ag)) {
			if(logging)
				control.writeLogMessage(new LogMessage("[msp] No AMSL altitude.",MAV_SEVERITY.MAV_SEVERITY_WARNING));
			is_ready = false;
		}

		return is_ready;
	}

	@Override
	public void run() {
		if (model.sys.isStatus(Status.MSP_ACTIVE))
			model.sys.setStatus(Status.MSP_READY_FOR_FLIGHT, checkFlightReadiness(false));
	}




}
