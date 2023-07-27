package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.autopilot.actions.TakeOffHandler;

public class TakeOffItem extends AbstractScenarioItem {

	private final TakeOffHandler takeoff_handler;

	private float altitude_m = Float.NaN;

	public TakeOffItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
		this.takeoff_handler = new TakeOffHandler(control,() -> completed(), () -> abort());
	}

	public void setTakeoffAltitude(float altitude_m) {
		this.altitude_m  = altitude_m;
	}

	@Override
	public void execute() {
		if(model.sys.isStatus(Status.MSP_LANDED))
			takeoff_handler.initiateTakeoff(2);
		else {
			control.writeLogMessage(new LogMessage("[msp] Takeoff skipped. Already in air.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			completed();
		}
	}
	
	public long getTimeout_ms() {
		return 20_000L;
	}

	@Override
	public void initialize() {
		super.initialize();
		if(Float.isFinite(altitude_m) && altitude_m > 0)
			takeoff_handler.setTakeoffAltitude(altitude_m);
	}

}
