package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.autopilot.actions.TakeOffHandler;

public class TakeOffItem extends AbstractScenarioItem {
	
	private final TakeOffHandler takeoff_handler;
	
	private float altitude_m = Float.NaN;

	public TakeOffItem(IMAVController control) {
	  super(AbstractScenarioItem.TYPE_TAKEOFF,control);
	  this.takeoff_handler = new TakeOffHandler(control,() -> completed());
	}
	
	public void setTakeoffAltitude(float altitude_m) {
		this.altitude_m  = altitude_m;
	}

	@Override
	public void execute() {
		takeoff_handler.initiateTakeoff(10);
	}

	@Override
	public void initialize() {
		if(Float.isFinite(altitude_m) && altitude_m > 0)
			takeoff_handler.setTakeoffAltitude(altitude_m);
	}

}
