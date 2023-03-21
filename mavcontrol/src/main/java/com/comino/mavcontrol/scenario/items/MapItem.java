package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.autopilot.AutoPilotBase;

public class MapItem extends AbstractScenarioItem {


	public MapItem(IMAVController control) {
		super(control);
		
	}

	public void setFilename(String filename) {
		
	}

	@Override
	public void execute() {
		
		AutoPilotBase.getInstance().loadMap2D();
		
		completed();
	}

}
