package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.autopilot.AutoPilotBase;

public class MapItem extends AbstractScenarioItem {


	public MapItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
		
	}

	public void setFilename(String filename) {
		
	}

	@Override
	public void execute() {
		
		AutoPilotBase.getInstance().getMapper().loadMap2D();
		
		completed();
	}

}
