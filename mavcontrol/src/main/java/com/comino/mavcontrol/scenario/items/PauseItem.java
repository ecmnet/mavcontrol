package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.IOffboardControl;

public class PauseItem extends AbstractScenarioItem {

	public PauseItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
	}

	@Override
	public void execute() {
		  completed();
	}

}
