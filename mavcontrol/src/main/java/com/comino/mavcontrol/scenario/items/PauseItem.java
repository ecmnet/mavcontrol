package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;

public class PauseItem extends AbstractScenarioItem {

	public PauseItem(IMAVController control) {
		super(control);
	}

	@Override
	public void execute() {
		  completed();
	}

}
