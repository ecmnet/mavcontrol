package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.autopilot.actions.OffboardActionFactory;

public class PrecisionLandItem extends AbstractScenarioItem {


	public PrecisionLandItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
	}


	@Override
	public void execute() {
		OffboardActionFactory.precision_landing_rotate();
		while(!model.sys.isStatus(Status.MSP_LANDED)) {
			wait(100);
			completed();
		}
	}


}
