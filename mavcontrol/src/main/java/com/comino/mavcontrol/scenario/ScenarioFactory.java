package com.comino.mavcontrol.scenario;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.scenario.items.MoveHomeItem;
import com.comino.mavcontrol.scenario.items.PrecisionLandItem;
import com.comino.mavcontrol.scenario.parser.Scenario;

public class ScenarioFactory {
	
	public static Scenario createRTLScenario(IMAVController control,IOffboardControl offboard) {
		Scenario rtl = new Scenario();
		rtl.setMaxSpeed(2.0f);
		rtl.add(new MoveHomeItem(control,offboard));
		rtl.add(new PrecisionLandItem(control,offboard));
		
		return rtl;
	}

}
