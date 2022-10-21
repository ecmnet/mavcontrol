package com.comino.mavcontrol.autopilot.actions;

import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavutils.MSPMathUtils;

public class OffboardActionFactory {
	
	
	public static boolean turn_to(float heading) {
		
		Offboard2Manager offboard = new Offboard2Manager(AutoPilotBase.getInstance().getControl());
		offboard.rotate(heading);
		
		return true;
	}
	
	
	
}
