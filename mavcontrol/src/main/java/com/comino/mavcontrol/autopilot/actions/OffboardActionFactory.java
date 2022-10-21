package com.comino.mavcontrol.autopilot.actions;

import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Point4D_F32;

public class OffboardActionFactory {
	
	
	public static boolean turn_to(float heading) {
		
		Offboard2Manager offboard = new Offboard2Manager(AutoPilotBase.getInstance().getControl());
		
		//offboard.rotate(heading);
		
		Point4D_F32 target = new Point4D_F32(5,5,-1,Float.NaN);
		offboard.moveTo(target);
		
		return true;
	}
	
	
	
}
