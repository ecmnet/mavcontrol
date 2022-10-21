package com.comino.mavcontrol.autopilot.actions;

import com.comino.mavcontrol.offboard2.Offboard2Manager;

public class OffboardActionFactory {


	public static boolean turn_to(float heading) {

		Offboard2Manager offboard = Offboard2Manager.getInstance();
		if(offboard!=null) {
			offboard.rotate(heading);
			return true;
		}
		return false;
	}
	
	public static boolean move_to(float x, float y,float z) {

		Offboard2Manager offboard = Offboard2Manager.getInstance();
		if(offboard!=null) {
			offboard.moveTo(x,y,z,Float.NaN);
			return true;
		}
		return false;
	}



}
