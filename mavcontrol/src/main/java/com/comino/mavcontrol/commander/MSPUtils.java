package com.comino.mavcontrol.commander;

public class MSPUtils {
	
	private final static String[] enable_disable = { "disabled", "enabeld" };
	
	public static String getEnableDisable(boolean v) {
		return v ? enable_disable[1] : enable_disable[0];
	}

}
