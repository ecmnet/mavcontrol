package com.comino.mavcontrol.offboard.control;

import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcontrol.offboard.IOffboardExternalConstraints;

public class DefaultConstraintListener implements IOffboardExternalConstraints {

	@Override
	public boolean get(float delta_sec, Polar3D_F32 speed, Polar3D_F32 path, Polar3D_F32 control) {
		return false;
	}

}
