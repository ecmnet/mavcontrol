package com.comino.mavcontrol.offboard.control;

import com.comino.mavcontrol.offboard.IOffboardExternalConstraints;
import com.comino.mavmap.struct.Polar3D_F32;

public class DefaultConstraintListener implements IOffboardExternalConstraints {

	@Override
	public boolean get(float delta_sec, Polar3D_F32 speed, Polar3D_F32 path, Polar3D_F32 control) {
		return false;
	}

}
