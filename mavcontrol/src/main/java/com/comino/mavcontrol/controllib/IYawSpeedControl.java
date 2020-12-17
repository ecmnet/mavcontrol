package com.comino.mavcontrol.controllib;

import georegression.struct.point.Vector4D_F32;

public interface IYawSpeedControl {

	void reset();

	float update(Vector4D_F32 target, Vector4D_F32 current, float delta_sec);
	float update(float yaw_diff, float delta_sec);

}