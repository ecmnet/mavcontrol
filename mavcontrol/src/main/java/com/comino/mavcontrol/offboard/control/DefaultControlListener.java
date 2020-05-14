package com.comino.mavcontrol.offboard.control;

import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcontrol.offboard.IOffboardExternalControl;
import com.comino.mavutils.MSPMathUtils;

public class DefaultControlListener implements IOffboardExternalControl {

	private static final float MAX_ACCELERATION		                = 0.3f;                   // Max acceleration in m/s2
	private static final float MAX_SPEED					        = 1.00f;          	      // Default Max speed in m/s
	private static final float MIN_SPEED					        = 0.1f;          	      // Default Min speed in m/s

	private static final float YAW_PV								= 0.005f;				  // P factor yaw control

	private boolean isBreaking  = false;
	private float   speed_incr  = 0;
	private float   acc_incr    = 0;
	private float   delta_angle = 0;


	public boolean determineSpeedAnDirection(float delta_sec, float ela_sec, float eta_sec, Polar3D_F32 spd, Polar3D_F32 path, Polar3D_F32 ctl) {

		ctl.angle_xz =  path.angle_xz;

		delta_angle = MSPMathUtils.normAngle(path.angle_xy - ctl.angle_xy);

		// follow direction changes by a simple P controller
		if(spd.value > 0.5) {
		  ctl.angle_xy = ctl.angle_xy + delta_angle / delta_sec * YAW_PV;
		  // slow down according to the path angle difference
	      speed_incr = - MAX_ACCELERATION * delta_angle * delta_sec;
		}
		else {
		  ctl.angle_xy = path.angle_xy;
    	}

		// start breaking 1.5 secs before reaching the goal
		if(eta_sec < 2.0 )  {
			isBreaking = true;
		}

		if(isBreaking) {
			speed_incr = - spd.value / ( 2 * eta_sec ) * delta_sec;
		} else {
			acc_incr = acc_incr + MAX_ACCELERATION / 2f * delta_sec;
			speed_incr = Math.min(MAX_ACCELERATION, acc_incr) * delta_sec;
		}

		ctl.value = MSPMathUtils.constraint(ctl.value + speed_incr, MAX_SPEED,MIN_SPEED);

		return true;
	}


	public void reset() {
		isBreaking = false; acc_incr = 0; speed_incr = 0;

	}


	public void initialize(Polar3D_F32 spd, Polar3D_F32 path) {
		reset();
	}

}
