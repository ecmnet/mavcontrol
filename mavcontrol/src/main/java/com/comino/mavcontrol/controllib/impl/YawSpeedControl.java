package com.comino.mavcontrol.controllib.impl;

import com.comino.mavcontrol.controllib.IYawSpeedControl;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;

public class YawSpeedControl implements IYawSpeedControl {
	
	private static final float RAMP_YAW_SPEED  = MSPMathUtils.toRad(90); // Ramp up Speed for yaw turning
	private static final float NO_CONTROL      = MSPMathUtils.toRad(0.5);
	
	private float yaw_d_target   = 0;
	private float yaw_d_current  = 0;
	private float p              = 0;
	private float min_yaw_speed  = 0;
	private float max_yaw_speed  = 0;
	
	public YawSpeedControl(float p, float min_yaw_speed, float max_yaw_speed) {
		this.p  = p;
		this.min_yaw_speed = min_yaw_speed;
		this.max_yaw_speed = max_yaw_speed;
	}
	
	@Override
	public void reset() {
		yaw_d_target = 0; yaw_d_current = 0;
	}
	
	@Override
	public float update(float yaw_diff, float delta_sec) {
		return update(yaw_diff,delta_sec, max_yaw_speed);
	}
	
	
	public float update(float yaw_diff, float delta_sec, float max_yaw_speed) {
		
		if(!Float.isFinite(yaw_diff) || Math.abs(yaw_diff) < NO_CONTROL || delta_sec < 0.001) {
			return 0;
		}
		
		yaw_d_target = MSPMathUtils.normAngle2(yaw_diff) * p / delta_sec;
		if(yaw_d_target > 0) {
			yaw_d_current = yaw_d_current + (RAMP_YAW_SPEED * delta_sec);
			if(yaw_d_current > yaw_d_target)
				yaw_d_current = yaw_d_target;
		} else if(yaw_d_target < 0) {
			yaw_d_current = yaw_d_current - (RAMP_YAW_SPEED * delta_sec );
			if(yaw_d_current < yaw_d_target)
				yaw_d_current = yaw_d_target;
		}
		
		// limit min yaw speed
		if(Math.abs(yaw_d_current)< min_yaw_speed)
			return min_yaw_speed * Math.signum(yaw_d_current);

		if(Math.abs(yaw_d_current)>max_yaw_speed)
			return max_yaw_speed * Math.signum(yaw_d_current);
		
		return yaw_d_current;
		
	}
	
	@Override
	public float update(Vector4D_F32 target, Vector4D_F32 current, float delta_sec) {
		
		return update(MSPMathUtils.normAngle2(target.w - current.w), delta_sec);
		
	}

}
