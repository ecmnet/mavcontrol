package com.comino.mavcontrol.controllib;

import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;

public class YawSpeedControl {
	
	private static final float RAMP_YAW_SPEED  = MSPMathUtils.toRad(45); // Ramp up Speed for yaw turning
	
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
	
	public void reset() {
		yaw_d_target = 0; yaw_d_current = 0;
	}
	
	public float update(float yaw_diff, float delta_sec) {
		
		if(!Float.isFinite(yaw_diff))
			return 0;
		
		yaw_d_target = MSPMathUtils.normAngle(yaw_diff) / delta_sec * p;
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
	
	public float update(Vector4D_F32 target, Vector4D_F32 current, float delta_sec) {
		
		return update(MSPMathUtils.normAngle2(target.w - current.w), delta_sec);
		
	}

}
