package com.comino.mavcontrol.offboard3.states;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;

public class Offboard3TargetState extends Offboard3State {
	
	public static final GeoTuple4D_F32<Point4D_F32> null_v = new Point4D_F32(0, 0, 0, 0);

	private float t_vel              =  0;
	private float d_sec              = -1;
	private long  t_started_ms       = 0;
	private boolean targetIsSetpoint = false;


	public Offboard3TargetState(GeoTuple4D_F32<?> p) {
		this(p,-1);
		this.tms_us = DataModel.getSynchronizedPX4Time_us();
	}

	public Offboard3TargetState(GeoTuple4D_F32<?> p, float d) {
		this.pos.setTo(p.x,p.y,p.z,p.w);
		this.vel.setTo(0,0,0,Float.NaN);
		this.t_vel = 0;
		this.d_sec = d;
		this.tms_us = DataModel.getSynchronizedPX4Time_us();
		
	}

	public Offboard3TargetState(GeoTuple4D_F32<?> p, GeoTuple4D_F32<?> c, float v, float d_sec) {

		this.pos.setTo(p.x,p.y,p.z,p.w);
		this.t_vel = v;
		this.d_sec = d_sec;
		this.tms_us = DataModel.getSynchronizedPX4Time_us();

		determineTargetVelocity(c);
	}

	public Offboard3TargetState(Point3D_F64 position) {
		this.pos.setTo((float)position.x,(float)position.y, (float)position.z, Float.NaN);
		this.vel.setTo(0,0,0,Float.NaN);
		this.t_vel =  0;
		this.d_sec = -1;
		this.tms_us = DataModel.getSynchronizedPX4Time_us();
	}

	public float getDuration() {
		return d_sec;
	}

	public long getStartedTimestamp() {
		if(t_started_ms == 0)
			t_started_ms = System.currentTimeMillis();
		return t_started_ms;
	}

	public void determineTargetVelocity(GeoTuple4D_F32<?> p_current) {

		vel.setTo(pos.x-p_current.x, pos.y-p_current.y, pos.z-p_current.z, 0);
		MSP3DUtils.replaceNaN3D(vel, null_v);

		float n = vel.norm();

		vel.x = vel.x * t_vel / n;
		vel.y = vel.y * t_vel / n;
		vel.z = vel.z * t_vel / n;
		
	}

	public boolean isPosReached(GeoTuple4D_F32<?> c, float max, float max_yaw) {

		if(isPositionFinite() && Float.isFinite(max)) {

			float dx = pos.x - c.x;
			float dy = pos.y - c.y;
			float dz = pos.z - c.z;

			if(Math.sqrt(dx*dx+dy*dy+dz*dz) > max) {
				return false;
			}
		}

		if(isYawFinite() && Float.isFinite(max_yaw) && normAngleAbs(pos.w,c.w) > max_yaw) {
			return false;	
		}
		return true;
	}

	public boolean isYawReached(GeoTuple4D_F32<?> c, float max_yaw) {
		if(isYawFinite() && Float.isFinite(max_yaw) && normAngleAbs(pos.w,c.w) > max_yaw) {
			return false;	
		}
		return true;
	}
	
	public void setTargetIsSetpoint(boolean flag) {
		targetIsSetpoint = flag;
	}
	
	public boolean isTargetSetpoint() {
		return targetIsSetpoint;
	}
	
	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append(super.toString());
		b.append(" D: "+d_sec);
		return b.toString();
	}


	private float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
	}

	private float normAngle(float a, float b) {
		return normAngle(b-a);
	}

	private float normAngleAbs(float a, float b) {
		return (float)Math.abs(normAngle(a,b));
	}


}
