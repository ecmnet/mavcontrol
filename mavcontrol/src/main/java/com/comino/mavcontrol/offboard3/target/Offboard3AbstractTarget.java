package com.comino.mavcontrol.offboard3.target;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3State;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;

public abstract class Offboard3AbstractTarget extends Offboard3State {

	public final static int TYPE_POS       = 1;
	public final static int TYPE_POS_VEL   = 2;
	public final static int TYPE_VEL       = 3;
	public final static int TYPE_VEL_ACC   = 4;


	public static final GeoTuple4D_F32<Point4D_F32> null_v = new Point4D_F32(0, 0, 0, 0);


	protected int     type               =  0;
	protected float   max_velocity       =  Float.NaN;
	protected float   duration           = -1;
	protected float   section_time       =  0;
	protected int     index              =  0;

	protected long    t_started_ns       =  0;
	protected boolean targetIsSetpoint   =  false;
	protected boolean auto_yaw           =  false;


	public Offboard3AbstractTarget(int type, float x, float y, float z, float w, float d_sec) {
		this(type,x,y,z,w,d_sec,Float.isNaN(w));
	}

	public Offboard3AbstractTarget(int type, float duration) {
		this.duration = duration;
		this.type     = type;	
	}
	
	public Offboard3AbstractTarget(int type) {
		this.type     = type;	
	}

	public Offboard3AbstractTarget(int type, float x, float y, float z, float w, float d_sec, boolean auto_yaw) {
		this(type,x,y,z,w,0,d_sec,auto_yaw);
	}

	public Offboard3AbstractTarget(int type, float x, float y, float z, float w, float v, float d_sec, boolean auto_yaw) {
		this.pos.setTo(x,y,z,w);
		this.vel.setTo(0,0,0,Float.NaN);
		this.acc.setTo(0,0,0);

		this.max_velocity = v;
		this.duration = d_sec;
		this.type     = type;	
		this.auto_yaw = auto_yaw;
	}

	public Offboard3AbstractTarget(int type, GeoTuple4D_F32<?> p, float v, float d_sec) {
		this(type,p,v,d_sec,Float.isNaN(p.w));
	}

	public Offboard3AbstractTarget(int type, GeoTuple4D_F32<?> v, GeoTuple4D_F32<?> a, float d_sec) {

		this.pos.setTo(Float.NaN, Float.NaN, Float.NaN,Float.NaN);
		if(v!=null)
			this.vel.setTo(v.x,v.y,v.z,Float.NaN);
		else
			this.vel.setTo(Float.NaN, Float.NaN, Float.NaN,Float.NaN);
		this.acc.setTo(a.x,a.y,a.z);

		if(Float.isNaN(acc.x)) acc.x = 0;
		if(Float.isNaN(acc.y)) acc.y = 0;
		if(Float.isNaN(acc.z)) acc.z = 0;

		this.type  = type;
		this.duration = d_sec;
		this.auto_yaw = true;
	}

	public Offboard3AbstractTarget(int type, GeoTuple4D_F32<?> p, float v, float d_sec, boolean auto_yaw) {

		this.pos.setTo(p.x,p.y,p.z,p.w);
		this.acc.setTo(0,0,0);
		this.max_velocity = v;

		this.type  = type;
		this.duration = d_sec;
		this.auto_yaw = auto_yaw;
	}

	public float getDuration() {
		return duration;
	}

	public void setDuration( float d) {
		this.duration = d;
	}

	public boolean isAutoYaw() {
		return auto_yaw;
	}

	public void setAutoYaw(boolean auto_yaw) {
		this.auto_yaw = auto_yaw;
	}

	public int getType() {
		return type;
	}

	public int getIndex() {
		return index;
	}

	public void setIndex(int i) {
		this.index = i;
	}

	public long getStartedTimestamp() {
		if(t_started_ns == 0)
			t_started_ns = System.nanoTime();
		return t_started_ns;
	}

	public float getElapsedTime() {
		if(t_started_ns == 0)
			t_started_ns = System.nanoTime();
		return (System.nanoTime() - t_started_ns ) / 1_000_000_000.0f;
	}


	public void determineTargetVelocity(GeoTuple4D_F32<?> p_current) {
		
		vel.setTo(pos.x-p_current.x, pos.y-p_current.y, pos.z-p_current.z, 0);
		MSP3DUtils.replaceNaN3D(vel, null_v);

		float n = MSP3DUtils.norm3D(vel);

		vel.x = vel.x * max_velocity / n;
		vel.y = vel.y * max_velocity / n;
		vel.z = vel.z * max_velocity / n;

	}

	public boolean isVelReached(GeoTuple4D_F32<?> c, float max) {

		if(isVelocityFinite() && Float.isFinite(max)) {

			float dx = vel.x - c.x;
			float dy = vel.y - c.y;
			float dz = vel.z - c.z;

			if((dx*dx+dy*dy+dz*dz) > max*max) {
				return false;
			}
			return true;
		}

		return false;
	}

	public boolean isPosReached(GeoTuple4D_F32<?> c, float max, float max_yaw) {

		if(isPositionFinite() && Float.isFinite(max)) {

			float dx = pos.x - c.x;
			float dy = pos.y - c.y;
			float dz = pos.z - c.z;

			if((dx*dx+dy*dy+dz*dz) > max*max) {
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

	public void setPlannedSectionTime(float t) {
		section_time = t;
	}

	public float getPlannedSectionTime() {
		return section_time;
	}

	public boolean isTargetSetpoint() {
		return targetIsSetpoint;
	}

	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append(super.toString());
		b.append("Est.: "+f.format(duration)+"s ");
		b.append("Plan: "+f.format(section_time)+"s ");
		b.append("Type: "+type+" ");
		b.append("Yaw: "+auto_yaw);
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
