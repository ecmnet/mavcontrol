package com.comino.mavcontrol.offboard3;

import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point4D_F32;

public class Offboard3Target {
	
	public static final GeoTuple4D_F32<Point4D_F32> null_v = new Point4D_F32(0, 0, 0, 0);

	// Targets
	private final GeoTuple4D_F32<Point4D_F32> p = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	private final GeoTuple4D_F32<Point4D_F32> v = new Point4D_F32(0, 0, 0, Float.NaN);
	private final GeoTuple3D_F32<Point3D_F32> a = new Point3D_F32(0,0,0);

	private float vel    =  0;
	private float d_sec  = -1;

	private long  t_started_ms = 0;


	public Offboard3Target(GeoTuple4D_F32<?> p) {
		this(p,0,-1);
	}

	public Offboard3Target(GeoTuple4D_F32<?> p, float d) {
		this(p,0,d);
	}

	public Offboard3Target(GeoTuple4D_F32<?> p, float v, float d_sec) {
		this.p.setTo(p.x,p.y,p.z,p.w);
		this.vel   = v;
		this.d_sec = d_sec;
	}

	public Offboard3Target(GeoTuple4D_F32<?> p, GeoTuple4D_F32<?> c, float v, float d_sec) {

		this.p.setTo(p.x,p.y,p.z,p.w);
		this.vel   = v;
		this.d_sec = d_sec;

		determineTargetVelocity(c);
	}

	public float getDuration() {
		return d_sec;
	}

	public long getStartedTimestamp() {
		if(t_started_ms == 0)
			t_started_ms = System.currentTimeMillis();
		return t_started_ms;
	}

	public GeoTuple4D_F32<?> getTargetPosition() {
		return p;
	}

	public GeoTuple4D_F32<?> getTargetVelocity() {
		return v;
	}

	public GeoTuple3D_F32<?> getTargetAcceleration() {
		return a;
	}

	public boolean isPositionFinite() {
		return Float.isFinite(p.x) && Float.isFinite(p.y) && Float.isFinite(p.z);
	}

	public void determineTargetVelocity(GeoTuple4D_F32<?> p_current) {

		v.setTo(p.x-p_current.x, p.y-p_current.y, p.z-p_current.z, 0);
		MSP3DUtils.replaceNaN3D(v, null_v);

		float n = v.norm();

		v.x = v.x * vel / n;
		v.y = v.y * vel / n;
		v.z = v.z * vel / n;
		
	}

	public boolean isPosReached(GeoTuple4D_F32<?> c, float max, float max_yaw) {

		if(Float.isFinite(p.x) && Float.isFinite(p.y) && Float.isFinite(p.z) && Float.isFinite(max)) {

			float dx = p.x - c.x;
			float dy = p.y - c.y;
			float dz = p.z - c.z;

			if(Math.sqrt(dx*dx+dy*dy+dz*dz) > max) {
				//System.err.println("=> position not reached");
				return false;
			}
		}

		if(Float.isFinite(p.w) && Float.isFinite(max_yaw) && normAngleAbs(p.w,c.w) > max_yaw) {
			//System.err.println("=> yaw not reached");
			return false;	
		}
		return true;
	}

	public boolean isYawReached(GeoTuple4D_F32<?> c, float max_yaw) {
		if(Float.isFinite(p.w) && Float.isFinite(max_yaw) && normAngleAbs(p.w,c.w) > max_yaw) {
			return false;	
		}
		return true;
	}
	
	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append(" Pos: "+p);
		b.append(" Vel: "+v);
		b.append(" Acc: "+a);
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
