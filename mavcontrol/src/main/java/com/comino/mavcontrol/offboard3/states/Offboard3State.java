package com.comino.mavcontrol.offboard3.states;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;

public class Offboard3State {
	
	protected final GeoTuple4D_F32<?> pos  = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple4D_F32<?> vel  = new Point4D_F32(0, 0, 0, Float.NaN);
	protected final GeoTuple4D_F32<?> acc  = new Point4D_F32(0,0,0,0);
	
	
	public boolean isPositionFinite() {
		return Float.isFinite(pos.x) && Float.isFinite(pos.y) && Float.isFinite(pos.z);
	}
	
	public boolean isVelocityFinite() {
		return Float.isFinite(vel.x) && Float.isFinite(vel.y) && Float.isFinite(vel.z);
	}
	
	public boolean isAccelerationFinite() {
		return Float.isFinite(acc.x) && Float.isFinite(acc.y) && Float.isFinite(acc.z);
	}
	
	public boolean isYawFinite() {
		return Float.isFinite(pos.w);
	}
	
	public boolean isYawRateFinite() {
		return Float.isFinite(vel.w);
	}
	
	public boolean isStateFinite() {
		return isPositionFinite() && isVelocityFinite() && isAccelerationFinite();
	}

}
