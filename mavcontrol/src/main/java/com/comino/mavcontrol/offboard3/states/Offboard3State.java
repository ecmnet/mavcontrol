package com.comino.mavcontrol.offboard3.states;

import java.text.DecimalFormat;

import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point4D_F32;

public class Offboard3State {

	protected final GeoTuple4D_F32<?> pos  = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple4D_F32<?> vel  = new Point4D_F32(0, 0, 0, Float.NaN);
	protected final GeoTuple3D_F32<?> acc  = new Point3D_F32(0,0,0);
	
	
	protected final static DecimalFormat f = new DecimalFormat(" #0.00;-#0.00");
	protected final static DecimalFormat r = new DecimalFormat(" #0.0°;-#0.0°");
	
	
	public Offboard3State() {
		
	}
	
   public Offboard3State(GeoTuple4D_F32<?> pos, GeoTuple4D_F32<?> vel, GeoTuple4D_F32<?> acc) {
		this.pos.setTo(pos.x,pos.y,pos.z,pos.w);
		this.vel.setTo(vel.x,vel.y,vel.z,vel.w);
		this.acc.setTo(acc.x,acc.y,acc.z);
	}
   
   public Offboard3State(GeoTuple4D_F32<?> pos, GeoTuple4D_F32<?> vel, GeoTuple3D_F32<?> acc) {
		this.pos.setTo(pos.x,pos.y,pos.z,pos.w);
		this.vel.setTo(vel.x,vel.y,vel.z,vel.w);
		this.acc.setTo(acc.x,acc.y,acc.z);
	}
	
	public GeoTuple4D_F32<?> pos() {
		return pos;
	}
	
	public GeoTuple4D_F32<?> vel() {
		return vel;
	}
	
	public GeoTuple3D_F32<?> acc() {
		return acc;
	}
	

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

	public boolean replaceNaNPositionBy(GeoTuple4D_F32<?> source) {
		if (Float.isNaN(pos.x)) pos.x = source.x;
		if (Float.isNaN(pos.y)) pos.y = source.y;
		if (Float.isNaN(pos.z)) pos.z = source.z;
		return isPositionFinite();
	}
	
	public boolean replaceNaNVelocityBy(GeoTuple4D_F32<?> source) {
		if (Float.isNaN(vel.x)) vel.x = source.x;
		if (Float.isNaN(vel.y)) vel.y = source.y;
		if (Float.isNaN(vel.z)) vel.z = source.z;
		return isVelocityFinite();
	}
	
	public void clear() {
		pos.setTo(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		vel.setTo(0, 0, 0, Float.NaN);
		acc.setTo(0,0,0);
	}
	
	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append("Pos: ("+f.format(pos.x)+","+f.format(pos.y)+","+f.format(pos.z)+")"+","+r.format(MSPMathUtils.fromRad(pos.w))); 
		b.append("\t");
		b.append("Vel: ("+f.format(vel.x)+","+f.format(vel.y)+","+f.format(vel.z)+")"); 
		b.append("\t");
		b.append("Acc: ("+f.format(acc.x)+","+f.format(acc.y)+","+f.format(acc.z)+")"); 
		b.append("\t");
		return b.toString();
	}
	
	public void set(RapidTrajectoryGenerator planner, float t) {
		planner.getPosition(    t, this.pos);
		planner.getVelocity(    t, this.vel);
		planner.getAcceleration(t, this.acc);
	}
	
	public Offboard3State clone() {
		return new Offboard3State(this.pos,this.vel,this.acc);	
	}

}
