package com.comino.mavcontrol.offboard3.target;

import georegression.struct.GeoTuple4D_F32;

public class Offboard3VelAccTarget extends Offboard3AbstractTarget {

	public Offboard3VelAccTarget(GeoTuple4D_F32<?> v, GeoTuple4D_F32<?> a, float d_sec) {
		super(TYPE_VEL_ACC,v,a,d_sec);
		this.section_time = d_sec;
	}
	
	public Offboard3VelAccTarget(GeoTuple4D_F32<?> a, float d_sec) {
		super(TYPE_VEL_ACC,null,a,d_sec);
		this.section_time = d_sec;
	}
}
