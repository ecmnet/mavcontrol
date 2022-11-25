package com.comino.mavcontrol.offboard3.target;

import georegression.struct.GeoTuple4D_F32;

public class Offboard3PosVelTarget extends Offboard3AbstractTarget {

	public Offboard3PosVelTarget(GeoTuple4D_F32<?> p, float v, float d_sec) {
		super(TYPE_POS_VEL,p, v,d_sec);
	}
	
	public Offboard3PosVelTarget(float v, float d) {
		super(TYPE_POS_VEL,Float.NaN, Float.NaN, Float.NaN, Float.NaN, v, d ,true);
	}

}
