package com.comino.mavcontrol.offboard3.target;

import georegression.struct.GeoTuple4D_F32;

public class Offboard3VelTarget extends Offboard3AbstractTarget {

	public Offboard3VelTarget(GeoTuple4D_F32<?> p, float v, float d_sec) {
		super(TYPE_VEL,p,v,d_sec);
	}

}
