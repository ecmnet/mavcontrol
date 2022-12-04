package com.comino.mavcontrol.offboard3.target;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;

public class Offboard3PosTarget extends Offboard3AbstractTarget {


	public Offboard3PosTarget(GeoTuple4D_F32<?> p) {
		super(TYPE_POS,p.x,p.y,p.z,p.w,-1);
	}
	
	public Offboard3PosTarget(GeoTuple4D_F32<?> p, float duration_sec) {
		super(TYPE_POS,p.x,p.y,p.z,p.w,duration_sec);
	}
	
	public Offboard3PosTarget(Point3D_F64 p) {
		super(TYPE_POS,(float)p.x,(float)p.y,(float)p.z, Float.NaN,-1);
	}

}
