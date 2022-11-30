package com.comino.mavcontrol.trajectory.minjerk.struct;

import georegression.struct.GeoTuple3D_F32;

public abstract class AbstractConvexObject {
	
	public abstract Boundary getTangentPlane(GeoTuple3D_F32<?> p); 
	public abstract boolean  isPointInside(GeoTuple3D_F32<?> p); 
	public abstract boolean  isValid();
	

}
