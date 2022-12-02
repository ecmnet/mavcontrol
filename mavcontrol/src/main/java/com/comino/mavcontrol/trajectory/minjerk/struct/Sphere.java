package com.comino.mavcontrol.trajectory.minjerk.struct;

import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.point.Point3D_F32;

public class Sphere extends AbstractConvexObject {
	
	private float radius;
	
	public Sphere(GeoTuple3D_F32<?> center, float radius) {
		super(center);
		this.radius = radius;
	}
	
	public Sphere(float x, float y, float z, float radius) {
		super(new Point3D_F32(x,y,z));
		this.radius = radius;
	}
	
	public boolean isValid() {
		return MSP3DUtils.isFinite(center) && radius > 0;
	}
	
	public float getRadius() {
		return radius;
	}

	@Override
	public Boundary getTangentPlane(GeoTuple3D_F32<?> p) {
        Boundary boundary = new Boundary();
        boundary.n.setTo(p.x - center.x, p.y - center.y,p.z - center.z); boundary.n.normalize();
        boundary.p.setTo(boundary.n); boundary.p.scale(radius); boundary.p.plusIP(center);
		return boundary;
	}

	@Override
	public boolean isPointInside(GeoTuple3D_F32<?> p) {
		Point3D_F32 k = new Point3D_F32(center);  k.scale(-1); k.plusIP(p);	
		return k.norm() <= radius;
	}
	

}
