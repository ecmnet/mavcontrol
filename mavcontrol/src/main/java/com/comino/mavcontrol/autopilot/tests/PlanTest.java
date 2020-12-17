package com.comino.mavcontrol.autopilot.tests;

import georegression.struct.point.Vector3D_F32;

public class PlanTest {

	public static void main(String[] args) {

		Vector3D_F32 start       = new Vector3D_F32(2,0,2);
		Vector3D_F32 target      = new Vector3D_F32(4,0,2);
		Vector3D_F32 current     = new Vector3D_F32(1,0,2);
		Vector3D_F32 t           = start.copy();

		target.scale(-1);
		start.plusIP(target);
		start.normalize();
		System.out.println(start);


		current.plusIP(target);
		System.out.println(current);


		float distance = current.dot(start);
		System.out.println(distance);
	}

}
