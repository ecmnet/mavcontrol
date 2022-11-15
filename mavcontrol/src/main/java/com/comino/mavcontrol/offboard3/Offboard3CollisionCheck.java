package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.exceptions.Offboard3CollisionException;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

import georegression.struct.point.Point3D_F64;

public class Offboard3CollisionCheck {

	private static final float MIN_DISTANCE_OBSTACLE            = 0.5F;                     // Minimal distance to obstacle
	
	private final RapidTrajectoryGenerator trajectory_generator;

	
	public Offboard3CollisionCheck(RapidTrajectoryGenerator trajectory_generator) {
		this.trajectory_generator = trajectory_generator;
	}


	public void check(LinkedList<Point3D_F64> obstacles, float time_section_start,int planningSectionsIndex) 
			throws Offboard3CollisionException {

		for(Point3D_F64 obstacle : obstacles) {
			check(obstacle,time_section_start,planningSectionsIndex);
		}
	}
	
	public void check(Point3D_F64 obstacle, float time_section_start) 
			throws Offboard3CollisionException {
         check(obstacle,time_section_start,0);
	}

	public void check(Point3D_F64 obstacle, float time_section_start, int planningSectionsIndex) 
			throws Offboard3CollisionException {


		float time_elapsed = 0; float time_step = 0.2f; 
		final Point3D_F64 position     = new Point3D_F64();


		if(!MSP3DUtils.isFinite(obstacle))
			return;

		// TODO: Put collsions check and re-planning into own class

		// TODO: Analytical solution as the line parameters are known

		// TODO get nearest obstacle position into obstacle

		// Brute force method
		for(time_elapsed = time_section_start; time_elapsed < trajectory_generator.getTotalTime(); time_elapsed += time_step) {
			trajectory_generator.getPosition(time_elapsed, position);

			// Check only YX distance
			if(MSP3DUtils.distance2D(position, obstacle) < MIN_DISTANCE_OBSTACLE) { 
				throw new Offboard3CollisionException(time_elapsed, trajectory_generator,planningSectionsIndex);
			}

			// TODO Check with current map

		}

	}

}
