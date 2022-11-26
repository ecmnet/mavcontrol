package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.exceptions.Offboard3CollisionException;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

import georegression.struct.point.Point3D_F32;

public class Offboard3CollisionCheck {

	private static final float MIN_DISTANCE_OBSTACLE            = 0.5f;                     // Minimal distance to obstacle
	private static final float STEP_TIME                        = 0.05f;                    // Time steps to check

	private final RapidTrajectoryGenerator trajectory_generator;


	public Offboard3CollisionCheck(RapidTrajectoryGenerator trajectory_generator) {
		this.trajectory_generator = trajectory_generator;
	}


	public void check(LinkedList<Point3D_F32> obstacles, float time_section_start, int planningSectionsIndex) 
			throws Offboard3CollisionException {

		for(Point3D_F32 obstacle : obstacles) {
			check(obstacle,time_section_start,planningSectionsIndex);
		}
	}

	public void check(Point3D_F32 obstacle, float time_section_start) 
			throws Offboard3CollisionException {
		check(obstacle,time_section_start, 0);
	}

	public void check(Point3D_F32 obstacle, float time_section_start, int planningSectionsIndex) 
			throws Offboard3CollisionException {

		if(!MSP3DUtils.isFinite(obstacle) || time_section_start > trajectory_generator.getTotalTime())
			return;

		float time_elapsed = 0; 
		final Offboard3State state_of_collision    = new Offboard3State();


		// TODO: Analytical solution as the line parameters are known

		// TODO get nearest obstacle position into obstacle

		// Brute force method
		for(time_elapsed = STEP_TIME; time_elapsed < trajectory_generator.getTotalTime(); time_elapsed += STEP_TIME) {

			trajectory_generator.getState(time_elapsed, state_of_collision);

			// Check only YX distance
			if(MSP3DUtils.distance2D(state_of_collision.pos(), obstacle) < MIN_DISTANCE_OBSTACLE) { 
					throw new Offboard3CollisionException(time_elapsed, trajectory_generator.getTotalTime(), state_of_collision, planningSectionsIndex);
				

			}
			// TODO Check with current map

		}

	}

}
