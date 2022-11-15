package com.comino.mavcontrol.offboard3.exceptions;

import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;

public class Offboard3CollisionException extends Exception {

	private static final long serialVersionUID = 3081893282526013227L;
	
	private float             time_of_collision;
	private Offboard3State    state_of_collision;
	private int               planning_section_index;
	
	
	public Offboard3CollisionException(float time_of_collision,RapidTrajectoryGenerator trajectory_generator) {
		this(time_of_collision, trajectory_generator,0);
	}
	
	public Offboard3CollisionException(float time_of_collision,RapidTrajectoryGenerator trajectory_generator,int planning_section_index) {
		
		this.time_of_collision      = time_of_collision;
		this.planning_section_index = planning_section_index;
		this.state_of_collision     = new Offboard3State();
		
		trajectory_generator.getPosition    (time_of_collision, state_of_collision.pos());
		trajectory_generator.getVelocity    (time_of_collision, state_of_collision.vel());
		trajectory_generator.getAcceleration(time_of_collision, state_of_collision.acc());
	}

	public Offboard3CollisionException(float time_of_collision, Offboard3State state_of_collsion) {
		this.time_of_collision  = time_of_collision;
		this.state_of_collision = state_of_collsion;
	}
	
	public float getExpectedTimeOfCollision() {
		return time_of_collision;
	}
	
	public Offboard3State getExpectedStateAtCollision() {
		return state_of_collision;
	}
	
	public int getPlanningSectionIndex() {
		return planning_section_index;
	}
	
	
}
