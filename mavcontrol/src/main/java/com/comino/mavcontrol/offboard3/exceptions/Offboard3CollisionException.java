package com.comino.mavcontrol.offboard3.exceptions;

import com.comino.mavcontrol.offboard3.states.Offboard3State;

public class Offboard3CollisionException extends Exception {

	private static final long serialVersionUID = 3081893282526013227L;
	
	private float             time_of_collision;
	private Offboard3State    state_of_collision;
	private int               planning_section_index;

	public Offboard3CollisionException(float time_of_collision, Offboard3State state_of_collision) {
		this(time_of_collision,state_of_collision,0);
	}

	public Offboard3CollisionException(float time_of_collision, Offboard3State state_of_collision, int planning_section_index) {
		
		this.time_of_collision      = time_of_collision;
		this.state_of_collision     = state_of_collision;
		this.planning_section_index = planning_section_index;
		
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
