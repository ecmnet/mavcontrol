package com.comino.mavcontrol.offboard3.states;

import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;

public class Offboard3Collision extends Exception {

	private static final long serialVersionUID = 3081893282526013227L;
	
	private float                time_of_collision;
	private float                total_time;
	private Offboard3State       state_of_collision;
	private Offboard3Current     current;
	private AbstractConvexObject obstacle;
	private int                  planning_section_index;
	
	
	public Offboard3Collision() {
		super();
	}

	public Offboard3Collision(AbstractConvexObject obstacle,float time_of_collision,float total_time,Offboard3Current current,Offboard3State state_of_collision) {
		this(obstacle,time_of_collision,total_time,current,state_of_collision,0);
	}

	public Offboard3Collision(AbstractConvexObject obstacle,float time_of_collision, float total_time, Offboard3Current current,Offboard3State state_of_collision, int planning_section_index) {
		
		this.time_of_collision      = time_of_collision;
		this.total_time             = total_time;
		this.state_of_collision     = state_of_collision;
		this.obstacle               = obstacle;
		this.current                = current;
		this.planning_section_index = planning_section_index;
		
	}
	
	public float getExpectedTimeOfCollision() {
		return time_of_collision;
	}
	
	public float getTotalTime() {
		return total_time;
	}
	
	public Offboard3State getExpectedStateAtCollision() {
		return state_of_collision;
	}
	
	public Offboard3Current getCurrent() {
		return current;
	}
	
	public AbstractConvexObject getObstacle() {
		return obstacle;
	}
	
	public int getPlanningSectionIndex() {
		return planning_section_index;
	}
	
	
}
