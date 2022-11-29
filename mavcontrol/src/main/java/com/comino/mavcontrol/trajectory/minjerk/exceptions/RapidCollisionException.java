package com.comino.mavcontrol.trajectory.minjerk.exceptions;

public class RapidCollisionException extends Exception {
	
	public float time_of_detection;
	
	public RapidCollisionException(float t) {
		this.time_of_detection = t;
	}

}
