package com.comino.mavcontrol.offboard3;

import java.util.LinkedList;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.trajectory.minjerk.RapidCollsionDetection;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;
import com.comino.mavcontrol.trajectory.minjerk.struct.Sphere;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;

public class Offboard3CollisionCheck {

	private static final float MIN_DISTANCE_OBSTACLE            = 0.50f;                     // Minimal distance to obstacle
  
	private final RapidCollsionDetection detector = new RapidCollsionDetection();

	
	public boolean isTargetFeasible(DataModel model,GeoTuple4D_F32<?> pos) {
		Sphere obstacle = new Sphere(model.slam.ox,model.slam.oy, model.slam.oz, MIN_DISTANCE_OBSTACLE);
		return isTargetFeasible(obstacle,pos);
	}
	
	public boolean isTargetFeasible(AbstractConvexObject obstacle,GeoTuple4D_F32<?> pos) {
		return !obstacle.isPointInside(new Point3D_F32(pos.x,pos.y,pos.z));
	}
	
	public boolean isTargetFeasible(LinkedList<AbstractConvexObject> obstacles, GeoTuple4D_F32<?> pos) {

		Point3D_F32 p = new Point3D_F32(pos.x,pos.y,pos.z);
		for(AbstractConvexObject obstacle : obstacles) {
			if(obstacle.isPointInside(p))
					return false;
		}
		return true;
	}

	public Offboard3Collision check(RapidTrajectoryGenerator xyzPlanner,LinkedList<AbstractConvexObject> obstacles, float time_section_start ,
			Offboard3Current current, int planningSectionsIndex) {

	    Offboard3Collision collision;
		for(AbstractConvexObject obstacle : obstacles) {
			collision = check(xyzPlanner,obstacle,time_section_start,current,planningSectionsIndex);
			if(collision!=null)
				return collision;
		}
		return null;
	}

	public Offboard3Collision check(RapidTrajectoryGenerator xyzPlanner, DataModel model, 
			                              float time_section_start, Offboard3Current current,int planningSectionsIndex)  {
		return check(xyzPlanner,new Sphere(model.slam.ox,model.slam.oy, model.slam.oz, MIN_DISTANCE_OBSTACLE),time_section_start,current,planningSectionsIndex);
	}
	

	public Offboard3Collision check(RapidTrajectoryGenerator xyzPlanner,AbstractConvexObject obstacle, 
			     float time_section_start, Offboard3Current current, int planningSectionsIndex)  {

		if(!obstacle.isValid() || time_section_start > xyzPlanner.getTotalTime())
			return null;
		
		float time_of_detection = detector.collisionCheck(xyzPlanner, time_section_start, obstacle, RapidTrajectoryGenerator.TIME_STEP);
	    if(time_of_detection >= 0) {
			final Offboard3State state_of_collision = new Offboard3State();
			xyzPlanner.getState(time_of_detection, state_of_collision);
			return new Offboard3Collision(obstacle, time_of_detection, xyzPlanner.getTotalTime(), 
					  current,state_of_collision, planningSectionsIndex);
		}
	    
	    return null;
		
//    Brute Force method 
//	    float time_elapsed = 0; 
//	    final Offboard3State state_of_collision    = new Offboard3State();
//		
//		for(time_elapsed = STEP_TIME; time_elapsed < trajectory_generator.getTotalTime(); time_elapsed += STEP_TIME) {
//
//			trajectory_generator.getState(time_elapsed, state_of_collision);
//			
//
//			// Check only YX distance
//			if(MSP3DUtils.distance3D(state_of_collision.pos(), obstacle.getCenter()) < MIN_DISTANCE_OBSTACLE) { 
//					throw new Offboard3CollisionException(obstacle, time_elapsed, trajectory_generator.getTotalTime(), 
//							  current,state_of_collision, planningSectionsIndex);
//				
//
//			}
//			// TODO Check with current map
//
//		}

	}

}
