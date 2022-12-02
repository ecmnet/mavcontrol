package com.comino.mavcontrol.offboard3.generator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import com.comino.mavcontrol.offboard3.Offboard3Planner;
import com.comino.mavcontrol.offboard3.exceptions.Offboard3CollisionException;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosVelTarget;
import com.comino.mavcontrol.trajectory.minjerk.struct.Boundary;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.point.Vector3D_F32;

public class Offboard3SphereTrajectoryGenerator {

	private static final int NUMBER_CANDIDATES = 100;

	private final Random              gauss = new Random();
	private final Vector3D_F32        tmp   = new Vector3D_F32();

	private final float               max_xyz_velocity;
	
	private final List<Offboard3Plan> candidates        = new ArrayList<Offboard3Plan>();
	private final List<Offboard3Plan> invalid_candidates = new ArrayList<Offboard3Plan>();


	public Offboard3SphereTrajectoryGenerator(float max_velocity) {
		this.max_xyz_velocity = max_velocity;
	}


	public Offboard3Plan getAvoidancePlan(Offboard3Planner planner, Offboard3Plan plan, Offboard3CollisionException col, float distance) {

		invalid_candidates.clear();
		
		generateNewAvoidanceCandidates(plan,col,distance);

		for(Offboard3Plan candidate : candidates) {
			try {
				planner.planPath(candidate, col.getCurrent());
			} catch (Offboard3CollisionException e) {
				invalid_candidates.add(candidate);
			}
			if(candidate.getTotalTime() <= 0) {
				System.err.println("Candidate with 0 time generated! -> removed");
				invalid_candidates.add(candidate);
			}
		}
		
		candidates.removeAll(invalid_candidates);
		
		if(candidates.isEmpty())
			return null;
		
		Collections.sort(candidates); 

		return candidates.get(0);
	}

	public void generateNewAvoidanceCandidates(Offboard3Plan plan, Offboard3CollisionException col, float distance) {

		candidates.clear();
		
		for(int i=0; i<NUMBER_CANDIDATES;i++ ) {
			
			Offboard3Plan candidate = new Offboard3Plan();
			Offboard3AbstractTarget target = generateRandomPoint(plan,col,distance);
			candidate.add(target);
			candidate.add(new Offboard3PosTarget(plan.getFirst().pos()));
			candidates.add(candidate);
		}
	}

	// Generates random targets on the half sphere with the center at a distance from the tangent plane
	public Offboard3AbstractTarget generateRandomPoint(Offboard3Plan plan,Offboard3CollisionException col,float min_distance_from_center) {

		float time     = plan.getTotalTimeUpTo(col.getPlanningSectionIndex()) + col.getExpectedTimeOfCollision();
	//	float velocity = max_xyz_velocity * (col.getTotalTime() - time) / col.getTotalTime();
		float velocity = max_xyz_velocity * 2.0f / 3.0f;//MSP3DUtils.norm3D(col.getExpectedStateAtCollision().vel());

		Offboard3AbstractTarget target = new Offboard3PosVelTarget(velocity,time);

		tmp.setTo(col.getCurrent().pos().x,col.getCurrent().pos().y,col.getCurrent().pos().z);
		Boundary tangentPlane = col.getObstacle().getTangentPlane(tmp); 
	//	do {
			tmp.setTo((float)gauss.nextGaussian(),(float)gauss.nextGaussian(),(float)gauss.nextGaussian()+tmp.z);
			tmp.normalize();
			tmp.scale(min_distance_from_center);
			tmp.setTo(col.getObstacle().getCenter().x+tmp.x,
					  col.getObstacle().getCenter().y+tmp.y,
					  col.getObstacle().getCenter().z+tmp.z);
			target.pos().setTo(tmp.x,tmp.y,tmp.z,Float.NaN);  
			tmp.setTo(tmp.x - tangentPlane.p.x, tmp.y - tangentPlane.p.y, tmp.z - tangentPlane.p.z);
	//	} while( tmp.dot(tangentPlane.n) <=0);

		return target;
	}


}
