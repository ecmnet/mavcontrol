package com.comino.mavcontrol.offboard3.generator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.comino.mavcontrol.offboard3.Offboard3Planner;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosVelTarget;
import com.comino.mavcontrol.trajectory.minjerk.struct.Boundary;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector3D_F32;

public class Offboard3SphereTrajectoryGenerator {

	private static final int   MAX_NUMBER_CANDIDATES = 100;
	private static final float SLOWDOWN_PERCENT      = 10;        // Reduce velocity at avoidancepoint by percentage

	private final Vector3D_F32        tmp   = new Vector3D_F32();

	private final List<Offboard3Plan> candidates = new ArrayList<Offboard3Plan>(MAX_NUMBER_CANDIDATES);

	private int no_valid, no_invalid;


	public Offboard3Plan getAvoidancePlan(Offboard3Planner planner, Offboard3Plan plan, Offboard3Collision col, float distance, float max_velocity ) {

		no_valid = no_invalid = 0;	candidates.clear();

		float time     = plan.getTotalTimeUpTo(col.getPlanningSectionIndex()) + col.getExpectedTimeOfCollision();
		//		float velocity = max_xyz_velocity * (col.getTotalTime() - time) / col.getTotalTime();
		float velocity = max_velocity * (100.0f - SLOWDOWN_PERCENT ) / 100.0f;//MSP3DUtils.norm3D(col.getExpectedStateAtCollision().vel());

		tmp.setTo(col.getCurrent().pos().x,col.getCurrent().pos().y,col.getCurrent().pos().z);
		Boundary tangentPlane = col.getObstacle().getTangentPlane(tmp); 

		GeoTuple3D_F32<?> center = col.getObstacle().getCenter();
		

		long ts = System.nanoTime(); int i =0;
		do {

			Offboard3Plan candidate = new Offboard3Plan();
			Offboard3AbstractTarget target = new Offboard3PosVelTarget(velocity,time);

			generateRandomPoint(target.pos(),center,tangentPlane,distance);

			candidate.add(target);
			candidate.add(new Offboard3PosTarget(plan.getLast().pos()));
			candidate.setEstimatedTime(plan.getEstimatedTime()*2);

			if(planner.planPath(candidate, col.getCurrent())!=null) {
				// Collision detected 
				no_invalid++;
			}
			else if(candidate.isEmpty()) {
				// No collision but not feasible
				no_invalid++;
				MSPStringUtils.getInstance().out("Fasibility violated.");
			}
			else {
				// No collision and feasible
				candidates.add(candidate); no_valid++;
			}

		} while((System.nanoTime() -ts) < 5000_000 && ++i < MAX_NUMBER_CANDIDATES );
	
		

		if(candidates.isEmpty())
			return null;

		Collections.sort(candidates); 

		MSPStringUtils.getInstance().out("Valid candidates: "+no_valid+" Invalid candidates: "+no_invalid);

		return candidates.get(0);
	}


	// Generates random targets on the upper half sphere with the center at a distance from the tangent plane
	public void generateRandomPoint(GeoTuple4D_F32<?> target, GeoTuple3D_F32<?> obstacle, Boundary boundary, float min_distance_from_center) 
	{
		do { 
			tmp.setTo(0,0,0);
			tmp.setTo((float)Math.random()*2f-1f,(float)Math.random()*2f-1f,(float)Math.random()*2f-1f);
			tmp.normalize();
			tmp.scale(min_distance_from_center);
			tmp.setTo(obstacle.x+tmp.x,
					obstacle.y+tmp.y,
					obstacle.z+tmp.z);
			target.setTo(tmp.x,tmp.y,tmp.z,Float.NaN);  
			tmp.setTo(tmp.x - boundary.p.x, tmp.y - boundary.p.y, tmp.z - boundary.p.z);

		} while( tmp.dot(boundary.n) <=0 || target.z > obstacle.z);

	}



}
