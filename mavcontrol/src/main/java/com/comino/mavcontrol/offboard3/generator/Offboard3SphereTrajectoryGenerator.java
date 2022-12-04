package com.comino.mavcontrol.offboard3.generator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import com.comino.mavcontrol.offboard3.Offboard3Planner;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosVelTarget;
import com.comino.mavcontrol.offboard3.utils.RuntimeAnalysis;
import com.comino.mavcontrol.trajectory.minjerk.struct.Boundary;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector3D_F32;

public class Offboard3SphereTrajectoryGenerator {

	private static final int MAX_NUMBER_CANDIDATES = 200;

	private final Random              gauss = new Random(System.nanoTime());
	private final Vector3D_F32        tmp   = new Vector3D_F32();

	private final float               max_xyz_velocity;

	private final List<Offboard3Plan> candidates         = new ArrayList<Offboard3Plan>(MAX_NUMBER_CANDIDATES);

	private int no_valid, no_invalid;


	public Offboard3SphereTrajectoryGenerator(float max_velocity) {
		this.max_xyz_velocity = max_velocity;
	}

	public Offboard3Plan getAvoidancePlan(Offboard3Planner planner, Offboard3Plan plan, Offboard3Collision col, float distance) {

		RuntimeAnalysis.start();

		no_valid = no_invalid = 0;	candidates.clear();

		float time     = plan.getTotalTimeUpTo(col.getPlanningSectionIndex()) + col.getExpectedTimeOfCollision();
		//		float velocity = max_xyz_velocity * (col.getTotalTime() - time) / col.getTotalTime();
		float velocity = max_xyz_velocity * 2.0f / 3.0f;//MSP3DUtils.norm3D(col.getExpectedStateAtCollision().vel());

		tmp.setTo(col.getCurrent().pos().x,col.getCurrent().pos().y,col.getCurrent().pos().z);
		Boundary tangentPlane = col.getObstacle().getTangentPlane(tmp); 

		GeoTuple3D_F32<?> center = col.getObstacle().getCenter();

		long ts = System.nanoTime(); int i =0;
		do {

			Offboard3Plan candidate = new Offboard3Plan();
			Offboard3AbstractTarget target = new Offboard3PosVelTarget(velocity,time);

			generateRandomPoint(target.pos(),center,tangentPlane,distance);

			candidate.add(target);
			candidate.add(new Offboard3PosTarget(plan.getFirst().pos()));

			if(planner.planPath(candidate, col.getCurrent())!=null)
				no_invalid++;
			else
				candidates.add(candidate); no_valid++;


		} while((System.nanoTime() -ts) < 2500_000 && ++i < MAX_NUMBER_CANDIDATES );

		if(candidates.isEmpty())
			return null;

		Collections.sort(candidates); 

		RuntimeAnalysis.end();
		
		MSPStringUtils.getInstance().out("Valid candidates: "+no_valid+" Invalid candidates: "+no_invalid);


		return candidates.get(0);
	}


	// Generates random targets on the upper half sphere with the center at a distance from the tangent plane
	public void generateRandomPoint(GeoTuple4D_F32<?> target, GeoTuple3D_F32<?> obstacle, Boundary boundary, float min_distance_from_center) 
	{

		do { 
			tmp.setTo((float)gauss.nextGaussian()*3f,(float)gauss.nextGaussian()*3f,(float)gauss.nextGaussian()*3f);
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
