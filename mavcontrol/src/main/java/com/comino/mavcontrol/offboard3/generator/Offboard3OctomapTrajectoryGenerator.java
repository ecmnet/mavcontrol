package com.comino.mavcontrol.offboard3.generator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NoSuchElementException;

import com.comino.mavcontrol.offboard3.Offboard3Planner;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosVelTarget;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.MSPStringUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector3D_F32;

public class Offboard3OctomapTrajectoryGenerator {

	private static final int   MAX_NUMBER_CANDIDATES = 100;       // Max number of plan candidates
	private static final float SLOWDOWN_PERCENT      = 10;        // Reduce velocity at avoidancepoint by percentage

	private final List<Offboard3Plan> candidates = new ArrayList<Offboard3Plan>(MAX_NUMBER_CANDIDATES);

	private final Vector3D_F32     tmp   = new Vector3D_F32();

	private int no_valid, no_invalid;

	public Offboard3Plan getAvoidancePlan(MAVOctoMap3D map, Offboard3Planner planner, Offboard3Plan plan, Offboard3Collision col, float distance, float max_velocity ) {

		no_valid = no_invalid = 0; candidates.clear(); int cycles=0;

		float time_to_collide = plan.getTotalTimeUpTo(col.getPlanningSectionIndex()) + col.getExpectedTimeOfCollision();
		float velocity = max_velocity * (100.0f - SLOWDOWN_PERCENT ) / 100.0f;

		long ts = System.nanoTime(); int i =0;
		do {

			Offboard3Plan candidate = new Offboard3Plan();

			// determine avoidance target
			Offboard3AbstractTarget avoidance_target = new Offboard3PosVelTarget(velocity * (1+(float)((Math.random()-0.5))),time_to_collide);
			cycles += generateAvoidanceTargetPoint(map, distance, avoidance_target.pos(), col.getExpectedStateAtCollision().pos());

			// Build new plan containing the avoidance target and the final target of the original plan.
			candidate.add(avoidance_target);
			candidate.add(new Offboard3PosTarget(plan.getLast().pos()));

			if(planner.planPath(candidate, col.getCurrent())!=null) {
				// Collision of the new plan detected 
				no_invalid++;
			}
			else if(candidate.isEmpty()) {
				// No collision but not feasible
				no_invalid++;
				MSPStringUtils.getInstance().err("Fasibility violated.");
			}
			else {
				// No collision and feasible
				candidates.add(candidate); no_valid++;
			}
		} while((System.nanoTime() -ts) < 10000_000 && ++i < MAX_NUMBER_CANDIDATES );


		if(candidates.isEmpty())
			return null;

		try {

			// get candidate with minimum costs
			Offboard3Plan minCost_candidate = candidates.stream()
					.min(Comparator.comparing(Offboard3Plan::getTotalCosts)).orElseThrow(NoSuchElementException::new);

			MSPStringUtils.getInstance().out("Valid candidates: "+no_valid+" Invalid candidates: "+no_invalid+" Time: "+((System.nanoTime() -ts) /1000L)+"us"+" Cycles: "+cycles);

			return minCost_candidate;

		} catch(NoSuchElementException ne) {
			return null;
		}
	}

	// Simple: random points on a sphere around the center of the collision detected, increasing the sphere by 10% after 20 tries.
	public int generateAvoidanceTargetPoint(MAVOctoMap3D map, float distance, GeoTuple4D_F32<?> target, GeoTuple4D_F32<?> center) {
		float _distance = distance; int count = 0; int cycle=0;
		do {
			count = 0;
			do {
				tmp.setTo((float)Math.random()*2f-1f,(float)Math.random()*2f-1f,(float)Math.random()*0.2f-0.1f);
				tmp.normalize();
				tmp.scale(_distance); 
				target.setTo(center.x+tmp.x,center.y+tmp.y,center.z+tmp.z,Float.NaN);
				// TODO: Check relative altitude instead of LOPS.Z must not go below 0.5m rel. altitude.
				if(target.z > -0.5f) target.z = center.z;
			}
			while(map.isOccupied(target,0.8f,14) || ++count < 50);
			_distance = _distance * 1.1f;
		} while(count >= 50 && _distance < 2.0f);
		
      return cycle;
	}

}
