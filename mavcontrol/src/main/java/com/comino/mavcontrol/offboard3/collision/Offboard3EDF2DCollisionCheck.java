package com.comino.mavcontrol.offboard3.collision;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavmap.map.map3D.impl.octomap.esdf.D2.DynamicESDF2D;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;

public class Offboard3EDF2DCollisionCheck {

	private final MAVOctoMap3D             map;
	private final Offboard3State           state    = new Offboard3State();
	private final DynamicESDF2D            edf;


	public Offboard3EDF2DCollisionCheck(MAVOctoMap3D map) {
		this.map      = map;
		this.edf      = map.getLocalEDF2D();
	}


	public boolean isTargetFeasible(GeoTuple4D_F32<?> pos) {
		return (float) edf.getDistanceAt(pos) > 0.5f;
	}


	public Offboard3Collision check(RapidTrajectoryGenerator xyzPlanner,
			float time_section_start, Offboard3Current current, int planningSectionsIndex)  {

		if(time_section_start > xyzPlanner.getTotalTime())
			return null;

		float time_elapsed = xyzPlanner.getTotalTime(); 

		long tms = System.nanoTime(); int count=0;

		while(time_elapsed > 0.05f) {
			
			xyzPlanner.getState(time_elapsed, state);
			
			float distance = (float) edf.getDistanceAt(state.pos());
			if(distance < 0.25f) {
				System.out.println("EDF2D time (us): "+((System.nanoTime()-tms)/1000L)+" Steps checked: "+(count++));
				return new Offboard3Collision(time_elapsed, xyzPlanner.getTotalTime(), 
						current,state, planningSectionsIndex);
			}
			
			float dt =  map.getResolution() / ( 4.0f * MSP3DUtils.norm3D(state.vel()));
			if(dt > 0.2) dt = 0.2F;
			time_elapsed = time_elapsed - dt ;
			
		}

		return null;

	}

}
