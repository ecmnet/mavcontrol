package com.comino.mavcontrol.offboard3.collision;

import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;

public class Offboard3OctoMapCollisionCheck {

	private final MAVOctoMap3D             map;
	private final Offboard3State           state    = new Offboard3State();


	public Offboard3OctoMapCollisionCheck(MAVOctoMap3D map) {
		this.map      = map;
	}


	public boolean isTargetFeasible(GeoTuple4D_F32<?> pos) {
		return !map.isOccupied(pos);
	}

	public boolean isTargetFeasible(AbstractConvexObject obstacle,GeoTuple4D_F32<?> pos) {
		return !obstacle.isPointInside(new Point3D_F32(pos.x,pos.y,pos.z));
	}


	public Offboard3Collision check(RapidTrajectoryGenerator xyzPlanner,
			float time_section_start, Offboard3Current current, int planningSectionsIndex)  {

		if(time_section_start > xyzPlanner.getTotalTime())
			return null;

		float time_elapsed = 0.05f; 

		long tms = System.nanoTime(); int count=0;

		while(time_elapsed < xyzPlanner.getTotalTime()) {
			xyzPlanner.getState(time_elapsed, state);
			float dt =  map.getResolution() / ( 4.0f * MSP3DUtils.norm3D(state.vel()));
			if(dt > 0.2) dt = 0.2F;
			time_elapsed = time_elapsed + dt ;
			count++;
			if (map.isOccupied(state.pos(),0.8,14)) {
				System.out.println("LeafSearch time (us): "+((System.nanoTime()-tms)/1000L)+" Nodes checked: "+count+" / "+map.getNumberOfNodes());
				return new Offboard3Collision(time_elapsed, xyzPlanner.getTotalTime(), 
						current,state, planningSectionsIndex);
			}
		}

		return null;

	}

}
