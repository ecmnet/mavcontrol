package com.comino.mavcontrol.offboard4;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3VelTarget;
import com.comino.mavcontrol.offboard4.optimizer.MAVTrajectoryBrentOptimizer2;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class Offboard4EDF2DPlanner {
	
	// Current state
		private final Offboard3Current              current;
		private final GeoTuple4D_F32<?>             local_target;
		private final GeoTuple4D_F32<?>             local_dir;
		private final GeoTuple4D_F32<?>             local_velocity;
		private final MAVTrajectoryBrentOptimizer2  optimizer;

		public Offboard4EDF2DPlanner(IMAVController control, MAVOctoMap3D map) {

			this.current        = new Offboard3Current(control.getCurrentModel()); 
			this.local_target   = new Vector4D_F32();
			this.local_velocity = new Vector4D_F32();
			this.local_dir      = new Vector4D_F32();
			this.optimizer      = new MAVTrajectoryBrentOptimizer2(map.getLocalEDF2D());

		}
		
		public RapidTrajectoryGenerator planDirectPath(GeoTuple4D_F32<?> global_start,GeoTuple4D_F32<?> global_target, float maxVelocity ,float time) {
			
			current.update();
			
			float velocity  = Math.min(maxVelocity, getDirection(global_target, current.pos(), local_dir)/2);
			
			local_velocity.setTo(local_dir.x, local_dir.y,local_dir.z,0);
			local_velocity.timesIP(velocity);
			
			local_target.setTo(local_dir.x, local_dir.y,local_dir.z,0);
			local_target.timesIP(velocity * time);
			local_target.plusIP(current.pos());
			
			RapidTrajectoryGenerator traj = optimizer.optimize(current,local_velocity,local_target, local_dir, time);
			
			return traj;
			
		}
		
		private float getDirection(GeoTuple4D_F32<?> t, GeoTuple4D_F32<?> p, GeoTuple4D_F32<?> dir) {
			float distance;
			dir.x = t.x - p.x;
			dir.y = t.y - p.y;
			dir.z = t.z - p.z;
			dir.w = 0;
			distance = dir.norm();
			dir.divideIP(distance);
			return distance;
		}

}
