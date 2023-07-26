package com.comino.mavcontrol.offboard4;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.utils.MAVTrajectoryBrentOptimizer;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;

public class Offboard4EDF2DPlanner {
	
	// Current state
		private final Offboard3Current            current;
		private final MAVTrajectoryBrentOptimizer optimizer;
		private final DataModel                   model;

		public Offboard4EDF2DPlanner(IMAVController control, MAVOctoMap3D map) {

			this.model      = control.getCurrentModel();
			this.current    = new Offboard3Current(model);  
			this.optimizer  = new MAVTrajectoryBrentOptimizer(current,map.getLocalEDF2D());

		}
		
		public RapidTrajectoryGenerator planDirectPath(GeoTuple4D_F32<?> pos_target, double time) {
			
			Point3D_F32 p = new Point3D_F32();
			pos_target.z = model.state.l_z;
			pos_target.w = 0;
			
			RapidTrajectoryGenerator traj = optimizer.optimize(pos_target, pos_target.copy(), time);
			traj.getPosition(time, p);
			model.obs.x = p.x;
			model.obs.y = p.y;
			model.obs.z = p.z;
			return traj;
			
		}

}
