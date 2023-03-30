/****************************************************************************
 *
 *   Copyright (c) 2017,2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.mavcontrol.autopilot;

import java.util.concurrent.locks.LockSupport;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

/****************************************************************************
 *
 *   Copyright (c) 2017,2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard3.Offboard3Manager;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTreeNode;
import com.comino.mavmap.map.map3D.impl.octomap.boundingbox.MAVSimpleBoundingBox;

import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector3D_F32;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;


public class SimplePlannerPilot extends AutoPilotBase {

	//	private final static float MIN_DISTANCE_TO_PERSON_M  = 1.5f;


	private final Offboard3Manager offboard = Offboard3Manager.getInstance();
	//	private final MessageBus       bus      = MessageBus.getInstance();


	protected SimplePlannerPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		// Subscribe to detected objects 
		//		bus.subscribe(new ModelSubscriber<msp_msg_nn_object>(msp_msg_nn_object.class, (n) -> {
		//
		//
		//			if(n.tms == 0) {
		//				model.obs.clear();
		//				return;
		//			}
		//
		//			if(!MSP3DUtils.convertCurrentPosition(model, current))
		//				return;
		//
		//			float distance = MSP3DUtils.distance3D(current, n.position);
		//
		//			float angle = MSP3DUtils.angleXY((float)(n.position.x - current.x),(float)(n.position.y - current.y));
		//			yaw_filter.add(angle);
		//
		//			float angle_filtered = (float)yaw_filter.getMean();
		//
		//			//    System.err.println(MSPMathUtils.fromRadSigned(angle_filtered)+" : "+MSPMathUtils.fromRadSigned(angle));
		//
		//
		//			if(distance > MIN_DISTANCE_TO_PERSON_M || control.isSimulation()) {
		//
		//				if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT) && Math.abs(angle_filtered) > 0.1f)
		//					offboard.rotate(angle_filtered, null);	 
		//			}
		//
		//
		//			// TODO: Replace the transfer by Message bus:
		//			model.slam.dm = distance;
		//			model.obs.x = (float)n.position.x;
		//			model.obs.y = (float)n.position.y;
		//			model.obs.z = (float)n.position.z;
		//			model.obs.sx = 0.5f;
		//			model.obs.sy = 0.5f;
		//			model.obs.sz = 2.0f;
		//
		//
		//		}));


		// TODO: Refactor PilotThreading: Not really used, but safety handler and map transfer currently started
		//       with super.start().
		//       Collision check not started as WQ cycle (because of overrun risk)

		start(200);
	}

	public void run() {

		model.sys.t_takeoff_ms = getTimeSinceTakeoff();

	}

	protected void takeoffCompleted() {


	}



	@Override
	protected void start(int cycle_ms) {
		super.start(cycle_ms);

		Thread collisionCheck = new Thread(new EmergencyCollisionCheck());
		collisionCheck.setName("EmergencyColisionCheck");
		collisionCheck.setPriority(Thread.MIN_PRIORITY);
		collisionCheck.start();
	}

	// Note: Octree access always in own Thread due to performance: Do not put into WQ!

	private class EmergencyCollisionCheck implements Runnable {

		private final MAVSimpleBoundingBox boundingBox;
		private final Vector3D_F32         obstacle_position;
		private final Point4D_F32 		   projected;

		public EmergencyCollisionCheck() {
			this.boundingBox        = new MAVSimpleBoundingBox(0.2f,16);
			this.projected          = new Point4D_F32();
			this.obstacle_position  = new Vector3D_F32();
		}

		@Override
		public void run() {

			float distance = 0, min_distance = 0;

			while(true) {
				LockSupport.parkNanos(100_000_000);

				// Emergency collision check

				// get projected position at t+1.0sec
				// TODO: time as a constant; maybe cycle slower, but dt increased
				// TODO: Time should be velocity dependent in order to allow breaking


				offboard.getProjectedPositionAt(1.0f, projected);

				model.obs.x = model.obs.y = model.obs.z = Float.NaN;

				long tms = System.nanoTime(); 
				int count=0;

				boundingBox.set(projected,2.0f);

				OcTreeIterable<MAVOccupancyOcTreeNode> nodes = 
						OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(),boundingBox);

				// searching for the minimum distance
				min_distance = Float.MAX_VALUE;
				for(var node : nodes) {
					if(map.isNodeOccupied(node)) {

						node.getCenter(obstacle_position);
						distance = (float)MSP3DUtils.distance3D(projected, obstacle_position);
						if(distance < min_distance) {
							min_distance = distance;
							model.obs.x =  obstacle_position.x;
							model.obs.y =  obstacle_position.y;
							model.obs.z =  obstacle_position.z;
							model.slam.di = min_distance;
						}
						count++;
					}		
				}

				// TODO: Where does this constant of 0.5m come from? 
				if(min_distance < 0.5) {
					if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP)) {
						logger.writeLocalMsg("[msp] Emergency stop.",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY,1000);
						offboard.abort();
					} else {
						logger.writeLocalMsg("[msp] Collision warning.",MAV_SEVERITY.MAV_SEVERITY_WARNING,1000);
					}
				}

				if(control.isSimulation() && count > 0)
					System.out.println("LeafSearch time (us): "+((System.nanoTime()-tms)/1000L)+" Nodes checked: "+count+" / "+map.getNumberOfNodes());
			}
		}
	}
}
