package com.comino.mavcontrol.autopilot;

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

import org.mavlink.messages.MAV_DISTANCE_SENSOR;
import org.mavlink.messages.lquac.msg_obstacle_distance;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavmap.map.map2D.filter.impl.DenoiseMapFilter;
import com.comino.mavmap.trajectory.collprev.CollisionPreventionConverter;

import georegression.struct.point.Vector3D_F32;

public class CollisonPreventionPilot extends AutoPilotBase {

	private static final int              CYCLE_MS	= 50;

	private CollisionPreventionConverter  collprev  = null;;

	protected CollisonPreventionPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		if(mapForget)
			registerMapFilter(new DenoiseMapFilter(800,800));

		this.collprev = new CollisionPreventionConverter(map,CERTAINITY_THRESHOLD);

		start();
	}


	public void run() {

		float[]               distances;
		Vector3D_F32          current    = new Vector3D_F32();

		msg_obstacle_distance msg       = new msg_obstacle_distance(1,2);
		msg.increment = 5;
		msg.max_distance = (int)(WINDOWSIZE * 100);
		msg.min_distance = 1;
		msg.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;

		System.out.println("CollisionPreventionPilot started");

		while(isRunning) {

			try { Thread.sleep(CYCLE_MS); } catch(Exception s) { }

			current.set(model.state.l_x, model.state.l_y,model.state.l_z);
			distances = collprev.update(current);

			for(int i=0;i<distances.length;i++) {
				if(distances[i]<10000)
					msg.distances[i] = (int)distances[i] / 10;
				else
					msg.distances[i] = msg.max_distance + 1;
			}

			msg.time_usec = DataModel.getSynchronizedPX4Time_us();

			control.sendMAVLinkMessage(msg);



		}

	}

}
