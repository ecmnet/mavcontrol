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
import com.comino.mavcom.messaging.MessageBus;
import com.comino.mavcom.messaging.ModelSubscriber;
import com.comino.mavcom.messaging.msgs.msp_msg_nn_object;
import com.comino.mavcom.model.segment.Battery;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcom.utils.SimpleLowPassFilter;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Point4D_F64;
import georegression.struct.point.Vector4D_F32;
import georegression.struct.point.Vector4D_F64;


public class SimplePlannerPilot extends AutoPilotBase {
	
	private final static float MIN_DISTANCE_TO_PERSON_M  = 1.5f;

	private final Point4D_F64 current      = new Point4D_F64();

	private final Offboard2Manager offboard = Offboard2Manager.getInstance();
	private final MessageBus       bus      = MessageBus.getInstance();
	
	private final SimpleLowPassFilter yaw_filter = new SimpleLowPassFilter(0.5f);


	protected SimplePlannerPilot(IMAVController control, MSPConfig config) {
		super(control,config);
		
		// Subscribe to detected objects 
		bus.subscribe(new ModelSubscriber<msp_msg_nn_object>(msp_msg_nn_object.class, (n) -> {
			
			if(n.tms == 0) {
				model.slam.dm = Float.NaN;
				model.slam.ox = Float.NaN;
				model.slam.oy = Float.NaN;
				model.slam.oz = Float.NaN;
				model.slam.pv = Float.NaN;
				return;
			}
			
			if(!MSP3DUtils.convertCurrentPosition(model, current))
				return;
			
			float distance = MSP3DUtils.distance3D(current, n.position);
			yaw_filter.add(MSPMathUtils.normAngle(MSP3DUtils.angleXY((float)(n.position.x - current.x),(float)(n.position.y - current.y))));
			float angle = (float)yaw_filter.getMean();
			
			if(distance > MIN_DISTANCE_TO_PERSON_M || control.isSimulation()) {
			 
			  if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.INTERACTIVE) && current.norm() > 0.1f)
				  offboard.rotate(angle, null);	 
			}
			
			// TODO: Replace the transfer by Message bus:
			model.slam.dm = distance;
			model.slam.ox = (float)n.position.x;
			model.slam.oy = (float)n.position.y;
			model.slam.oz = (float)n.position.z;
			
			// Test only 
			model.slam.pd = angle;
			model.slam.pv = 0.5f;
			
			
		}));


		start(200);
	}

	public void run() {
		

		MSP3DUtils.convertCurrentPosition(model, current);

		model.sys.t_takeoff_ms = getTimeSinceTakeoff();

	}

	protected void takeoffCompleted() {

	}


}
