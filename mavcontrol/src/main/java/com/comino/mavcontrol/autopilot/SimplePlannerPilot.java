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

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_rc_channels_override;

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
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.point.Vector4D_F32;


public class SimplePlannerPilot extends AutoPilotBase {


	private static final int   RC_LAND_CHANNEL						= 8;                      // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD            		= 1600;		              // RC channel 8 landing threshold

	private boolean is_landing = false;
	private boolean valid_target = false;
	private final Vector4D_F32 target  = new Vector4D_F32();
	private final Vector4D_F32 current  = new Vector4D_F32();

	private long sp_tms;
	
	private int reset_counter = 0;
	
	private final msg_rc_channels_override  fcum_thrust = new msg_rc_channels_override(1,1);


	protected SimplePlannerPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
			is_landing = false;
		});

		start(100);
	}

	public void run() {
		
		if(reset_counter != model.est.reset_counter) {
            if(reset_counter>0)
              control.writeLogMessage(new LogMessage("[msp] EKF2 reset detected (AP).", MAV_SEVERITY.MAV_SEVERITY_DEBUG)); 
            reset_counter =  model.est.reset_counter;
		}

		MSP3DUtils.convertCurrentPosition(model, current);
		
		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.FCUM)) {

			if(model.sys.isStatus(Status.MSP_ARMED)) {
				fcum_thrust.chan4_raw = 1500;
				fcum_thrust.chan1_raw = 1500;
				fcum_thrust.chan2_raw = 1500;
				fcum_thrust.chan3_raw = 1500;
			} else {
				fcum_thrust.chan4_raw = 1500;
				fcum_thrust.chan1_raw = 900;
				fcum_thrust.chan2_raw = 1500;
				fcum_thrust.chan3_raw = 1500;
			}
			control.sendMAVLinkMessage(fcum_thrust);
			return;
		}



		// Safety: Channel 8 triggers landing mode of PX4
		if(model.rc.get(RC_LAND_CHANNEL) > RC_LAND_THRESHOLD && !is_landing) {
			logger.writeLocalMsg("[msp] Landing commanded by RC",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			sequencer.abort();
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
					logger.writeLocalMsg("[msp] Auto-Land not accepted",MAV_SEVERITY.MAV_SEVERITY_INFO);
				else {
					logger.writeLocalMsg("[msp] Landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
					is_landing = true;
				}
			}, 0, 0,  model.state.h  );
		}

		model.sys.t_takeoff_ms = getTimeSinceTakeoff();

	}

	protected void takeoffCompleted() {

	}


}
