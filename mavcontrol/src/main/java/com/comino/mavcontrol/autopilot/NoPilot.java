package com.comino.mavcontrol.autopilot;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

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
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.status.StatusManager;


public class NoPilot extends AutoPilotBase {


	private static final int   RC_LAND_CHANNEL						= 8;                      // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD            		= 1600;		              // RC channel 8 landing threshold

	private boolean is_landing = false;


	protected NoPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_STATUS, Status.MSP_ARMED, StatusManager.EDGE_RISING, (n) -> {
			is_landing = false;
		});

		start();
	}

	public void run() {

		while(isRunning) {

			// Safety: Channel 8 triggers landing mode of PX4
			if(model.rc.get(RC_LAND_CHANNEL) > RC_LAND_THRESHOLD && !is_landing) {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, (cmd, result) -> {
					if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
						logger.writeLocalMsg("[msp] Auto-Land not accepted",MAV_SEVERITY.MAV_SEVERITY_INFO);
					else {
						logger.writeLocalMsg("[msp] Landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
						is_landing = true;
					}
				}, 0, 2, 0.05f );
			}

			try { Thread.sleep(50); } catch(Exception s) { }
			publishSLAMData();
		}

	}

	protected void takeoffCompleted() {

	}




	@Override
	public void moveto(float x, float y, float z, float yaw) {

	}

}
