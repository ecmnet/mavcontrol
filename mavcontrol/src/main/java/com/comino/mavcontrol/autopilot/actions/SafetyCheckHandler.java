/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
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


package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcontrol.sequencer.Sequencer;
import com.comino.mavutils.workqueue.WorkQueue;

public class SafetyCheckHandler implements Runnable {

	private static final int   RC_LAND_CHANNEL		   = 8;                       // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD       = 1600;		              // RC channel 8 landing threshold

	private final IMAVController  control;
	private final DataModel       model;
	private final MSPLogger       logger;
	private final Sequencer       sequencer;

	private final WorkQueue wq = WorkQueue.getInstance();

	private boolean emergency        = false;

	public SafetyCheckHandler(IMAVController control, Sequencer sequencer) {
		this.control   = control;
		this.sequencer = sequencer;
		this.model     = control.getCurrentModel();
		this.logger    = MSPLogger.getInstance();

		System.out.println("Safetycheck handler started");
		
		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			if(n.isStatus(Status.MSP_ARMED) && n.isStatus(Status.MSP_LANDED))
				emergency = false; 
		});
	}
	
	public void start() {
		if(wq.addCyclicTask("HP", 100, this) != 0) {
			System.out.println("SafetyCheckHandler started..");
		}
	}

	@Override
	public void run() {

		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			emergency = false;
			return;
		}

		// Safety: Channel 8 triggers landing mode / precision landing of PX4
		if(model.rc.get(RC_LAND_CHANNEL) > RC_LAND_THRESHOLD && !control.isSimulation()) {
			emergency = true;
			sequencer.abort();
			if(!model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, ( cmd,result) -> {
					if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
						logger.writeLocalMsg("[msp] PX4 landing rejected ("+result+")",MAV_SEVERITY.MAV_SEVERITY_WARNING);
					else
						logger.writeLocalMsg("[msp] PX4 landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
				}, 0, 0, 0, Float.NaN );
			} else {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
					if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
						logger.writeLocalMsg("[mgc] PX4 Prec.Landing rejected ("+result+")",MAV_SEVERITY.MAV_SEVERITY_WARNING);
					else
						logger.writeLocalMsg("[mgc] PX4 Prec.Landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
				},	MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND );
			}
		}
		
		// Safety:  ...
		
		// publish emergency case via MessageBus if required
	}

	public boolean isEmergencyLanding() {
		return emergency;
	}

}
