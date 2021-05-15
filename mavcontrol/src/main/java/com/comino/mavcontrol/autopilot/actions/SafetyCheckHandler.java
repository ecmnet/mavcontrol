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
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavutils.workqueue.WorkQueue;

public class SafetyCheckHandler implements Runnable {
	
	private static final int   RC_DEADBAND             = 20;				      // RC Deadband
	private static final int   RC_LAND_CHANNEL		   = 8;                       // RC channel 8 landing
	private static final int   RC_LAND_THRESHOLD       = 2010;		              // RC channel 8 landing threshold
	
	private final String message = "[msp] Emergency landing triggered by RC";

	private IMAVController  control;
	private DataModel       model;
	private MSPLogger       logger;

	private final WorkQueue wq = WorkQueue.getInstance();
	
	private boolean emergencyLanding = false;
	private Runnable action;
	
	public SafetyCheckHandler(IMAVController control) {
		this(control,null);
	}

	public SafetyCheckHandler(IMAVController control, Runnable action) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.logger   = MSPLogger.getInstance();
		this.action   = action;

		System.out.println("Safetycheck handler started");
		wq.addCyclicTask("HP", 50, this);
	}

	@Override
	public void run() {
		
		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			emergencyLanding = false;
			return;
		}

		// Safety: Channel 8 (Mid) triggers landing mode of PX4
		if(Math.abs(model.rc.get(RC_LAND_CHANNEL) - RC_LAND_THRESHOLD) < RC_DEADBAND && !emergencyLanding && !control.isSimulation()) {
			emergencyLanding = true;
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, (cmd, result) -> {
				if(result==MAV_RESULT.MAV_RESULT_ACCEPTED)
				  logger.writeLocalMsg("[msp] Emergency landing triggered.",MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
				else
				  logger.writeLocalMsg("[msp] Emergency landing refused.",MAV_SEVERITY.MAV_SEVERITY_ALERT);
			},0, 2, 0, Float.NaN );
			if(action!=null)
				action.run();
		}
	}
	
	public boolean isEmergencyLanding() {
		return emergencyLanding;
	}

}
