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

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavutils.workqueue.WorkQueue;

public class TakeOffHandler {

	protected static final float MAX_REL_DELTA         = 0.10f;
	protected static final int   INITIAL_DELAY_MS      = 7000;

	private final int STATE_IDLE        = 0;
	private final int STATE_INITIATED   = 1;
	private final int STATE_COUNT_DOWN  = 2;
	private final int STATE_TAKEOFF     = 3;
	private final int STATE_LOITER      = 4;
	private final int STATE_FINALIZED   = 5;

	protected IMAVController  control  = null;
	protected int             state    = STATE_IDLE;

	private   int     max_tko_time_ms = 0;
	private   double  delta_height    = 0;

	private float visx, visy, visz;

	private final WorkQueue wq = WorkQueue.getInstance();
	private int task;

	private DataModel       model;
	private MSPLogger       logger;
	private Runnable        completed;
	private Runnable        aborted;

	private ParameterAttributes takeoff_alt_param;
	private ParameterAttributes takeoff_speed_param;

	protected long       tms_takeoff_plan = 0;

	public TakeOffHandler(IMAVController control) {
		this(control,null,null);
	}

	public TakeOffHandler(IMAVController control, Runnable completedAction, Runnable abortedAction) {
		this.control   = control;
		this.model     = control.getCurrentModel();
		this.logger    = MSPLogger.getInstance();
		this.completed = completedAction;
		this.aborted   = abortedAction;
	}
	
	public void setTakeoffAltitude(float altitude_m) {
		final PX4Parameters params = PX4Parameters.getInstance();
		params.sendParameter("MIS_TAKEOFF_ALT", altitude_m);
	}

	public void initiateTakeoff(int count_down_secs) {

		if(state!=STATE_IDLE) {
			logger.writeLocalMsg("[msp] Takeoff already initiated. Aborting.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			return;
		}

		final PX4Parameters params = PX4Parameters.getInstance();
		takeoff_alt_param   = params.getParam("MIS_TAKEOFF_ALT");
		takeoff_speed_param = params.getParam("MPC_TKO_SPEED");

		if(takeoff_alt_param == null || takeoff_speed_param == null) {
			logger.writeLocalMsg("[msp] CountDown aborted. Parameters not loaded",MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
			return;
		}

		max_tko_time_ms = (int)((takeoff_alt_param.value * 4000 / takeoff_speed_param.value )) + count_down_secs *1000 + INITIAL_DELAY_MS;

		logger.writeLocalMsg("[msp] Count down initiated.", MAV_SEVERITY.MAV_SEVERITY_INFO);
		logger.writeLocalMsg("[msp] Altitude of "+takeoff_alt_param.getValueFormatted()+" m reached in "+(max_tko_time_ms/1000)+" s.", MAV_SEVERITY.MAV_SEVERITY_INFO);

		state = STATE_INITIATED;
		task  = wq.addCyclicTask("LP", 100, new TakeOffStateMachine(count_down_secs));
	}

	public void abort(String reason) {
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
		switch(state) {
		case STATE_IDLE:
			return;
		case STATE_INITIATED:
			state = STATE_IDLE;
			break;
		case STATE_COUNT_DOWN:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM, 0 );
			logger.writeLocalMsg("[msp] CountDown stopped: "+reason,MAV_SEVERITY.MAV_SEVERITY_WARNING);
			state = STATE_IDLE;
			break;
		case STATE_TAKEOFF:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			logger.writeLocalMsg("[msp] Takeoff interrupted: "+reason,MAV_SEVERITY.MAV_SEVERITY_WARNING);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0, Float.NaN );	
			state = STATE_IDLE;
			break;
		}
	}

	public long getPlannedTakeoffTime() {
		return tms_takeoff_plan;
	}

	private class TakeOffStateMachine implements Runnable {

		private long    count_down_ms      = 0;
		private long    tms_takeoff_act    = 0;

		public TakeOffStateMachine(int count_down_secs) {
			this.count_down_ms = count_down_secs * 1000;
		}


		@Override
		public void run() {

			switch(state) {
			case STATE_IDLE:
				tms_takeoff_plan = 0; 
				wq.removeTask("LP", task);
				break;
			case STATE_INITIATED:
				if(!initialChecks()) {
					state = STATE_IDLE;
					if(aborted!=null) aborted.run();
				}
				else {
					
					tms_takeoff_plan = System.currentTimeMillis() + count_down_ms;
					visx = model.vision.x;
					visy = model.vision.y;
					visz = model.vision.z;
					state = STATE_COUNT_DOWN;
				}
				break;
			case STATE_COUNT_DOWN:

				if(!control.isSimulation()) {

					// Check LIDAR availability
					if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY)) {
						logger.writeLocalMsg("[msp] CountDown aborted. LIDAR not available",
								MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
						state = STATE_IDLE;
						if(aborted!=null) aborted.run();
					}

					// Check EKF reports relative position
					if((model.est.flags & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ) != ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ  ||
							(model.est.flags & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ACCEL_ERROR)==ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ACCEL_ERROR) {
						logger.writeLocalMsg("[msp] CountDown aborted. EKF reports fault.",
								MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
						state = STATE_IDLE;
						if(aborted!=null) aborted.run();
					}

					// Check stability of CV
					if(Math.abs(model.vision.x - visx) > MAX_REL_DELTA || Math.abs(model.vision.y - visy) > MAX_REL_DELTA 
							|| Math.abs(model.vision.z - visz) > MAX_REL_DELTA) {
						logger.writeLocalMsg("[msp] CountDown aborted. Odometry not stable.",
								MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
						state = STATE_IDLE;
						if(aborted!=null) aborted.run();
					}
				} 

				if(System.currentTimeMillis() > tms_takeoff_plan) {
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_TAKEOFF, (cmd, result) -> {
						if(result == MAV_RESULT.MAV_RESULT_ACCEPTED) {
							model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);	
							tms_takeoff_act = System.currentTimeMillis();
							state = STATE_TAKEOFF;
						} else {
							state = STATE_IDLE;
							if(aborted!=null) aborted.run();
						}
						// TODO: WARNING: This leads in SITL to takeoff to LPOS 0,0
						//       Might be in cases only, where GPOS is not valid
					},0, 0, 0, Float.NaN, Float.NaN, Float.NaN,Float.NaN);

				}

				break;
			case STATE_TAKEOFF:
				delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;
				if(delta_height < MAX_REL_DELTA) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff altitude reached.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					state = STATE_LOITER;
				}

				if((System.currentTimeMillis() - tms_takeoff_act) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff (1) did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
					if(aborted!=null) aborted.run();
				}

				break;
			case STATE_LOITER:

				if(model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER)) {
					control.writeLogMessage(new LogMessage("[msp] Switched to PX4 HOLD mode.", MAV_SEVERITY.MAV_SEVERITY_INFO));
					state = STATE_FINALIZED;
					return;
				}


				if((System.currentTimeMillis() - tms_takeoff_act) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff (2) did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
					if(aborted!=null) aborted.run();
					return;
				}


				break; 


			case STATE_FINALIZED:

				control.writeLogMessage(new LogMessage("[msp] Setting takeoff position.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

			//	if(completed!=null && model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE))
				if(completed!=null)
					completed.run();

				state = STATE_IDLE;

				break;
			}

		}

		private boolean initialChecks() {

			if(!model.sys.isStatus(Status.MSP_READY_FOR_FLIGHT)) {
				if(model.sys.isSensorAvailable(Status.MSP_MSP_AVAILABILITY)) {
					logger.writeLocalMsg("[msp] Takeoff aborted. Not ready for flight.",MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
					return false;
				}
			}

			if(!model.sys.isStatus(Status.MSP_ARMED)) {
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
				logger.writeLocalMsg("[msp] CountDown not initiated. Not armed.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
				return false;
			}

			return true;
		}

	}



}
