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
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.point.Vector4D_F32;

public class TakeOffHandler {

	protected static final float MAX_REL_DELTA_HEIGHT  = 0.10f;
	protected static final int   INITIAL_DELAY_MS      = 5000;

	private final int STATE_IDLE        = 0;
	private final int STATE_INITIATED   = 1;
	private final int STATE_COUNT_DOWN  = 2;
	private final int STATE_TAKEOFF     = 3;
	private final int STATE_LOITER      = 4;
	private final int STATE_OFFBOARD    = 5;
	private final int STATE_FINALIZED   = 6;

	protected IMAVController  control  = null;
	protected int             state    = STATE_IDLE;

	private   int     max_tko_time_ms = 0;
	private   double  delta_height    = 0;

	private final WorkQueue wq = WorkQueue.getInstance();
	private int task;

	private DataModel       model;
	private MSPLogger       logger;
	private OffboardManager offboard;
	private Runnable        completed;

	private ParameterAttributes takeoff_alt_param;
	private ParameterAttributes takeoff_speed_param;

	protected final Vector4D_F32  takeoff = new Vector4D_F32();
	protected long       tms_takeoff_plan = 0;
	
	public TakeOffHandler(IMAVController control, OffboardManager offboard) {
		this(control,offboard,null);
	}

	public TakeOffHandler(IMAVController control, OffboardManager offboard, Runnable completedAction) {
		this.control   = control;
		this.offboard  = offboard;
		this.model     = control.getCurrentModel();
		this.logger    = MSPLogger.getInstance();
		this. completed = completedAction;
	}

	public void initiateTakeoff(int count_down_secs) {

		if(state!=STATE_IDLE) {
			logger.writeLocalMsg("[msp] Takeoff already initiated. Aborting.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			return;
		}

		takeoff.set(Float.NaN,Float.NaN,Float.NaN,Float.NaN);

		final PX4Parameters params = PX4Parameters.getInstance();
		takeoff_alt_param   = params.getParam("MIS_TAKEOFF_ALT");
		takeoff_speed_param = params.getParam("MPC_TKO_SPEED");
		
		max_tko_time_ms = (int)((takeoff_alt_param.value * 2000 / takeoff_speed_param.value )) + count_down_secs *1000 + INITIAL_DELAY_MS;

		control.writeLogMessage(new LogMessage("[msp] Takeoff procedure initiated.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
		control.writeLogMessage(new LogMessage("[msp] Est. takeoff time: "+(max_tko_time_ms/1000)+" sec.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));

		state = STATE_INITIATED;
		task  = wq.addCyclicTask("LP", 100, new TakeOffStateMachine(count_down_secs));
	}

	public void abort() {
		switch(state) {
		case STATE_IDLE:
			takeoff.set(Float.NaN,Float.NaN,Float.NaN,Float.NaN);
			return;
		case STATE_INITIATED:
			state = STATE_IDLE;
			break;
		case STATE_COUNT_DOWN:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,0 );
			logger.writeLocalMsg("[msp] CountDown aborted.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			state = STATE_IDLE;
			break;
		case STATE_TAKEOFF:
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0, 0 );	
			state = STATE_IDLE;
			break;
		}
	}

	public Vector4D_F32 getTakeoffPosition() {
		return takeoff;
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
				if(!initialChecks())
					state = STATE_IDLE;
				else {
					tms_takeoff_plan = System.currentTimeMillis() + count_down_ms;
					logger.writeLocalMsg("[msp] CountDown initiated.",MAV_SEVERITY.MAV_SEVERITY_INFO);
					state = STATE_COUNT_DOWN;
				}
				break;
			case STATE_COUNT_DOWN:
				
				// Check LIDAR availability
				if(!model.sys.isSensorAvailable(Status.MSP_LIDAR_AVAILABILITY)) {
					control.writeLogMessage(new LogMessage("[msp] CountDown aborted. LIDAR not available",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
				}
				
				// Check EKF reports absolut position
				if((model.est.flags & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS) != ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS ) {
					control.writeLogMessage(new LogMessage("[msp] CountDown aborted. EKF reports no abs.Pos.",
							MAV_SEVERITY.MAV_SEVERITY_CRITICAL));
					state = STATE_IDLE;
				}
				
				if(System.currentTimeMillis() > tms_takeoff_plan) {
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_TAKEOFF, (cmd, result) -> {
						if(result == MAV_RESULT.MAV_RESULT_ACCEPTED) {
							model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);	
							tms_takeoff_act = System.currentTimeMillis();
							state = STATE_TAKEOFF;
						} else {
							state = STATE_IDLE;
						}
					},-1, 0, 0, Float.NaN, Float.NaN, Float.NaN,Float.NaN);

				}
				
				break;
			case STATE_TAKEOFF:
				delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;
				if(delta_height < MAX_REL_DELTA_HEIGHT) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff altitude reached.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					state = STATE_LOITER;
				}

				if((System.currentTimeMillis() - tms_takeoff_act) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff (1) did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
				}

				break;
			case STATE_LOITER:
				if((System.currentTimeMillis() - tms_takeoff_act) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff (2) did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
				}

				if(model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER)) {
					offboard.start();
					control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
						if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
							offboard.stop();
							control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
							control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
									MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
									MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );
							state = STATE_IDLE;
						} else {
							state = STATE_OFFBOARD;
						}
					}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
							MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
				}

				break; 
			case STATE_OFFBOARD:

				if((System.currentTimeMillis() - tms_takeoff_act) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff (3) did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					state = STATE_IDLE;
				}

				if(model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
					state = STATE_FINALIZED;
				}

				break;
			case STATE_FINALIZED:

				control.writeLogMessage(new LogMessage("[msp] Setting takeoff position.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				takeoff.set(model.state.l_x,model.state.l_y,model.state.l_z, model.attitude.y);

				if(completed!=null && model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.TAKEOFF_PROCEDURE))
					completed.run();
				
				state = STATE_IDLE;

				break;
			}

		}

		private boolean initialChecks() {

			if(!model.sys.isStatus(Status.MSP_ARMED)) {
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.TAKEOFF, false);
				logger.writeLocalMsg("[msp] CountDown not initiated. Not armed.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
				return false;
			}

			if(!model.sys.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY)) {
				if(!control.isSimulation()) {
					return false;
				}
			}
			return true;
		}

	}



}
