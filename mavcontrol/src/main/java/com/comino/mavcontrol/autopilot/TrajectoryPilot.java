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
import org.mavlink.messages.MAV_COMPONENT;
import org.mavlink.messages.MAV_MODE_FLAG;

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

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_trajectory_representation_waypoints;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;


/*
 * Autopilot that uses the trajectory interface instead of offboard
 */

public class TrajectoryPilot extends AutoPilotBase {

	private static final int              CYCLE_MS	        = 100;


	private final Vector4D_F32  target_pos  = new Vector4D_F32();
	private final Vector4D_F32  current_pos = new Vector4D_F32();

	private boolean             enabled     = false;

	private final msg_trajectory_representation_waypoints wp = new msg_trajectory_representation_waypoints(1,2);

	protected TrajectoryPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		// Register
		registerRTL();

		// Trajectory control requires COM_OBS_AVOID to be set to 1
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED))		
				params.sendParameter("COM_OBS_AVOID", 1.0f);
		});

		start(50);
	}

	public void run() {

			MSP3DUtils.convertCurrentState(model, current_pos);

			try { Thread.sleep(CYCLE_MS); } catch(Exception s) { }
			
			model.sys.t_takeoff_ms = getTimeSinceTakeoff();

			if(enabled) {
				sendTrajectory(target_pos);	
				transferObstacleToModel(null);
			} 
	}


	@Override
	public int getAutopilotStatus() {
		return 0;
	}


	@Override
	public void moveto(float x, float y, float z, float yaw) {
		target_pos.setTo(x,y,z,yaw);
		if(Float.isNaN(yaw)) {
			if(MSP3DUtils.distance2D(target_pos, current_pos) > 0.3)
				target_pos.w = MSP3DUtils.angleXY(target_pos, current_pos);
			else
				target_pos.w = current_pos.w;

		}

		logger.writeLocalMsg("[msp] New setpoint "+String.format("(%.1f,%.1f)",x,y),MAV_SEVERITY.MAV_SEVERITY_DEBUG);
	}


	protected void registerLanding() {
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_LAND, StatusManager.EDGE_RISING, (n) -> {
			System.out.println("Landing triggered");
			enabled = false;
		});

	}

	protected void registerRTL() {
		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_RTL, StatusManager.EDGE_RISING, (n) -> {
			System.out.println("RTL triggered");
			enabled = false;
		});
	}

	protected void registerTakeoff() {

		control.getStatusManager().addListener(StatusManager.TYPE_PX4_NAVSTATE, Status.NAVIGATION_STATE_AUTO_TAKEOFF, StatusManager.EDGE_RISING, (n) -> {

			final ParameterAttributes  takeoff_alt_param   = params.getParam("MIS_TAKEOFF_ALT");
			final ParameterAttributes  takeoff_speed_param = params.getParam("MPC_TKO_SPEED");

			// calculate maximum takeoff time
			final int max_tko_time_ms = (int)(takeoff_alt_param.value / takeoff_speed_param.value ) * 1000 + 15000;

			control.writeLogMessage(new LogMessage("[msp] Takeoff proecdure initiated.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

			long takeoff_start_tms = System.currentTimeMillis();
			double delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;


			// Phase 1: Wait for height is in range
			while(delta_height > MAX_REL_DELTA_HEIGHT) {
				try { Thread.sleep(50); } catch(Exception e) { }
				delta_height = Math.abs(takeoff_alt_param.value - model.hud.ar) / takeoff_alt_param.value;
				if((System.currentTimeMillis() - takeoff_start_tms) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					return;
				}
			}

			// Phase 2: Wait for LOITER NavState to indicate that takeoff has completed
			while(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER)) {
				try { Thread.sleep(50); } catch(Exception e) { }
				if((System.currentTimeMillis() - takeoff_start_tms) > max_tko_time_ms) {
					control.writeLogMessage(new LogMessage("[msp] Takeoff did not complete within "+(max_tko_time_ms/1000)+" secs",
							MAV_SEVERITY.MAV_SEVERITY_WARNING));
					return;
				}
			}


			// Phase 3: Switch to HOLD
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE,
					MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER );

			control.writeLogMessage(new LogMessage("[msp] Setting takeoff position.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			this.takeoff.setTo(model.state.l_x,model.state.l_y,model.state.l_z,0);


			try { Thread.sleep(200); } catch(Exception e) { }

			MSP3DUtils.convertCurrentState(model, target_pos);
			enabled = true;

		});

	}

	private void sendTrajectory(Vector4D_F32 target) {

		wp.pos_x[0] = target.x;
		wp.pos_y[0] = target.y;
		wp.pos_z[0] = target.z;

		if(Float.isNaN(target.w)) 
			wp.pos_yaw[0] = current_pos.w;	
		else
			wp.pos_yaw[0] = target_pos.w;

		wp.command[0] = 0;
		wp.valid_points = 1;
		wp.time_usec = DataModel.getSynchronizedPX4Time_us();

		control.sendMAVLinkMessage(wp);


		model.slam.pd = wp.pos_yaw[0];
		model.slam.di = MSP3DUtils.distance2D(target, current_pos);
		if(model.slam.di > 0.2) {
			model.slam.px = target.x;
			model.slam.py = target.y;
			model.slam.pz = target.z;
		} else
			model.slam.di = 0;

		model.slam.pv = model.hud.s;

	}

	private void sendTrajectory() {

		wp.pos_x[0] = model.way.posx[0];
		wp.pos_y[0] = model.way.posy[0];
		wp.pos_z[0] = model.way.posz[0];

		wp.pos_yaw[0] = model.way.pyaw[0];

		wp.command[0] = model.way.cmd[0];
		wp.valid_points = 1;
		wp.time_usec = model.way.tms * 1000;

		control.sendMAVLinkMessage(wp);

	}

}
