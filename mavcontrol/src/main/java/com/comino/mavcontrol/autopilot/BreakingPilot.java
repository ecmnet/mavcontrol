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

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_micro_slam;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavmap.struct.Polar3D_F32;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;


public class BreakingPilot extends AutoPilotBase {

	private static final int              CYCLE_MS	        = 50;
	private static final float            ROBOT_RADIUS      = 0.25f;

	private static final float OBSTACLE_MINDISTANCE_0MS  	= 0.5f;
	private static final float OBSTACLE_MINDISTANCE_1MS  	= 1.5f;
	private static final float MIN_BREAKING_SPEED           = 0.2f;
	private static final float BREAKING_ACCELERATION       	= 0.5f;

	private boolean             tooClose      = false;
	private boolean             isStopped     = false;

	private float				max_speed_obstacle = 0;
	private float               relAngle = 0;

	final private Polar3D_F32   obstacle      = new Polar3D_F32();
	final private Polar3D_F32   plannedPath   = new Polar3D_F32();
	final private Polar3D_F32   currentSpeed  = new Polar3D_F32();


	protected BreakingPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		obstacle.value = Float.POSITIVE_INFINITY;

		// calculate speed constraints considering the distance to obstacles
		offboard.registerExternalConstraintsListener((delta_sec, speed, path, ctl) -> {

			plannedPath.set(path);
			currentSpeed.set(speed);

			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP)) {

				if(Float.isInfinite(obstacle.value) || ctl.value < MIN_BREAKING_SPEED ) {
					return false;
				}

				float obs_sec = (float)Math.cos(relAngle) * obstacle.value / speed.value;
                if(obs_sec < 0)
                	return false;



				max_speed_obstacle = ( 0.5f - 0.3f) / ( OBSTACLE_MINDISTANCE_1MS - OBSTACLE_MINDISTANCE_0MS) * obstacle.value;
				if(ctl.value > max_speed_obstacle && tooClose )
					ctl.value = ctl.value - BREAKING_ACCELERATION * delta_sec;


				if(ctl.value < MIN_BREAKING_SPEED || ( tooClose && isStopped)) ctl.value = MIN_BREAKING_SPEED;
				return true;

			}
			return false;
		});

		start();
	}

	public void run() {

		while(isRunning) {

			try { Thread.sleep(CYCLE_MS); } catch(Exception s) { }

			map.processWindow(model.state.l_x, model.state.l_y);
	        map.nearestObstacle(obstacle);

			relAngle = MSPMathUtils.normAngleDiff(obstacle.angle_xy, plannedPath.angle_xy);

//			System.out.println("PATH: "+MSPMathUtils.fromRad(plannedPath.angle_xy)+"°  OBSTACLE:"+MSPMathUtils.fromRad(obstacle.angle_xy)+"° Difference: "+
//					MSPMathUtils.fromRad(relAngle)+"°  rel.Distance: "+obstacle.value);

			if(obstacle.value < OBSTACLE_MINDISTANCE_1MS
					&& !tooClose && Math.abs(Math.cos(relAngle)) > 0.2f
				    && currentSpeed.value > 0.3f ) {
				tooClose = true;
				logger.writeLocalMsg("[msp] Collision warning. Breaking.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			}


			if(obstacle.value < OBSTACLE_MINDISTANCE_0MS && !isStopped && Math.abs(Math.cos(relAngle)) > 0.2f) {
				if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP))
					stop_and_turn(obstacle.angle_xy);
				isStopped = true;
			}


			if(obstacle.value > OBSTACLE_MINDISTANCE_1MS+ROBOT_RADIUS || Math.abs(Math.cos(relAngle)) < 0.2f) {
				tooClose = false;
				isStopped = false;
			}


			if(tooClose)
				publishSLAMData(obstacle);
			else
				publishSLAMData();

			if(mapForget && mapFilter != null)
				map.applyMapFilter(mapFilter);

		}
	}

	protected void takeoffCompleted() {
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP,true);
		super.takeoffCompleted();
	}




	@Override
	public void moveto(float x, float y, float z, float yaw) {
		isStopped = false;
		super.moveto(x, y, z, yaw);
	}

	public void stop_and_turn(float targetAngle) {
		logger.writeLocalMsg("[msp] Emergency breaking",MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
		final Vector4D_F32 target = new Vector4D_F32(Float.NaN,Float.NaN,Float.NaN,targetAngle);
		offboard.registerActionListener( (m,d) -> {
			offboard.finalize();
			logger.writeLocalMsg("[msp] Turned to nearest obstacle.",MAV_SEVERITY.MAV_SEVERITY_INFO);
			offboard.start(OffboardManager.MODE_LOITER);
		});
		offboard.setTarget(target);
		offboard.start(OffboardManager.MODE_SPEED_POSITION);
	}





}
