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

package com.comino.mavcontrol.autopilot;

import org.mavlink.messages.MAV_CMD;
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
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavutils.MSPMathUtils;
import com.ochafik.lang.jnaerator.runtime.This;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;



public class BreakingPilot extends AutoPilotBase {

	private static float SMOOTH_TARGET_FILTER               = 0.1f;

	private static final int              CYCLE_MS	        = 50;

	private static final float            ROBOT_RADIUS      = 0.25f;

	private static final float OBSTACLE_MINDISTANCE_0MS  	= ROBOT_RADIUS + 0.25f;
	private static final float OBSTACLE_MINDISTANCE_1MS  	= ROBOT_RADIUS + 0.75f;
	private static final float MIN_BREAKING_SPEED           = 0.2f;
	private static final float MIN_REL_ANGLE                = MSPMathUtils.toRad(90);

	private boolean             tooClose      = false;

	private float               relAngle = 0;

	final private Polar3D_F32   obstacle      = new Polar3D_F32();
	final private Polar3D_F32   plannedPath   = new Polar3D_F32();
	final private Polar3D_F32   currentSpeed  = new Polar3D_F32();

	private final Point3D_F64   smooth_target = new Point3D_F64(Double.NaN, Double.NaN, Double.NaN);
	private final Vector4D_F32  target        = new Vector4D_F32();
	private final Vector4D_F32  current       = new Vector4D_F32();
	private boolean             smooth_target_initialized = false;

	private float               obs_acc       = 0;


	protected BreakingPilot(IMAVController control, MSPConfig config) {
		super(control,config);

		this.obstacle.value = Float.POSITIVE_INFINITY;
		
		control.getStatusManager().addListener(Status.MSP_PARAMS_LOADED, (n) -> {
			if(n.isStatus(Status.MSP_PARAMS_LOADED))		
				params.sendParameter("COM_OBS_AVOID", 0.0f);
		});

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_AUTOPILOT, MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT, StatusManager.EDGE_BOTH, (n) -> {
			if(n.isAutopilotMode(MSP_AUTOCONTROL_MODE.FOLLOW_OBJECT)) {
				smooth_target_initialized = false;
				smooth_target.set(model.state.l_x, model.state.l_y, model.state.l_z);
				offboard.setTarget((float)smooth_target.x, (float)smooth_target.y, (float)smooth_target.z, 0);
				logger.writeLocalMsg("[msp] Follow object mode enabled.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
				offboard.start(OffboardManager.MODE_SPEED_POSITION);
			} else {
				smooth_target_initialized = false;
				logger.writeLocalMsg("[msp] Follow object mode disabled.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			}
		});

		// calculate speed constraints considering the distance to obstacles
		offboard.registerContraintControl((delta_sec, speed, path, ctl) -> {

			plannedPath.set(path); currentSpeed.set(speed);

			relAngle = Math.abs(MSPMathUtils.normAngle2(Math.abs(obstacle.angle_xy-plannedPath.angle_xy)));

			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP)) {

				if(Float.isInfinite(obstacle.value) || ctl.value < MIN_BREAKING_SPEED ) {
					return false;
				}

				//				float obs_sec = relAngle * obstacle.value / speed.value;
				//				if(obs_sec < 0)
				//					return false;

				obs_acc = currentSpeed.value / ( obstacle.value - OBSTACLE_MINDISTANCE_0MS) * (float)Math.cos(relAngle);

				if(obstacle.value <  OBSTACLE_MINDISTANCE_1MS && tooClose && obs_acc > 0 ) {
					ctl.value = ctl.value - obs_acc * delta_sec;
					if(ctl.value < MIN_BREAKING_SPEED) ctl.value = MIN_BREAKING_SPEED;
					//System.out.println("Breaking: "+ctl.value+" Delta: "+delta_sec + " ETA.Obs: " + obs_sec);
				}

				return true;

			}
			return false;
		});

		start();
	}

	public void run() {

		System.out.println(this.getClass().getSimpleName()+" started");

		while(isRunning) {


			try { Thread.sleep(CYCLE_MS); } catch(Exception s) { }

			if(!super.safetyChecks()) {

			}

			// Apply filter first
			if(mapForget && mapFilter != null)
				map.applyMapFilter(mapFilter);

			// Publish SLAM data
			if(tooClose)
				publishSLAMData(obstacle);
			else
				publishSLAMData();

            obstacle.clear();
			map.processWindow(model.state.l_x, model.state.l_y);
			map.nearestObstacle(obstacle);

			// Control only in affected offboard modes
			if(offboard.getMode() != OffboardManager.MODE_SPEED_POSITION)
				continue;


		//	relAngle = Math.abs(MSPMathUtils.normAngle2(Math.abs(obstacle.angle_xy-plannedPath.angle_xy)));

			if(obstacle.value < OBSTACLE_MINDISTANCE_1MS && !tooClose && relAngle < MIN_REL_ANGLE && currentSpeed.value > 0.1 ) {
				tooClose = true;
//				System.out.println("W"+MSPMathUtils.fromRad(MIN_REL_ANGLE)+" -> "+MSPMathUtils.fromRad(relAngle) +" :"+ MSPMathUtils.fromRad(obstacle.angle_xy)+" :"+MSPMathUtils.fromRad(plannedPath.angle_xy));
				logger.writeLocalMsg("[msp] Collision warning. Breaking.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			}


			if(obstacle.value < OBSTACLE_MINDISTANCE_0MS && relAngle < MIN_REL_ANGLE && currentSpeed.value > 0.1 ) {
//				System.out.println("S"+MSPMathUtils.fromRad(MIN_REL_ANGLE)+" -> "+MSPMathUtils.fromRad(relAngle) +" :"+ MSPMathUtils.fromRad(obstacle.angle_xy)+" :"+MSPMathUtils.fromRad(plannedPath.angle_xy));
//				System.out.println("S"+MIN_REL_ANGLE+" -> "+relAngle);
				if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.OBSTACLE_STOP )) {
					emergency_stop_and_turn(obstacle.angle_xy);
				}
				tooClose = true;
			}

			if(tooClose && obstacle.value > OBSTACLE_MINDISTANCE_1MS+ROBOT_RADIUS) {
				logger.writeLocalMsg("[msp] Collision warning removed.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
//				System.out.println("R"+MSPMathUtils.fromRad(MIN_REL_ANGLE)+" -> "+MSPMathUtils.fromRad(relAngle) +" :"+ MSPMathUtils.fromRad(obstacle.angle_xy)+" :"+MSPMathUtils.fromRad(plannedPath.angle_xy));
				tooClose = false;
			}

			if(tooClose && relAngle > MIN_REL_ANGLE) {
				logger.writeLocalMsg("[msp] Collision warning removed (Angle).",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
//				System.out.println("R"+MSPMathUtils.fromRad(MIN_REL_ANGLE)+" -> "+MSPMathUtils.fromRad(relAngle) +" :"+ MSPMathUtils.fromRad(obstacle.angle_xy)+" :"+MSPMathUtils.fromRad(plannedPath.angle_xy));
				tooClose = false;
			}

		}
	}

	protected void takeoffCompletedAction() {
		super.takeoffCompletedAction();
	}


	@Override
	public boolean update(Point3D_F64 point, Point3D_F64 body) {

		if(!smooth_target_initialized) {
			smooth_target_initialized = true;
			smooth_target.x = point.x;
			smooth_target.y = point.y;
			smooth_target.z = model.target_state.l_z;
			return false;

		}

		smooth_target.x = smooth_target.x * (1- SMOOTH_TARGET_FILTER) + point.x * SMOOTH_TARGET_FILTER;
		smooth_target.y = smooth_target.y * (1- SMOOTH_TARGET_FILTER) + point.y * SMOOTH_TARGET_FILTER;
		smooth_target.z = model.target_state.l_z;

		current.set(model.state.l_x, model.state.l_y, model.state.l_z, model.attitude.y);
		target.set((float)smooth_target.x, (float)smooth_target.y, (float)smooth_target.z,MSP3DUtils.angleXY(target, current));

		offboard.updateTarget(target);

		return true;
	}



	@Override
	public void moveto(float x, float y, float z, float yaw) {
		logger.writeLocalMsg("[msp] New setpoint "+String.format("(%.1f,%.1f)",x,y),MAV_SEVERITY.MAV_SEVERITY_DEBUG);
	//	System.out.println(MSPMathUtils.fromRad(MIN_REL_ANGLE)+" -> "+MSPMathUtils.fromRad(relAngle) +":"+ MSPMathUtils.fromRad(obstacle.angle_xy)+":"+MSPMathUtils.fromRad(plannedPath.angle_xy));
		super.moveto(x, y, z, yaw);
	}

}
