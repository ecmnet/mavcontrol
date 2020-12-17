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

package com.comino.mavcontrol.controllib.impl;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcontrol.controllib.ISpeedControl;
import com.comino.mavutils.MSPMathUtils;

public class SimpleXYZSpeedControl implements ISpeedControl {

	private static final float MAX_ACCELERATION		                = 0.3f;                   // Max acceleration in m/s2


	private static final float YAW_PV								= 0.005f;				  // P factor yaw control

	private boolean isBreaking  = false;
	private float   speed_incr  = 0;
	private float   acc_incr    = 0;
	private float   delta_angle = 0;

	private DataModel model;
	private float max_speed;
	private float min_speed;
	
	public SimpleXYZSpeedControl(DataModel model, float min_speed, float max_speed) {
		this.model = model;
		this.max_speed = max_speed;
		this.min_speed = min_speed;
	}


	public boolean update(float delta_sec, float ela_sec, float eta_sec, Polar3D_F32 spd, Polar3D_F32 path, Polar3D_F32 ctl) {

		ctl.angle_xz =  path.angle_xz;

		delta_angle = MSPMathUtils.normAngle(path.angle_xy - ctl.angle_xy);

		// follow direction changes by a simple P controller
		if(spd.value > 0.5) {
		  ctl.angle_xy = ctl.angle_xy + delta_angle / delta_sec * YAW_PV;
		  // slow down according to the path angle difference
	      speed_incr = - MAX_ACCELERATION * delta_angle * delta_sec;
		}
		else {
		  ctl.angle_xy = path.angle_xy;
    	}

		// start breaking 1.5 secs before reaching the goal
		if(eta_sec < 2.0 )  {
			isBreaking = true;
		}

		if(isBreaking) {
			speed_incr = - spd.value / ( 2 * eta_sec ) * delta_sec;
			ctl.value = ctl.value + speed_incr;
			if(ctl.value <= min_speed) {
				ctl.value = min_speed;
				model.slam.flags = Slam.OFFBOARD_FLAG_MOVE;
			} else
				model.slam.flags = Slam.OFFBOARD_FLAG_SLOWDOWN;
		} else {
			acc_incr = acc_incr + MAX_ACCELERATION / 2f * delta_sec;
			speed_incr = Math.min(MAX_ACCELERATION, acc_incr) * delta_sec;
			ctl.value = ctl.value + speed_incr;
			if(ctl.value >= max_speed) {
				ctl.value = max_speed;
				model.slam.flags = Slam.OFFBOARD_FLAG_MOVE;
			} else
				model.slam.flags = Slam.OFFBOARD_FLAG_SPEEDUP;
		}
		
		return true;
	}


	public void reset() {
		isBreaking = false; acc_incr = 0; speed_incr = 0;

	}


	public void initialize(Polar3D_F32 spd, Polar3D_F32 path) {
		reset();
	}

}
