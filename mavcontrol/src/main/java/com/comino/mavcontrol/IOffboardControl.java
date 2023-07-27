package com.comino.mavcontrol;

import com.comino.mavcontrol.offboard3.action.ITargetReached;
import com.comino.mavcontrol.offboard3.action.ITimeout;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;

import georegression.struct.GeoTuple4D_F32;

public interface IOffboardControl {

	void rotate(float radians, ITargetReached action);

	void rotateBy(float radians, ITargetReached action);

	void executePlan(Offboard3Plan plan, ITargetReached action);

	void setMaxVelocity(float velocity_max_ms);

	void setTimeoutAction(ITimeout timeout);

	void moveTo(float x, float y, float z, float w, ITargetReached action);

	void moveTo(float x, float y, float z, float w, ITargetReached action, float acceptance_radius_m);

	void moveTo(float x, float y, float z, float w);

	void abort();

	boolean isPlanned();

	void getProjectedPositionAt(float time, GeoTuple4D_F32<?> pos);

}