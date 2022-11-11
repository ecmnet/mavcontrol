package com.comino.mavcontrol.offboard3.states;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector4D_F32;

public class Offboard3CurrentState extends Offboard3State {

	protected final GeoTuple4D_F32<?> current_pos_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	
	private final DataModel model;
	
	
	public Offboard3CurrentState(DataModel model) {
		this.model = model;
	}
	
	public GeoTuple4D_F32<?> pos() {
		return pos;
	}
	
	public GeoTuple4D_F32<?> vel() {
		return vel;
	}
	
	public GeoTuple4D_F32<?> acc() {
		return acc;
	}
	
	public GeoTuple4D_F32<?> sep() {
		return current_pos_setpoint;
	}
	
	public void update() {

		MSP3DUtils.convertCurrentPosition(model, pos);
		MSP3DUtils.convertCurrentSpeed(model, vel);
		MSP3DUtils.convertCurrentAcceleration(model,acc);
		MSP3DUtils.convertTargetState(model, current_pos_setpoint);
		tms_us = model.state.tms;

	}
}
