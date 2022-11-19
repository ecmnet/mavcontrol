package com.comino.mavcontrol.offboard3.states;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector4D_F32;

public class Offboard3Current extends Offboard3State {

	protected final GeoTuple4D_F32<?> current_pos_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple4D_F32<?> current_vel_setpoint = new Point4D_F32(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
	protected final GeoTuple3D_F32<?> current_acc_setpoint = new Point3D_F32(Float.NaN, Float.NaN, Float.NaN);


	private final DataModel model;

	public Offboard3Current() {
		this.model =null;
	}

	public Offboard3Current(DataModel model) {
		this.model = model;
		update();
	}

	public GeoTuple4D_F32<?> sep() {
		return current_pos_setpoint;
	}

	public GeoTuple4D_F32<?> sev() {
		return current_vel_setpoint;
	}
	
	public GeoTuple3D_F32<?> sea() {
		return current_acc_setpoint;
	}


	public void update() {

		if(model!=null) {
			MSP3DUtils.convertCurrentPosition(model, pos);
			MSP3DUtils.convertCurrentSpeed(model, vel);
			MSP3DUtils.convertCurrentAcceleration(model,acc);
			MSP3DUtils.convertTargetState(model, current_pos_setpoint);
			MSP3DUtils.convertTargetSpeed(model, current_vel_setpoint);
			MSP3DUtils.convertTargetAcceleration(model, current_acc_setpoint);
			
		} else {
			System.err.println("Current state not updated. Model is NULL");
		}

	}
}
