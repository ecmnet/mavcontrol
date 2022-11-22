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
			
			pos.setTo(model.state.l_x,  model.state.l_y,  model.state.l_z,  model.attitude.y);
			vel.setTo(model.state.l_vx, model.state.l_vy, model.state.l_vz, model.attitude.yr);
			acc.setTo(model.state.l_ax, model.state.l_ay, model.state.l_az);
		
			current_pos_setpoint.setTo(model.target_state.l_x,  model.target_state.l_y,  model.target_state.l_z,  model.attitude.sy);
			current_vel_setpoint.setTo(model.target_state.l_vx, model.target_state.l_vy, model.target_state.l_vz, model.attitude.syr);
			current_acc_setpoint.setTo(model.target_state.l_ax, model.target_state.l_ay, model.target_state.l_az);

			
		} else {
			System.err.println("Current state not updated. Model is NULL");
		}

	}
}
