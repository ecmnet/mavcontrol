package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class MoveToItem extends AbstractScenarioItem {


	private final GeoTuple4D_F32<?> position = new Vector4D_F32();
	private float acceptance_radius_m = Float.NaN;


	public MoveToItem(IMAVController control) {
		super(control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	public void setPositionDistance(float d, float axy, float axz, float w_deg) {
		System.err.println("Not implemented");
	}

	public void setAcceptanceRadius(float acceptance_radius_m) {
		this.acceptance_radius_m = acceptance_radius_m;
	}
	
	

	@Override
	public long getTimeout_ms() {
		return (long)(Math.sqrt(position.x*position.x + position.y*position.y)/max_xyz_velocity)*5000L
		       + super.getTimeout_ms();
	}

	@Override
	public void execute() {

		switch(pos_type) {

		case POS_TYPE_ABSOLUTE:

			if(Float.isFinite(acceptance_radius_m))
				offboard.moveTo(position.x, 
						position.y, 
						position.z, 
						position.w, 
						(m) -> completed(),acceptance_radius_m);
			else
				offboard.moveTo(position.x, 
						position.y, 
						position.z, 
						position.w, 
						(m) -> completed());

			break;

		case POS_TYPE_RELATIVE:

			if(Float.isFinite(acceptance_radius_m))
				offboard.moveTo(position.x + model.state.l_x, 
						position.y + model.state.l_y, 
						position.z + model.state.l_z, 
						position.w, (m) -> completed(),acceptance_radius_m);
			else
				offboard.moveTo(position.x + model.state.l_x, 
						position.y + model.state.l_y, 
						position.z + model.state.l_z,  
						position.w, 
						(m) -> completed());


		}

	}


}
