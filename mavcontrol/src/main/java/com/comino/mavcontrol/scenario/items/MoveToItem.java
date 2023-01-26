package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class MoveToItem extends AbstractScenarioItem {


	public final static int TYPE_ABSOLUTE = 0;
	public final static int TYPE_RELATIVE = 1;

	private final GeoTuple4D_F32<?> target_position = new Vector4D_F32();
	private float acceptance_radius_m = Float.NaN;

	private int   type = TYPE_ABSOLUTE;

	public MoveToItem(IMAVController control) {
		super(control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		target_position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	public void setType(int type) {
		this.type = type;
	}

	public void setPositionDistance(float d, float axy, float axz, float w_deg) {
		System.err.println("Not implemented");
	}

	public void setAcceptanceRadius(float acceptance_radius_m) {
		this.acceptance_radius_m = acceptance_radius_m;
	}

	@Override
	public void execute() {

		switch(type) {

		case TYPE_ABSOLUTE:

			if(Float.isFinite(acceptance_radius_m))
				offboard.moveTo(target_position.x, 
						target_position.y, 
						target_position.z, 
						target_position.w, 
						(m) -> completed(),acceptance_radius_m);
			else
				offboard.moveTo(target_position.x, 
						target_position.y, 
						target_position.z, 
						target_position.w, 
						(m) -> completed());

			break;

		case TYPE_RELATIVE:

			if(Float.isFinite(acceptance_radius_m))
				offboard.moveTo(target_position.x + model.state.l_x, 
						target_position.y + model.state.l_y, 
						target_position.z + model.state.l_z, 
						target_position.w, (m) -> completed(),acceptance_radius_m);
			else
				offboard.moveTo(target_position.x + model.state.l_x, 
						target_position.y + model.state.l_y, 
						target_position.z + model.state.l_z,  
						target_position.w, 
						(m) -> completed());


		}

	}


}
