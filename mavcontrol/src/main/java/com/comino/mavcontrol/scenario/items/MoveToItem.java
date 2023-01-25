package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class MoveToItem extends AbstractScenarioItem {

	private final GeoTuple4D_F32<?> target_position = new Vector4D_F32();
	private float acceptance_radius_m = Float.NaN;

	public MoveToItem(IMAVController control) {
		super(control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
       target_position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}
	
	public void setAcceptanceRadius(float acceptance_radius_m) {
	      this.acceptance_radius_m = acceptance_radius_m;
		}

	@Override
	public void execute() {
		if(Float.isFinite(acceptance_radius_m))
		 offboard.moveTo(target_position.x, target_position.y, target_position.z, target_position.w, (m) -> completed(),acceptance_radius_m);
		else
		 offboard.moveTo(target_position.x, target_position.y, target_position.z, target_position.w, (m) -> completed());
	}


}
