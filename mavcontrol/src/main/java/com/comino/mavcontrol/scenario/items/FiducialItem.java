package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class FiducialItem extends AbstractScenarioItem {

	private final GeoTuple4D_F32<?> fiducial_position = new Vector4D_F32();

	public FiducialItem(IMAVController control) {
		super(AbstractScenarioItem.TYPE_FIDUCIAL,control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		fiducial_position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	@Override
	public void execute() {
		
	}

	@Override
	public void initialize() {
       // Nothing to prepare
	}

}
