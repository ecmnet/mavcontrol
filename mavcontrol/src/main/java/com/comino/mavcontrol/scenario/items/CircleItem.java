package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class CircleItem extends AbstractScenarioItem {
	
	private final GeoTuple4D_F32<?> center_position = new Vector4D_F32();
	private float radius = 0;
	private float angle  = 0;

	public CircleItem(IMAVController control) {
		super(control);
	}
	
	public void setPositionLocal(float x, float y, float z, float w_deg) {
		center_position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}
	
	public void setCircleParams(float radius, float angle) {
		this.radius = radius;
		this.angle  = MSPMathUtils.toRad(angle);
	}

	@Override
	public void execute() {
		System.err.println("Radius: "+radius +" Rounds: "+(angle/(2*Math.PI)));
		  completed();
	}

}
