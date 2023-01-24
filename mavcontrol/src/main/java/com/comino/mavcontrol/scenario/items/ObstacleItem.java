package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class ObstacleItem extends AbstractScenarioItem {
	
	public static final int OBSTACLE_TYPE_SPHERE = 1;
	public static final int OBSTACLE_TYPE_BOX    = 2;

	private final GeoTuple4D_F32<?> position = new Vector4D_F32();
	private float size_m;
	private int   obstacle_type;

	public ObstacleItem(IMAVController control) {
		super(AbstractScenarioItem.TYPE_OBSTACLE,control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}
	
	public void setType(int obstacle_type) {
		this.obstacle_type = obstacle_type;
	}
	
	public void setSize(float size_m) {
		this.size_m = size_m;
	}

	@Override
	public void execute() {
		
	}

	@Override
	public void initialize() {
       // Nothing to prepare
	}

}
