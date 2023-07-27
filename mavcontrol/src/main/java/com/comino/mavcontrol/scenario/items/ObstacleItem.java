package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F32;

public class ObstacleItem extends AbstractScenarioItem {

	public static final int OBSTACLE_TYPE_SPHERE = 1;
	public static final int OBSTACLE_TYPE_BOX    = 2;

	private final GeoTuple4D_F32<?> position = new Vector4D_F32();
	private final GeoTuple3D_F32<?> size     = new Vector3D_F32();
	private int   obstacle_type;

	public ObstacleItem(IMAVController control,IOffboardControl offboard) {
		super(control, offboard);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	public void setType(int obstacle_type) {
		this.obstacle_type = obstacle_type;
	}

	public void setSize(float sx, float sy, float sz) {
		this.size.setTo(sx,sy,sz);
	}

	@Override
	public void execute() {

		if(MSP3DUtils.isFinite(size)) {

			if(Float.isFinite(position.x))
				model.obs.x = position.x;
			else
				model.obs.x = model.state.l_x;

			if(Float.isFinite(position.y))
				model.obs.y = position.y;
			else
				model.obs.y = model.state.l_y;

			if(Float.isFinite(position.z))
				model.obs.z = position.z;
			else
				model.obs.z = model.state.l_z;
			
			model.obs.sx = size.x;
			model.obs.sy = size.y;
			model.obs.sz = size.z;

		} else 
			model.obs.clear();

		completed();

	}

}
