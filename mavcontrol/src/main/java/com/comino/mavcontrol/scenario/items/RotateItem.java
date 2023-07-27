package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavutils.MSPMathUtils;

public class RotateItem extends AbstractScenarioItem {

	private float yaw_rad = Float.NaN;

	public RotateItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
	}

	public void setYaw(float yaw_degree) {
		this.yaw_rad = MSPMathUtils.toRad(yaw_degree);
	}

	@Override
	public long getTimeout_ms() {
		return 10_000L;
	}

	@Override
	public void execute() {
		if(Float.isFinite(yaw_rad)) {
			if(this.isRelative())
				offboard.rotateBy(yaw_rad, (m) -> completed());
			else
				offboard.rotate(yaw_rad, (m) -> completed());
		}
		else
			completed();
	}

}
