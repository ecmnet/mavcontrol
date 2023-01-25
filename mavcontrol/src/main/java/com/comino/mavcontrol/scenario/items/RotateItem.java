package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavutils.MSPMathUtils;

public class RotateItem extends AbstractScenarioItem {

	private float yaw_rad = Float.NaN;

	public RotateItem(IMAVController control) {
		super(control);
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
		if(Float.isFinite(yaw_rad))
		  offboard.rotate(yaw_rad, (m) -> completed());
		else
		  completed();
	}

}
