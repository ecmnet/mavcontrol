package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class CircleItem extends AbstractScenarioItem {

	private final GeoTuple4D_F32<?> center = new Vector4D_F32();
	private float radius = 0;
	private float angle  = 0;

	public CircleItem(IMAVController control) {
		super(control);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		center.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	public void setCircleParams(float radius, float angle) {
		this.radius = radius;
		this.angle  = MSPMathUtils.toRad(angle);
	}

	public long getTimeout_ms() {
		return (long)(Math.PI*radius*5000/max_xyz_velocity)+super.getTimeout_ms();
	}

	@Override
	public void execute() {
		
		if(!control.isSimulation())
			completed();
		
		if(radius < 0.95f) {
			control.writeLogMessage(new LogMessage("[msp] Radius < 1m. Item skipped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			completed();
			return;
		}

		offboard.circle(center.x, center.y, center.z, center.w, radius, angle, (m) -> completed());

	}

}
