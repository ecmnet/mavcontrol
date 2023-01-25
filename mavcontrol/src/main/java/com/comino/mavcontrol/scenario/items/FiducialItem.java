package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class FiducialItem extends AbstractScenarioItem {

	private final GeoTuple4D_F32<?> fiducial_position = new Vector4D_F32();
	
	private final static msg_msp_vision msg = new msg_msp_vision(2,1);

	public FiducialItem(IMAVController control) {
		super(control);
		fiducial_position.setTo(Float.NaN,Float.NaN,Float.NaN,Float.NaN);
	}

	public void setPositionLocal(float x, float y, float z, float w_deg) {
		fiducial_position.setTo(x,y,z,MSPMathUtils.toRad(w_deg));
	}

	@Override
	public void execute() {
		
		if(!control.isSimulation()) {
			completed();
			return;
		}
			
		if(Float.isFinite(fiducial_position.x) && Float.isFinite(fiducial_position.y)) {

			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, true);

			model.vision.px = fiducial_position.x;
			model.vision.py = fiducial_position.y;
			model.vision.pz = 0f;	
			
			if(Float.isFinite(fiducial_position.w))
				model.vision.pw =fiducial_position.w;
			else
				model.vision.pw = ((float)Math.random()-0.5f)*12f;

		} else {
			
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, false);
			
		}
		
		msg.px    =  model.vision.px;
		msg.py    =  model.vision.py;
		msg.pz    =  model.vision.pz;
		msg.pw    =  model.vision.pw;
		msg.flags =  model.vision.flags;
		
		control.sendMAVLinkMessage(msg);
		
		completed();
	}

}
