package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavutils.MSPMathUtils;

public class TestActionFactory {
	
	final static msg_msp_vision msg = new msg_msp_vision(2,1);
	
	public static void simulateFiducial(IMAVController control, float radius) {

		final DataModel model = control.getCurrentModel();
//
//		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK)) {
//			if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
//				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
//				msg_msp_vision msg = new msg_msp_vision(2,1);
//				msg.flags =  model.vision.flags;
//				control.sendMAVLinkMessage(msg);
//			}
//			return;
//		}
		
		if(model.sys.isStatus(Status.MSP_LANDED)) {
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			msg.px    =  Float.NaN;
			msg.py    =  Float.NaN;
			msg.pz    =  Float.NaN;
			msg.pw    =  Float.NaN;
			msg.flags =  model.vision.flags;
			control.sendMAVLinkMessage(msg);
			return;
		}

		if(!model.vision.isStatus(Vision.FIDUCIAL_LOCKED) && model.sys.isStatus(Status.MSP_LPOS_VALID) && 
				!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_TAKEOFF) ) {

		    model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, true);

			model.vision.px = model.state.l_x + ((float)Math.random()-0.5f)*radius;
			model.vision.py = model.state.l_y + ((float)Math.random()-0.5f)*radius;
			model.vision.pz = 0.5f;	
			model.vision.pw = ((float)Math.random()-0.5f)*12f;
			
			model.vision.pw = MSPMathUtils.toRad(0);
			
			System.out.println("Simulated fiducial rotation: "+MSPMathUtils.fromRad(model.vision.pw));
			
			msg.px    =  model.vision.px;
			msg.py    =  model.vision.py;
			msg.pz    =  model.vision.pz;
			msg.pw    =  model.vision.pw;
			msg.flags =  model.vision.flags;
			control.sendMAVLinkMessage(msg);
		} 

	}

}
