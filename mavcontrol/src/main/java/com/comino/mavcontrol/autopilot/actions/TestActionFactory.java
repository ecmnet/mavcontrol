package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

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

			//model.vision.pw = MSPMathUtils.toRad(135);

			System.out.println("Simulated fiducial rotation: "+MSPMathUtils.fromRad(model.vision.pw));

			msg.px    =  model.vision.px;
			msg.py    =  model.vision.py;
			msg.pz    =  model.vision.pz;
			msg.pw    =  model.vision.pw;
			msg.flags =  model.vision.flags;
			control.sendMAVLinkMessage(msg);
		} 

	}

	static int worker = 0;
	public static void test_simulate_yaw_follow() {

		final WorkQueue wq = WorkQueue.getInstance();
		final DataModel m  = AutoPilotBase.getInstance().getControl().getCurrentModel();


		final Offboard2Manager offboard = Offboard2Manager.getInstance();

		offboard.rotate(45, () -> {	
			final long tms = System.currentTimeMillis();

			worker = wq.addCyclicTask("NP", 100, () -> {	
				offboard.rotate(m.attitude.y+(0.3f),null);
				if((System.currentTimeMillis()-tms) > 10000) {
					System.out.println("removed");
					wq.removeTask("NP", worker);	
				}
			});

			System.out.println("Following "+worker);
		});
	}

}
