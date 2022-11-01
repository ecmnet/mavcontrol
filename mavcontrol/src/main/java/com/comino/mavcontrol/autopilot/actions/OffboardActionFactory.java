package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

public class OffboardActionFactory {


	public static boolean turn_to(float heading) {

		Offboard2Manager offboard = Offboard2Manager.getInstance();
		if(offboard!=null) {
			offboard.rotate(heading,null);
			return true;
		}
		return false;
	}

	public static boolean move_to(float x, float y,float z) {

		final Offboard2Manager offboard = Offboard2Manager.getInstance();
		if(offboard!=null) {
			offboard.moveTo(x,y,z,Float.NaN);
			return true;
		}
		return false;

	}
	
	public static void precision_landing_rotate() {
		
		final IMAVController control = AutoPilotBase.getInstance().getControl();
		final DataModel m  = control.getCurrentModel();
		final Offboard2Manager offboard = Offboard2Manager.getInstance();
		final MSPLogger logger = MSPLogger.getInstance();
		
		if(!m.vision.isStatus(Vision.FIDUCIAL_LOCKED))
			return;
		offboard.moveTo(m.vision.px,m.vision.py,Float.NaN,m.vision.pw,() -> {
			
			logger.writeLocalMsg("[mgc] PX4 prec.simulation",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			
//			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
//				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
//					logger.writeLocalMsg("[mgc] PX4 Prec.Landing rejected ("+result+")",MAV_SEVERITY.MAV_SEVERITY_WARNING);
//				else
//					logger.writeLocalMsg("[mgc] PX4 prec.landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
//			},	MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
//					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND );
		});
		
		
		
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
