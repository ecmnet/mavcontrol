package com.comino.mavcontrol.autopilot.actions;

import com.comino.mavcom.model.DataModel;
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

		Offboard2Manager offboard = Offboard2Manager.getInstance();
		if(offboard!=null) {
			offboard.moveTo(x,y,z,Float.NaN);
			return true;
		}
		return false;

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
