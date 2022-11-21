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
import com.comino.mavcontrol.offboard3.Offboard3Manager;

public class OffboardActionFactory {


	public static boolean turn_to(float heading) {

		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		if(offboard!=null) {
			offboard.rotate(heading,null);
			return true;
		}
		return false;
	}

	public static boolean move_to(float x, float y,float z) {

		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		if(offboard!=null) {
			offboard.moveTo(x,y,z,Float.NaN);
			return true;
		}
		return false;

	}
	
	public static void precision_landing_rotate() {
		
		final IMAVController control = AutoPilotBase.getInstance().getControl();
		final DataModel m  = control.getCurrentModel();
		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		final MSPLogger logger = MSPLogger.getInstance();
		
		if(!m.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
			logger.writeLocalMsg("[mgc] Fiducial not visible. Not executed.",MAV_SEVERITY.MAV_SEVERITY_INFO);
			return;
		}
		
		logger.writeLocalMsg("[mgc] PX4 prec.Landing with rotate.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		
		offboard.moveTo(m.vision.px,m.vision.py,Float.NaN,m.vision.pw,() -> {
			
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED)
					logger.writeLocalMsg("[mgc] PX4 Prec.Landing rejected ("+result+")",MAV_SEVERITY.MAV_SEVERITY_WARNING);
				else
					logger.writeLocalMsg("[mgc] PX4 prec.landing initiated",MAV_SEVERITY.MAV_SEVERITY_INFO);
			},	MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND );
		});
		
		
		
	}

}
