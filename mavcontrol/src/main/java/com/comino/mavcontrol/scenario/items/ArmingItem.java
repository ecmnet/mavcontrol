package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;

public class ArmingItem extends AbstractScenarioItem {


	public ArmingItem(IMAVController control) {
		super(control);
	}

	@Override
	public void execute() {
		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			
			if(control.isSimulation()) {
				
				// SITL: Reset mode in order to be able to re-arm after offboard mode 
				// TODO: Might be required for real vehicle also => Check; First try without props
				
				control.writeLogMessage(new LogMessage("[mgc] Arming prep.: Switch to Loiter",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd,result) -> {
					if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) 
						control.writeLogMessage(new LogMessage("Arming preparation failed",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
			}

			
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,( cmd,result) -> { 
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					abort();
				}
				else {
					wait(1000);
					completed();
				}
			},1 );
		}
		else {
			control.writeLogMessage(new LogMessage("[msp] Already armed. Item skipped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			completed();
		}
	}

	@Override
	public void initialize() {
		super.initialize();
		
	}

}
