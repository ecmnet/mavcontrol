package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.autopilot.actions.TakeOffHandler;
import com.comino.mavcontrol.scenario.ScenarioManager;

public class ArmingItem extends AbstractScenarioItem {


	public ArmingItem(IMAVController control) {
		super(AbstractScenarioItem.TYPE_ARM,control);
	}

	@Override
	public void execute() {
		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,( cmd,result) -> { 
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					control.writeLogMessage(new LogMessage("[msp] Arming denied. Abort scenario.", MAV_SEVERITY.MAV_SEVERITY_INFO));
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
