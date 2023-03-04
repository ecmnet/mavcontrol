package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;

public class DisarmingItem extends AbstractScenarioItem {


	public DisarmingItem(IMAVController control) {
		super(control);
	}

	@Override
	public void execute() {
		if(model.sys.isStatus(Status.MSP_ARMED)) {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM,( cmd,result) -> { 
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					abort();
				}
				else {
					while(model.sys.isStatus(Status.MSP_ARMED))
					  wait(100);
					wait(1000);
					completed();
				}
			},0 );
		}
		else {
			control.writeLogMessage(new LogMessage("[msp] Already disarmed. Item skipped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			completed();
		}
	}

	@Override
	public void initialize() {
		super.initialize();
		
	}

}
