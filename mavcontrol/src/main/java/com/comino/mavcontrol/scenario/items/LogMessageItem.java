package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcontrol.IOffboardControl;

public class LogMessageItem extends AbstractScenarioItem {
	
	private String message;
	private char   type;

	public LogMessageItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
	}
	
	public void setMessage(String text, char type) {
		this.message = text;
		this.type    = type;
	}

	@Override
	public void execute() {
		switch(type) {
		case 'W':
			control.writeLogMessage(new LogMessage("[msp] "+message,MAV_SEVERITY.MAV_SEVERITY_WARNING));
			break;
		case 'E':
			control.writeLogMessage(new LogMessage("[msp] "+message,MAV_SEVERITY.MAV_SEVERITY_CRITICAL));
			break;
		default:
			control.writeLogMessage(new LogMessage("[msp] "+message,MAV_SEVERITY.MAV_SEVERITY_INFO));
		}
		  completed();
	}

}
