package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.IOffboardControl;

public class SearchObjectItem extends AbstractScenarioItem {
	
	// NOT IMPLEMENTED YET

	private String objectType;

	public SearchObjectItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
	}
	
	public void setObjectType(String objectType) {
	      this.objectType = objectType;
	}


}
