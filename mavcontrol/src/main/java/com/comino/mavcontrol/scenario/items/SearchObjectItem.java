package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;

public class SearchObjectItem extends AbstractScenarioItem {
	
	// NOT IMPLEMENTED YET

	private String objectType;

	public SearchObjectItem(IMAVController control) {
		super(control);
	}
	
	public void setObjectType(String objectType) {
	      this.objectType = objectType;
	}


}
