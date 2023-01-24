package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.Offboard3Manager;

public abstract class AbstractScenarioItem {

	public static final int  TYPE_TAKEOFF  		= 100;
	public static final int  TYPE_MOVETO   		= 101;
	public static final int  TYPE_ROTATE   		= 102;
	public static final int  TYPE_LAND     		= 103;
	public static final int  TYPE_PRECLAND  	= 104;

	public static final int  TYPE_FIDUCIAL  	= 300;
	public static final int  TYPE_OBSTACLE  	= 301;

	public static final int  TYPE_TEST  		= 999;

	protected final IMAVController     control;
	protected final DataModel          model;
	protected final Offboard3Manager   offboard;

	private final int type;

	public AbstractScenarioItem(int type, IMAVController control) {

		this.type    = type;
		this.control = control;

		if(control!=null) {
			this.model   = control.getCurrentModel();
			this.offboard = Offboard3Manager.getInstance(control);
		} else {
			this.model    = null;
			this.offboard = null;
		}
	}

	public abstract void initialize();

	public abstract void execute();

	public void setPositionLocal(float x, float y, float z, float w) {

	}

	public void setPositionGlobal(double lat, double lon, double alt, double w) {
		// TODO convert to local position and call setPositionLocal
	}

	public int getType() {
		return type;
	}

	protected void completed() {
		// TODO: Notify manager to proceed to the next item
	}
	
	protected void abort() {
		// TODO: Notify manager to abort the scenario
	}


}
