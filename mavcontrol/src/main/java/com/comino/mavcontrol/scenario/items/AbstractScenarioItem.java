package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.Offboard3Manager;

public abstract class AbstractScenarioItem {

	public static final int  TYPE_ARM  		    = 1;
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

	private final int     type;
	

	private       int     delay_secs = 0;
	
	private       boolean isCompleted;
	private       boolean isAborted;
	private       Object  owner;

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

	public void initialize() {
		this.isCompleted = false;
		this.isAborted   = false;
	}

	public abstract void execute();

	public void setPositionLocal(float x, float y, float z, float w) {

	}

	public void setPositionGlobal(double lat, double lon, double alt, double w) {
		// TODO convert to local position and call setPositionLocal
	}
	
	public void setDelay(int secs) {
		this.delay_secs = secs;
	}

	public long getTimeout_ms() {
		return 30_000L;
	}

	public int getType() {
		return type;
	}

	public boolean isCompleted() {
		return isCompleted;
	}
	
	public boolean isAborted() {
		return isAborted;
	}
	
	public void setOwner(Object owner) {
		this.owner = owner;
	}

	protected void completed() {
		synchronized(owner) {
			if(delay_secs > 0)
				wait(delay_secs*1_000);
			this.isCompleted= true;
			owner.notify();
		}
	}

	protected void abort() {
		synchronized(owner) {
			this.isAborted = true;
			owner.notify();
		}
	}
	
	protected void wait(int millisec) {
		try { 
			Thread.sleep(millisec); 
		} catch(Exception e ) { }
	}
	
	public String toString() {
		return this.getClass().getSimpleName();
	}


}
