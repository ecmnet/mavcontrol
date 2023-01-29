package com.comino.mavcontrol.scenario.items;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcontrol.offboard3.Offboard3Manager;

public abstract class AbstractScenarioItem {
	
	public final static int  POS_TYPE_ABSOLUTE = 0;
	public final static int  POS_TYPE_RELATIVE = 1;

	protected final IMAVController     control;
	protected final DataModel          model;
	protected final Offboard3Manager   offboard;
	
	private       int     delay_ms = 0;
	private       int     pos_type = POS_TYPE_ABSOLUTE;
	
	private       boolean isCompleted;
	private       boolean isAborted;
	private       Object  owner;

	public AbstractScenarioItem(IMAVController control) {

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

	public void execute() {
		control.writeLogMessage(new LogMessage("[msp] ScenarioItem not implemented.", MAV_SEVERITY.MAV_SEVERITY_INFO));
		completed();
	};

	public void setPositionLocal(float x, float y, float z, float w) {

	}


	public void setPositionGlobal(double lat, double lon, double alt, double w) {
		// TODO convert to local position and call setPositionLocal
	}
	
	public void setDelay(int delay_ms) {
		this.delay_ms = delay_ms;
	}

	public long getTimeout_ms() {
		return delay_ms+30_000L;
	}

	public boolean isCompleted() {
		return isCompleted;
	}
	
	public boolean isRelative() {
		return pos_type == POS_TYPE_RELATIVE;
	}
	
	public boolean isAborted() {
		return isAborted;
	}
	
	public void setOwner(Object owner) {
		this.owner = owner;
	}

	protected void completed() {
		synchronized(owner) {
			if(delay_ms > 0)
				wait(delay_ms);
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
