package com.comino.mavcontrol.struct;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.controllib.ISpeedControl;
import com.comino.mavcontrol.sequencer.ISeqAction;

import georegression.struct.point.Vector4D_F32;

public class SeqItem {

	public static final int ITEM_TIMEOUT_MS = 60000;


	private Vector4D_F32              target = null;
	private int                          mode = 0;
	private ISeqAction                 action = null;
	private ISpeedControl  control = null;
	private int                      delay_ms = 0;



	public SeqItem(float x, float y, float z, float w, int mode, ISeqAction action, int delay_ms) {
		this(x,y,z,w,ISeqAction.REL,action,null,delay_ms);
	}

	public SeqItem(float x, float y, float z, float w, int mode, ISeqAction action, ISpeedControl control, int delay_ms) {
		this.target    = new Vector4D_F32(x,y,z,w);
		this.mode      = mode;
		this.action    = action;
		this.control   = control;
		this.delay_ms  = delay_ms;
	}

	public SeqItem(Vector4D_F32 target, int mode, ISeqAction action, int delay_ms) {
		this.target    = target.copy();
		this.mode      = mode;
		this.action    = action;
		this.delay_ms  = delay_ms;
	}

	public SeqItem(float x, float y, float z, float w) {
		this(x,y,z,w,ISeqAction.REL,null,null,0);
	}

	public SeqItem(float x, float y, float z, float w, int mode) {
		this(x,y,z,w,mode,null,null, 0);
	}

	public SeqItem(ISeqAction action, int delay_ms) {
		this(Float.NaN, Float.NaN, Float.NaN, Float.NaN,ISeqAction.REL,action, null, delay_ms);
	}

	public SeqItem(ISeqAction action) {
		this(Float.NaN, Float.NaN, Float.NaN, Float.NaN,ISeqAction.REL,action, null, 0);
	}

	public Vector4D_F32 getTarget(DataModel model) {

		switch(mode) {
		// Note: If NaN use current PX4 setpoint for XYZ, for Yaw use current yaw
		case ISeqAction.REL:
			target.x = Float.isNaN(target.x) ? model.target_state.l_x : target.x + model.state.l_x;
			target.y = Float.isNaN(target.y) ? model.target_state.l_y : target.y + model.state.l_y;
			target.z = Float.isNaN(target.z) ? model.target_state.l_z : target.z + model.state.l_z;
			target.w = Float.isNaN(target.w) ? Float.NaN : target.w + model.attitude.y;
			break;
		case ISeqAction.ABS:
			target.x = Float.isNaN(target.x) ? model.target_state.l_x : target.x ;
			target.y = Float.isNaN(target.y) ? model.target_state.l_y : target.y ;
			target.z = Float.isNaN(target.z) ? model.target_state.l_z : target.z ;
			target.w = Float.isNaN(target.w) ? Float.NaN : target.w;
		default:

		}

		return target;
	}

	public boolean hasTarget() {
		return !Float.isNaN(target.x) || !Float.isNaN(target.y)  || !Float.isNaN(target.z)  || !Float.isNaN(target.w);
	}

	public boolean isRelative() {
		return mode == ISeqAction.REL;
	}

	public long getTimeout_ms() {
		return ITEM_TIMEOUT_MS + delay_ms;
	}

	public ISpeedControl getControlListener() {
		return control;
	}

	public boolean executeAction() {

		if(delay_ms > 0) {
			try { Thread.sleep(delay_ms); } catch(Exception e) { }
		}

		if(action!=null) {
			return action.execute();
		}
		return true;
	}


}
