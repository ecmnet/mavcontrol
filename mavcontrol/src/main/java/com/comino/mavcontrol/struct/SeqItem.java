package com.comino.mavcontrol.struct;

import com.comino.mavcom.model.DataModel;
import georegression.struct.point.Vector4D_F32;

public class SeqItem {

	public static final int       ABS    = 0;
	public static final int       REL    = 1;

	private Vector4D_F32          target = null;
	private int                     mode = 0;
	private ISeqAction            action = null;
	private int                 delay_ms = 0;


	public SeqItem(float x, float y, float z, float w) {
		this(x,y,z,w,REL,null,0);
	}

	public SeqItem(float x, float y, float z, float w, int mode) {
		this(x,y,z,w,mode,null,0);
	}

	public SeqItem(float x, float y, float z, float w, int mode, ISeqAction action, int delay_ms) {
		this.target    = new Vector4D_F32(x,y,z,w);
		this.mode      = mode;
		this.action    = action;
		this.delay_ms  = delay_ms;
	}

	public SeqItem(Vector4D_F32 target, int mode, ISeqAction action, int delay_ms) {
		this.target    = target.copy();
		this.mode      = mode;
		this.action    = action;
		this.delay_ms  = delay_ms;
	}

	public SeqItem(ISeqAction action, int delay_ms) {
		this(Float.NaN, Float.NaN, Float.NaN, Float.NaN,REL,action, delay_ms);
	}

	public SeqItem(ISeqAction action) {
		this(Float.NaN, Float.NaN, Float.NaN, Float.NaN,REL,action, 0);
	}

	public Vector4D_F32 getTarget(DataModel model) {

		if(mode == REL) {
			target.x = Float.isNaN(target.x) ? Float.NaN : target.x + model.state.l_x;
			target.y = Float.isNaN(target.y) ? Float.NaN : target.y + model.state.l_y;
			target.z = Float.isNaN(target.z) ? Float.NaN : target.z + model.state.l_z;
			target.w = Float.isNaN(target.w) ? Float.NaN : target.w + model.attitude.y;
		}
		return target;
	}

	public boolean hasTarget() {
		return !Float.isNaN(target.x) || !Float.isNaN(target.y)  || !Float.isNaN(target.z)  || !Float.isNaN(target.w);
	}

	public boolean isRelative() {
		return mode == REL;
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
