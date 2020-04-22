package com.comino.mavcontrol.struct;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard.IOffboardTargetAction;

import georegression.struct.point.Vector4D_F32;

public class SeqItem {

	public static final int       ABS    = 0;
	public static final int       REL    = 1;

	private Vector4D_F32          target = null;
	private int                     mode = 0;
	private ISeqAction            action = null;


	public SeqItem(float x, float y, float z, float w) {
		this(x,y,z,w,REL,null);
	}

	public SeqItem(float x, float y, float z, float w, int mode) {
		this(x,y,z,w,mode,null);
	}

	public SeqItem(float x, float y, float z, float w, int mode, ISeqAction action) {
		this.target = new Vector4D_F32(x,y,z,w);
		this.mode   = mode;
		this.action = action;
	}

	public SeqItem(ISeqAction action) {
		this(Float.NaN, Float.NaN, Float.NaN, Float.NaN,REL,action);
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

	public boolean isRelative() {
		return mode == REL;
	}

	public boolean executeAction() {
		if(action!=null) {
			return action.execute();
		}
		return true;
	}


}
