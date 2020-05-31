package com.comino.mavcontrol.struct;

public interface ISeqAction {

	public static final int NONE = 0;
	public static final int ABS  = 1;
	public static final int REL  = 2;

	public boolean execute();

}
