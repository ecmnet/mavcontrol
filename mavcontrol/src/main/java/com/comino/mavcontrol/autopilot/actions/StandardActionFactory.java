package com.comino.mavcontrol.autopilot.actions;

import com.comino.mavcontrol.sequencer.ISeqAction;
import com.comino.mavcontrol.sequencer.Sequencer;
import com.comino.mavcontrol.struct.SeqItem;
import com.comino.mavutils.MSPMathUtils;

public class StandardActionFactory {
	
	
	/**
	 * Turns vehicle to absolute heading
	 * @param sequencer
	 * @param heading in degree
	 */
	public static void turn_to(Sequencer sequencer,float heading) {
		sequencer.clear();
		float rad = MSPMathUtils.toRad(heading);
		sequencer.add(new SeqItem(Float.NaN,Float.NaN,Float.NaN,rad,ISeqAction.ABS));
		sequencer.execute();
	}
	
	/**
	 * Performs a square around the current position
	 * @param length of the square in m
	 * @param sequencer
	 */
	public static void square(Sequencer sequencer, float length) {
		sequencer.clear();
		sequencer.add(new SeqItem(length/2 , length/2 , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.add(new SeqItem(Float.NaN, -length  , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.add(new SeqItem(-length  , Float.NaN, Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.add(new SeqItem(Float.NaN, length   , Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.add(new SeqItem(length   , Float.NaN, Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.add(new SeqItem(-length/2, -length/2 ,Float.NaN, Float.NaN, ISeqAction.REL,null,0));
		sequencer.execute();
	}

}
