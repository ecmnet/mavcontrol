package com.comino.mavcontrol.autopilot.tests;

import com.comino.mavcontrol.sequencer.ISeqAction;
import com.comino.mavcontrol.sequencer.Sequencer;
import com.comino.mavcontrol.struct.SeqItem;

public class SequenceTestFactory {


	public static void randomSequence(Sequencer sequencer) {
		sequencer.clear();

		for(int i=1;i<5;i++)
			sequencer.add(new SeqItem((float)(Math.random()*4-2),
					(float)(Math.random()*4-2),
					(float)(-Math.random()*0.5+0.2),
					Float.NaN, ISeqAction.REL, null,0));
		sequencer.add(new SeqItem( 0.5f    ,       0.5f  , -2.0f, (float)(Math.PI), ISeqAction.ABS,null,0));

		sequencer.execute();
	}
	

}
