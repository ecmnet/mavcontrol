package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
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
	 * @param 1/2 length of the square in m
	 * @param sequencer
	 */
	public static void square(Sequencer sequencer, float length) {

		sequencer.clear();
		sequencer.add(new SeqItem( length, length, Float.NaN, Float.NaN, length/4, ISeqAction.ABS));
		sequencer.add(new SeqItem( length,-length, -0.5f, Float.NaN, length/4, ISeqAction.ABS));
		sequencer.add(new SeqItem(-length,-length, -3.0f, Float.NaN, length/4, ISeqAction.ABS));
		sequencer.add(new SeqItem(-length, length, -1.5f, Float.NaN, length/4, ISeqAction.ABS));
		sequencer.add(new SeqItem( length, length, Float.NaN, Float.NaN, length/4, ISeqAction.ABS));
		sequencer.add(new SeqItem(0, 0,Float.NaN, Float.NaN, ISeqAction.ABS));
		sequencer.execute();
	}


	public static void precisionLanding(Sequencer sequencer, IMAVController control, float lockx, float locky, float orientation) {
		DataModel model = control.getCurrentModel();
		float target_z = model.state.l_z + model.hud.al - 0.1f;
		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			sequencer.replace(new SeqItem( lockx, locky, target_z, orientation, 0.05f , ISeqAction.ABS, () -> {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND,0, 0, 0, Float.NaN);
				return true;
			}, 0));
		} else {
			sequencer.clear();
			sequencer.add(new SeqItem( lockx, locky, target_z, orientation, 0.05f , ISeqAction.ABS, () -> {
				control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND,0, 0, 0, Float.NaN);
				return true;
			}, 0));
			sequencer.execute();
		}
	}


}
