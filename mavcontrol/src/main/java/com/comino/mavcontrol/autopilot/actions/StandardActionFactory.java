package com.comino.mavcontrol.autopilot.actions;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;
import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.sequencer.ISeqAction;
import com.comino.mavcontrol.sequencer.Sequencer;
import com.comino.mavcontrol.struct.SeqItem;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;

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

	/**
	 * Performs a precision landing procedure
	 */
	public static void precisionLanding(Sequencer sequencer, IMAVController control) {
		DataModel model = control.getCurrentModel();
		// get fiducial position via cb if locked; otherwise use current
		control.writeLogMessage(new LogMessage("[msp] Precision landing triggered.",MAV_SEVERITY.MAV_SEVERITY_INFO));
		sequencer.replace_add(new SeqItem((target) -> {
			if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
				target.x = model.vision.px;
				target.y = model.vision.py;
				target.z = model.state.l_z + model.hud.al - 0.1f;
				target.w = model.vision.pw;

				msg_msp_vision msg = new msg_msp_vision(2,1);
				msg.px    =  model.vision.px;
				msg.py    =  model.vision.py;
				msg.pz    =  model.vision.pz;
				msg.pw    =  model.vision.pw;
				msg.flags = model.vision.flags;
				control.sendMAVLinkMessage(msg);

			} else {
				MSP3DUtils.convertCurrentState(model, target);
			}
		}, 0.05f , ISeqAction.ABS, () -> {
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND,0, 0, 0, Float.NaN);
			return true;
		}, 0));
	}

	/**
	 * Performs return to land with precision landing 
	 * @param takeoff coordinates (in air)
	 * @param enable (false to abort)
	 */
	
	private final static float RETURN_ALT = 0.8f;
	
	public static void returnToLand(Sequencer sequencer, IMAVController control, Vector4D_F32 takeoff, boolean enable) {

		DataModel model = control.getCurrentModel();

		Vector4D_F32 landing_preparation = takeoff.copy();
		landing_preparation.z = -RETURN_ALT;   
		landing_preparation.w = Float.NaN;

		if(control.isSimulation()) {
			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, true);
		}

		if(!enable) {
			sequencer.abort();
			control.writeLogMessage(new LogMessage("[msp] RTL aborted on user request.",MAV_SEVERITY.MAV_SEVERITY_INFO));
			return;
		}

		if(Float.isNaN(takeoff.x) || Float.isNaN(takeoff.y) || Float.isNaN(takeoff.z)) {
			control.writeLogMessage(new LogMessage("[msp] No valid takeoff ccordinates. Landing.",MAV_SEVERITY.MAV_SEVERITY_INFO));
			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND, 0, 0, 0,  Float.NaN );
			return;
		}

		if(!model.vision.isStatus(Vision.FIDUCIAL_ENABLED)) {
			// go to landing preparation position and perform PX4 landing
			sequencer.replace_add(
					new SeqItem(landing_preparation,ISeqAction.ABS, () -> {
						control.writeLogMessage(new LogMessage("[msp] Standard landing triggered.",MAV_SEVERITY.MAV_SEVERITY_INFO));
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND,0, 0, 0, Float.NaN);
						return true;
					},100));
		} else {
			//   1.go to landing preparation position (assumes that fiducial is visible there) - wait 10ms
			//   2.if fiducial locked determine fiducial position via cb, otherwise remain at landing preparation position
			//   3.move to target and perform PX4 landing
			sequencer.replace_add(
					new SeqItem(landing_preparation,ISeqAction.ABS, () -> {
						control.writeLogMessage(new LogMessage("[msp] Precision landing triggered.",MAV_SEVERITY.MAV_SEVERITY_INFO));
						return true; 
					},10),
					new SeqItem((target) -> {
						if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {
							target.x = model.vision.px;
							target.y = model.vision.py;
							// TODO: Check/Test altitude: Should be 0.1m above landing platform in all cases
							target.z = model.state.l_z + model.hud.al - 0.1f;
							target.w = model.vision.pw;

							msg_msp_vision msg = new msg_msp_vision(2,1);
							msg.px    =  model.vision.px;
							msg.py    =  model.vision.py;
							msg.pz    =  model.vision.pz;
							msg.pw    =  model.vision.pw;
							msg.flags = model.vision.flags;
							control.sendMAVLinkMessage(msg);

						} else {
							target.setTo(landing_preparation);
						}
					}, 0.05f, ISeqAction.ABS, () -> {
						control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_NAV_LAND,0, 0, 0, Float.NaN);
						return true;
					}, 0)
					);
		}

	}

}
