package com.comino.mavcontrol.ekf2utils;

import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_estimator_status;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;


public class EKF2ResetCheck implements IMAVLinkListener {

	private final List<Runnable> listener = new ArrayList<Runnable>();
	private final IMAVController control;
	private final DataModel model;

	private float local_pos_x = 0;
	private float local_pos_y = 0;
	private float local_pos_z = 0;

	private float cum_x_reset = 0;
	private float cum_y_reset = 0;
	private float cum_z_reset = 0;
	
	private int   est_flags_old = 0;


	public EKF2ResetCheck(IMAVController control) {
		this.control = control;
		this.model = control.getCurrentModel();
		control.addMAVLinkListener(msg_estimator_status.class, this);
		
		System.out.println("EKF2 reset counter check initialized..");
		
	}

	public void addListener(Runnable r) {
		listener.add(r);
	}

	public void reset(boolean reset_cumulated_reset) {

		if(reset_cumulated_reset) {
			cum_x_reset = 0;
			cum_y_reset = 0;
			cum_z_reset = 0;
			control.writeLogMessage(new LogMessage("[msp] EKF2 reset offset cleared.", MAV_SEVERITY.MAV_SEVERITY_DEBUG)); 
		}	
		else {
			
			cum_x_reset += model.est.l_x_reset;
			cum_y_reset += model.est.l_y_reset;
			cum_z_reset += model.est.l_z_reset;
		}

		model.est.l_x_reset = 0;
		model.est.l_y_reset = 0;
		model.est.l_z_reset = 0;

		local_pos_x = 0;
		local_pos_y = 0;
		local_pos_z = 0;
		
		est_flags_old = model.est.flags;

	}

	@Override
	public void received(Object o) {
		msg_estimator_status status = (msg_estimator_status) o;

		if((status.flags & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS) == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS &&
			(est_flags_old & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS) == 0) {

			// calculate current offset between previous lpos and new one
			model.est.l_x_reset += (model.state.l_x - local_pos_x);
			model.est.l_y_reset += (model.state.l_y - local_pos_y);
			model.est.l_z_reset += (model.state.l_z - local_pos_z);

			// Run listener
			listener.forEach((r) -> r.run());
			
			control.writeLogMessage(new LogMessage("[msp] EKF2 reset detected.", MAV_SEVERITY.MAV_SEVERITY_DEBUG)); 

		} else {

			local_pos_x = model.state.l_x;
			local_pos_y = model.state.l_y;
			local_pos_z = model.state.l_z;

		}
		
		est_flags_old = status.flags;

		// Corrected LPOS is PX4 LPOS - current offset - cumulated offset 
		model.state.l_rx = model.state.l_x - model.est.l_x_reset - cum_x_reset;
		model.state.l_ry = model.state.l_y - model.est.l_y_reset - cum_y_reset;
		model.state.l_rz = model.state.l_z - model.est.l_z_reset - cum_z_reset;


	}	

}