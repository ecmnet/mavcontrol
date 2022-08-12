package com.comino.mavcontrol.ekf2utils;

import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_local_position_ned;
import org.mavlink.messages.lquac.msg_msp_ekf2_reset;
import org.mavlink.messages.lquac.msg_odometry;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;

import georegression.struct.point.Vector3D_F64;

public class EKF2ResetCheck implements IMAVLinkListener {

	private int   reset_counter_old = 0;
	private final List<Runnable> listener = new ArrayList<Runnable>();
	private final IMAVController control;
	private final DataModel model;

	private float local_pos_x = 0;
	private float local_pos_y = 0;
	private float local_pos_z = 0;

	private int counter = 0;

	private final msg_msp_ekf2_reset reset_msg = new msg_msp_ekf2_reset(2,1);

	public EKF2ResetCheck(IMAVController control) {
		this.control = control;
		this.model = control.getCurrentModel();
		control.addMAVLinkListener(msg_odometry.class, this);
		System.out.println("EKF2 reset counter check initialized..");
	}

	public void addListener(Runnable r) {
		listener.add(r);
	}

	public void reset() {

		this.counter = 0;

		model.state.l_rx = 0;
		model.state.l_ry = 0;
		model.state.l_rz = 0;

		local_pos_x = 0;
		local_pos_y = 0;
		local_pos_z = 0;

	}
	
	@Override
	public void received(Object o) {
		msg_odometry odom = (msg_odometry) o;

		if(reset_counter_old != odom.reset_counter) {
			reset_counter_old = odom.reset_counter;	

			// calculate offset between previous lpos and new one
			model.state.l_rx += (odom.x - local_pos_x);
			model.state.l_ry += (odom.y - local_pos_y);
			model.state.l_rz += (odom.z - local_pos_z);

			// Run listener
			listener.forEach((r) -> r.run());

			// Send MSP message to GC
			reset_msg.offset_x = model.state.l_rx;
			reset_msg.offset_y = model.state.l_ry;
			reset_msg.offset_z = model.state.l_rz;
			reset_msg.counter  = ++counter;
			reset_msg.tms = DataModel.getSynchronizedPX4Time_us();
			control.sendMAVLinkMessage(reset_msg);

			control.writeLogMessage(new LogMessage("[msp] EKF2 reset detected.", MAV_SEVERITY.MAV_SEVERITY_DEBUG)); 

		} else {
			
			local_pos_x = odom.x;
			local_pos_y = odom.y;
			local_pos_z = odom.z;
			
		}
	}	

}