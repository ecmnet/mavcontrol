package com.comino.mavcontrol.ekf2utils;

import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_odometry;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.segment.LogMessage;

public class EKF2ResetCheck implements IMAVLinkListener {

	private int   reset_counter_old = 0;
	private final List<Runnable> listener = new ArrayList<Runnable>();
	private final IMAVController control;

	public EKF2ResetCheck(IMAVController control) {
		this.control = control;
		control.addMAVLinkListener(msg_odometry.class, this);
		System.out.println("Reset counter check initilized..");
	}

	public void addListener(Runnable r) {
		listener.add(r);
	}

	@Override
	public void received(Object o) {
		msg_odometry odom = (msg_odometry) o;
		if(reset_counter_old != odom.reset_counter) {
			reset_counter_old = odom.reset_counter;	
			listener.forEach((r) -> r.run());
			control.writeLogMessage(new LogMessage("[msp] EKF2 reset detected.", MAV_SEVERITY.MAV_SEVERITY_DEBUG)); 
		}	
	}	
}