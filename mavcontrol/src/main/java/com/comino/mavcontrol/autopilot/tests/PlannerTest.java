package com.comino.mavcontrol.autopilot.tests;

import org.mavlink.messages.MAV_COMPONENT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MAV_TYPE;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_heartbeat;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;
import org.mavlink.messages.lquac.msg_trajectory_representation_waypoints;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.point.Vector4D_F32;

public class PlannerTest implements Runnable, IMAVLinkListener {

	private static final msg_heartbeat 						beat 		= new msg_heartbeat(1,MAV_COMPONENT.MAV_COMP_ID_OBSTACLE_AVOIDANCE);
	private static final msg_set_position_target_local_ned 	pos_cmd   	= new msg_set_position_target_local_ned(1,2);

	protected DataModel               model    = null;
	protected MSPLogger               logger   = null;
	protected IMAVController          control  = null;

	private final Vector4D_F32		target_p   = new Vector4D_F32();
	private final Vector4D_F32		current    = new Vector4D_F32();

	private boolean is_running;
	private boolean is_planning = false;

	public PlannerTest(IMAVController control, MSPConfig config)  {

		this.control  = control;
		this.model    = control.getCurrentModel();
		this.logger   = MSPLogger.getInstance();

		this.control.addMAVLinkListener(this);

		target_p.set(Float.NaN, Float.NaN, Float.NaN, Float.NaN);

		beat.type = MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER;

	}

	public void setTarget(Vector4D_F32 t) {
		is_planning = true;
		target_p.set(t);
	//	ExecutorService.get().schedule(() -> { is_planning = false; }, 20, TimeUnit.SECONDS);
	}


	public void send() {
		if(!control.isSimulation()) {
			logger.writeLocalMsg("[msp] Not executed - no simulation.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			return;
		}

		is_planning = true;

		target_p.set(1,1,-1,0);

		System.out.println("Planner send bezier 2");
		is_planning = true;



	//	ExecutorService.get().schedule(() -> { is_planning = false; }, 10, TimeUnit.SECONDS);

	}

	public void enable(boolean enable) {

		if(!control.isSimulation()) {
			logger.writeLocalMsg("[msp] Not executed - no simulation.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
			return;
		}

		this.is_running = enable;
		if(enable ) {
			new Thread(this).start();
		}
	}


	public boolean isStarted() {
		return is_running;
	}

	@Override
	public void run() {

		Polar3D_F32 path = new Polar3D_F32(); // planned direct path

		logger.writeLocalMsg("[msp] PX4 Planner started.",MAV_SEVERITY.MAV_SEVERITY_INFO);
		PX4Parameters params = PX4Parameters.getInstance();

		params.sendParameter("COM_OBS_AVOID", 1);
		this.is_planning = false;

		//		msg_trajectory_representation_bezier bez = new msg_trajectory_representation_bezier(1,1);
		msg_trajectory_representation_waypoints wp = new msg_trajectory_representation_waypoints(1,2);

		while(is_running) {
			try { Thread.sleep(200); } catch (InterruptedException e) { }

			current.set(model.state.l_x, model.state.l_y, model.state.l_z,model.attitude.y);
			path.set(target_p, current);

			if(is_planning) {

				if(path.value < 0.2) {
					logger.writeLocalMsg("[msp] Target reached.",MAV_SEVERITY.MAV_SEVERITY_INFO);
                    path.clear();
					is_planning = false;
				}

				wp.pos_x[0]   = target_p.x;
				wp.pos_y[0]   = target_p.y;
				wp.pos_z[0]   = target_p.z;
				wp.pos_yaw[0] = path.angle_xy;

				wp.acc_x[0]   = Float.NaN;
				wp.acc_y[0]   = Float.NaN;
				wp.acc_z[0]   = Float.NaN;

				wp.valid_points = 1;
				wp.time_usec = model.sys.getSynchronizedPX4Time_us();
				control.sendMAVLinkMessage(wp);
			}

			control.sendMAVLinkMessage(beat);
			toModel(target_p,path);

		}
		params.sendParameter("COM_OBS_AVOID", 0);
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PX4_PLANNER, false);
		logger.writeLocalMsg("[msp] PX4 Planner stopped.",MAV_SEVERITY.MAV_SEVERITY_INFO);

	}


	@Override
	public void received(Object o) {
		// Mirror back if not planning
		if( o instanceof msg_trajectory_representation_waypoints && !is_planning) {
			msg_trajectory_representation_waypoints wp = (msg_trajectory_representation_waypoints)o;
			control.sendMAVLinkMessage(wp);
		}

	}

	private void toModel(Vector4D_F32 target, Polar3D_F32 path ) {
		if(is_planning) {
			model.slam.px = target.getX();
			model.slam.py = target.getY();
			model.slam.pz = target.getZ();
			model.slam.pd = path.angle_xy;
			model.slam.di = path.value;
			model.slam.pv = model.state.getSpeed();
		} else {
			model.slam.px = Float.NaN;
			model.slam.py = Float.NaN;
			model.slam.pz = Float.NaN;
			model.slam.pd = Float.NaN;
			model.slam.di = 0;
			model.slam.pv = 0;
		}
		model.slam.tms = model.sys.getSynchronizedPX4Time_us();

	}

	private void sendPositionControlToVehice(Vector4D_F32 target, int frame) {

		pos_cmd.target_component = 1;
		pos_cmd.target_system    = 1;
		pos_cmd.type_mask        = 0b000101111111000;

		pos_cmd.x   = target.x;
		pos_cmd.y   = target.y;
		pos_cmd.z   = target.z;
		pos_cmd.yaw = Float.isNaN(target.w)? model.attitude.y : MSPMathUtils.normAngle(target.w);

		if(target.x==Float.MAX_VALUE) pos_cmd.type_mask = pos_cmd.type_mask | 0b000000000000001;
		if(target.y==Float.MAX_VALUE) pos_cmd.type_mask = pos_cmd.type_mask | 0b000000000000010;
		if(target.z==Float.MAX_VALUE) pos_cmd.type_mask = pos_cmd.type_mask | 0b000000000000100;
		if(target.w==Float.MAX_VALUE) pos_cmd.type_mask = pos_cmd.type_mask | 0b000010000000000;

		pos_cmd.coordinate_frame = frame;

		control.sendMAVLinkMessage(pos_cmd);

	}



}
