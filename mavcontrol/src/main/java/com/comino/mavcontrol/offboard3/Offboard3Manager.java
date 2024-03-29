package com.comino.mavcontrol.offboard3;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import org.mavlink.messages.MAV_CMD;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_MODE_FLAG;
import org.mavlink.messages.MAV_RESULT;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.lquac.msg_msp_trajectory;
import org.mavlink.messages.lquac.msg_set_position_target_local_ned;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.mavlink.MAV_CUST_MODE;
import com.comino.mavcom.mavlink.MAV_MASK;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Slam;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.IOffboardControl;
import com.comino.mavcontrol.controllib.impl.YawSpeedControl;
import com.comino.mavcontrol.offboard3.action.ITargetReached;
import com.comino.mavcontrol.offboard3.action.ITimeout;
import com.comino.mavcontrol.offboard3.collision.Offboard3CollisionCheck;
import com.comino.mavcontrol.offboard3.collision.Offboard3OctoMapCollisionCheck;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.states.Offboard3Collision;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.states.Offboard3State;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;
import com.comino.mavcontrol.offboard3.target.Offboard3PosTarget;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.SingleAxisTrajectory;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.MSPStringUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F32;


public class Offboard3Manager implements IOffboardControl {

	private static Offboard3Manager instance;

	private static final int   UPDATE_RATE                 	    = 50;					    // Offboard update rate in [ms]
	private static final float DEFAULT_TIMEOUT                	= 5.0f;				        // Default timeout 1s

	private static final float RADIUS_ACCEPT                    = 0.1f;                     // Acceptance radius in [m]
	private static final float RADIUS_ACCEPT_VEL                = 0.1f;                     // Acceptance radius in [m/s]
	private static final float YAW_ACCEPT                	    = MSPMathUtils.toRad(1);    // Acceptance alignmnet yaw in [rad]

	private static final float MAX_YAW_VEL                      = MSPMathUtils.toRad(100);  // Maxumum speed in [rad/s]
	private static final float MAX_Z_SLOPE_FOR_YAW_CONTROL      = MSPMathUtils.toRad(60);   // Maxumum slope to adjust yaw to the path
	private static final float MIN_DISTANCE_FOR_YAW_CONTROL     = 0.5f;                    // Minimum distance for auto yaw control
	private static final float MIN_YAW_PLANNING_DURATION        = 0.2f;                     // Minumum duration the planner ist used in [s]
	private static final float YAW_PV							= 0.06f;                    // P factor for auto yaw rate control
	private static final float MAX_XYZ_VEL                      = 1.0f;                     // Maxumum speed in [m/s]
	private static final float EMERGENCY_STOP_TIME              = 0.5f;                     // A expected collision within this time leads to an immediate stop


	private final MAVOctoMap3D      map;
	private final Offboard3Worker   worker;
	private final Offboard3Planner  planner;
	private final DataModel         model;
	private final IMAVController    control;

	private float acceptance_radius     = RADIUS_ACCEPT;
	private float acceptance_radius_vel = RADIUS_ACCEPT_VEL;
	private float acceptance_yaw        = YAW_ACCEPT;
	private float max_xyz_vel           = MAX_XYZ_VEL;

	public static IOffboardControl getInstance(IMAVController control,MAVOctoMap3D map) {
		if(instance==null) 
			instance = new Offboard3Manager(control,map);
		return instance;
	}

	public static Offboard3Manager getInstance() {
		return instance;
	}

	private Offboard3Manager(IMAVController control, MAVOctoMap3D map) {
		this.map     = map;
		this.model   = control.getCurrentModel();
		this.worker  = new Offboard3Worker(control);
		this.control = control;

		MSPConfig config	= MSPConfig.getInstance();

		max_xyz_vel = config.getFloatProperty(MSPParams.AUTOPILOT_MAX_XYZ_VEL, String.valueOf(MAX_XYZ_VEL));
		System.out.println("Maximum planning velocity: "+max_xyz_vel+" m/s");
		acceptance_radius = config.getFloatProperty(MSPParams.AUTOPILOT_RADIUS_ACCEPT, String.valueOf(RADIUS_ACCEPT));
		System.out.println("Acceptance radius: "+acceptance_radius+" m");

		this.planner = new Offboard3Planner(control, map, acceptance_radius, max_xyz_vel);
		System.out.println("Offboard3Manager instantiated");

	}


	@Override
	public void rotate(float radians, ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		this.acceptance_radius = RADIUS_ACCEPT;

		Offboard3Plan plan = planner.planDirectYaw(radians);

		if(plan.isEmpty() && action!=null) {
			action.execute(model);
			return;
		}

		worker.setPlan(plan);	
		worker.start(action);


	}

	@Override
	public void rotateBy(float radians, ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return ;

		float target = MSPMathUtils.normAngle(model.attitude.y+radians);

		this.acceptance_radius = RADIUS_ACCEPT;

		Offboard3Plan plan = planner.planDirectYaw(target);

		if(plan.isEmpty() && action!=null) {
			action.execute(model);
			return;
		}

		worker.setPlan(plan);	
		worker.start(action);

	}

	@Override
	public void executePlan(Offboard3Plan plan, ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		if(plan.isEmpty() && action!=null) {
			action.execute(model);
			return;
		}

		worker.setPlan(plan);
		worker.start(action);

	}

	@Override
	public void setMaxVelocity(float velocity_max_ms) {
		this.max_xyz_vel = velocity_max_ms;
		this.planner.setMaxVelocity(velocity_max_ms);
		control.writeLogMessage(new LogMessage("[msp] Set maximum velocity to "+max_xyz_vel+" m/s", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
	}

	@Override
	public void setTimeoutAction(ITimeout timeout) {
		worker.setTimeoutAction(timeout);
	}

	public void circle(float x, float y, float z, float w, float r, float a, ITargetReached action) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;

		if(!control.isSimulation()) {
			System.err.println("CircleTask only in SITL currently");
			return;
		}

		Point4D_F32 p = new Point4D_F32(x,y,z,w);
		Offboard3Plan plan = planner.planCircle(p, r, a);

		if(plan.isEmpty() && action!=null) {
			action.execute(model);
			return;
		}

		worker.setPlan(plan);
		worker.start(action);
	}

	@Override
	public void moveTo(float x, float y, float z, float w, ITargetReached action) {
		moveTo(x,y,z,w,action,RADIUS_ACCEPT);
	}

	@Override
	public void moveTo(float x, float y, float z, float w, ITargetReached action, float acceptance_radius_m) {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
			return;
		}
		

		final Point4D_F32 p = new Point4D_F32(x,y,z,w);
		
		this.acceptance_radius = acceptance_radius_m;

		Offboard3Plan plan = planner.planDirectPath(p);

		if((plan == null || plan.isEmpty()) && action!=null) {
			action.execute(model);
			return;
		}

		worker.setPlan(plan);
		worker.start(action);

		return;
	}

	@Override
	public void moveTo(float x, float y, float z, float w) {
		moveTo(x,y,z,w,null);
	}

	@Override
	public void abort() {

		if(!model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))
			return;
		worker.stopAndLoiter();
	}

	@Override
	public boolean isPlanned() {
		return worker.isPlanned();
	}
	
	@Override
	public void getProjectedPositionAt(float time, GeoTuple4D_F32<?> pos) {
		worker.getProjectedPositionAt(time, pos);
	}


	private class Offboard3Worker implements Runnable {

		private final msg_set_position_target_local_ned cmd = new msg_set_position_target_local_ned(1,1);

		private final IMAVController control;
		private final DataModel      model;

		private final WorkQueue wq = WorkQueue.getInstance();


		private int     offboard_worker = 0;
		private boolean isRunning       = false;
		private boolean offboardEnabled = false;

		// Actions
		private ITargetReached  reached = null;
		private ITimeout        timeout = null;

		// Current target
		private Offboard3AbstractTarget current_target;

		// current state
		private final Offboard3Current current;

		// Collsion check
		private final Offboard3OctoMapCollisionCheck   collisionCheck;
		private final Vector4D_F32                     target_to_stop = new Vector4D_F32();

		// Queue of plans
		private final BlockingQueue<Offboard3Plan> planQueue = new ArrayBlockingQueue<Offboard3Plan>(1);
		private Offboard3Plan current_plan;

		// Executors
		private final SingleAxisTrajectory      yawExecutor = new SingleAxisTrajectory();
		private final RapidTrajectoryGenerator  xyzExecutor = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

		// Controllers
		private final YawSpeedControl           yawControl = new YawSpeedControl(YAW_PV, 0 ,MAX_YAW_VEL);

		// Timing
		private float t_timeout              = 0;
		private float t_section_elapsed      = 0;
		private float t_section_elapsed_last = 0;
		private float t_planned_yaw          = 0;
		private float t_planned_xyz          = 0;

		public Offboard3Worker(IMAVController control) {

			this.control = control;
			this.model   = control.getCurrentModel();
			this.current = new Offboard3Current(model);
			this.collisionCheck = new Offboard3OctoMapCollisionCheck(map);

		}


		public void start(ITargetReached reached_action) {

			if(model.sys.isStatus(Status.MSP_LANDED)) {
				reset();
				control.writeLogMessage(new LogMessage("[msp] Landed. Offboard control rejected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				return;
			}

			if(!model.sys.isStatus(Status.MSP_CONNECTED))
				return;

			this.reached     = reached_action;
			this.t_section_elapsed   = 0;

			if(isRunning)
				return;

			this.t_section_elapsed_last = System.nanoTime();

			isRunning = true; 
			
			if(!wq.isInQueue("NP", offboard_worker))
			   offboard_worker = wq.addCyclicTask("NP", UPDATE_RATE, this);
		}

		public void stopAndLoiter() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd,result) -> {
				if(result == MAV_RESULT.MAV_RESULT_ACCEPTED) 
					stop();	
				else
					control.writeLogMessage(new LogMessage("Switching to hold failed. Continue offboard",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			},MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_AUTO, MAV_CUST_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
		}

		public void stop() {
			this.isRunning       = false;
			this.offboardEnabled = false;
			wq.removeTask("NP", offboard_worker);
			this.offboard_worker = 0;
			reset(); 
			model.slam.clearFlags();		
			model.traj.clear();
			control.sendMAVLinkMessage(new msg_msp_trajectory(2,1));
			return;
		}

		//		public void planRotation(final float yaw) {
		//			setPlan(planner.planDirectYaw(yaw));		
		//		}

		//		public void planMovement(GeoTuple4D_F32<?> pos_target) {
		//
		//			final GeoTuple4D_F32<?> _target = pos_target.copy();
		//			setPlan(planner.planDirectPath(_target));
		//		}

		public void setPlan(Offboard3Plan plan) {
			planQueue.clear();
			if(plan!=null)
				if(!planQueue.offer(plan))
					System.err.println("plan not accepted");
		}

		public boolean isPlanned() {
			return yawExecutor.isPlanned() || xyzExecutor.isPlanned();
		}

		public void setTimeoutAction(ITimeout timeout) {
			this.timeout = timeout;
		}
		
		public void getCurrentPositionAt(GeoTuple4D_F32<?> pos) {
			pos.setTo(current.pos().x, current.pos().y,current.pos().z, current.pos().w);
		}
		
		public void getProjectedPositionAt(float time, GeoTuple4D_F32<?> pos) {
			if(!xyzExecutor.isPlanned()) {
				// get current if no plan available
				MSP3DUtils.convertCurrentPosition(model, pos);
				return;
			}
			// get projected position at current time + time
			xyzExecutor.getPosition(t_section_elapsed + time, pos);
		}

		@Override
		public void run() {

			if(!isRunning)
				return;


			if(!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) && !model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD)) {
				worker.stop(); worker.reset();
				control.writeLogMessage(new LogMessage("[msp] Offboard externally stopped.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				return;
			}

			// Convert current state
			current.update();

			// A new plan is available -> use it
			if(!planQueue.isEmpty()) {
				reset();
				current_plan = planQueue.poll();

				if(current_plan==null) {
					stopAndLoiter();
					return;
				}

				//	control.writeLogMessage(new LogMessage("[msp] Next plan.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

				current_target = planNextSectionExecution(current);	
				t_section_elapsed = 0;

				if(current_target == null) {
					control.writeLogMessage(new LogMessage("[msp] No target. Stopped.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					stopAndLoiter();
					return;
				}

				// Already within the target acceptance radius 
				if(current_target.isPosReached(current.pos(), acceptance_radius, acceptance_yaw) && 
						current_target.isVelReached(current.vel(), acceptance_radius_vel)) {
					control.writeLogMessage(new LogMessage("[msp] Target already reached. Perform action.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					if(reached!=null && planQueue.isEmpty()) {
						ITargetReached action = reached; reached = null;
						action.execute(model);
						if(reached == null)
							stop();
					} else {		
						stopAndLoiter();
					}
					return;
				}

			}
			
			if(current_target == null) {
				control.writeLogMessage(new LogMessage("[msp] No target. Stopped.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				stopAndLoiter();
				return;
			}

			if(current_target.isPositionFinite() && MSP3DUtils.distance3DSQ(current.pos(), current_target.pos())< acceptance_radius*acceptance_radius) {
				if(reached!=null && planQueue.isEmpty() && current_plan.isEmpty()) {
					control.writeLogMessage(new LogMessage("[msp] Reached.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					ITargetReached action = reached; reached = null;
					action.execute(model);
					if(reached != null)
						return;
				}
			}


			//System.out.println("T1 "+t_section_elapsed+":"+xyzExecutor.isPlanned()+":"+current_plan.getTotalTime());

			// timing
			t_section_elapsed_last = t_section_elapsed;
			t_section_elapsed = current_target.getElapsedTime();

			// check current state and perform action 
			if((yawExecutor.isPlanned() || xyzExecutor.isPlanned()) && current_plan.isEmpty() &&
					t_section_elapsed > current_plan.getTotalTime()) {

				// Resend last offboard command until target is hit => avoid instability when switching to HOLD
				cmd.time_boot_ms = model.sys.t_boot_ms;
				cmd.isValid  = true;
				cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
				control.sendMAVLinkMessage(cmd);

				model.slam.setFlag(Slam.OFFBOARD_FLAG_REACHED, true);
				model.slam.di = 0;
				model.slam.ix = Float.NaN;
				model.slam.iy = Float.NaN;
				model.slam.iz = Float.NaN;

				model.traj.clear();
				control.sendMAVLinkMessage(new msg_msp_trajectory(2,1));
				return;

			}

			//System.out.println("T2");

			// Plan next target if required
			if(!current_plan.isEmpty() && xyzExecutor.isPlanned() && t_section_elapsed >= xyzExecutor.getTotalTime()) {

				// replace current by last planned state of previous section to avoid undefined acceleration
				//				if(t_section_elapsed > 0.01f)
				//					current.set(xyzExecutor, t_section_elapsed);

				//				control.writeLogMessage(new LogMessage("[msp] Next section.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

				current_target = planNextSectionExecution(current);	
				if(current_target == null) {
					control.writeLogMessage(new LogMessage("[msp] Execution of plan not possible. Stopped.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					stopAndLoiter();
					return;
				}	
				t_section_elapsed = current_target.getElapsedTime();		
			}
			
			
//			Offboard3Collision collision = collisionCheck.check(xyzExecutor, t_section_elapsed, current,0);
//		
//			if(collision!=null) {
//
//				float stop_time = collision.getExpectedTimeOfCollision() - t_section_elapsed;
//				if(stop_time < EMERGENCY_STOP_TIME) {
//					// Collision within 1 sec
//					control.writeLogMessage(new LogMessage("[msp] Collison within "+ MSPStringUtils.getInstance().t_format(stop_time)+". Stopped.", 
//							MAV_SEVERITY.MAV_SEVERITY_EMERGENCY));
//					stopAndLoiter();
//					return;
//				} else {
//					// Collision expected later 
//					xyzExecutor.getPosition(stop_time-EMERGENCY_STOP_TIME, target_to_stop);
//					Offboard3Plan plan_to_stop = planner.planDirectPath(target_to_stop, collision.getExpectedTimeOfCollision(),true);
//					planQueue.clear(); planQueue.add(plan_to_stop);
//					control.writeLogMessage(new LogMessage("[msp] Collison within "+ MSPStringUtils.getInstance().t_format(stop_time)+". Stopping.", 
//							MAV_SEVERITY.MAV_SEVERITY_CRITICAL));
//					reached = new ITargetReached() {
//						@Override
//						public void execute(DataModel m) {
//							stopAndLoiter();
//						}		
//					};
//					return;			
//				}
//			}
            
			// check timeout
			if(t_timeout > 0 && t_section_elapsed > t_timeout) {
				model.slam.setFlag(Slam.OFFBOARD_FLAG_TIMEOUT, true);
				stopAndLoiter();
				control.writeLogMessage(new LogMessage("[msp] Offboard timeout. Switched to HOLD.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));

				if(timeout!=null)
					timeout.execute();

				updateTrajectoryModel(t_section_elapsed);
				return;
			}

			//System.out.println("T3");

			// Calculate and send command
			synchronized(this) {

				cmd.type_mask        = 0;
				cmd.target_system    = 1;
				cmd.target_component = 1;

				model.slam.clearFlags();


				if(xyzExecutor.isPlanned() && t_section_elapsed <= xyzExecutor.getTotalTime()) {
					model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER, true);
					cmd.x       = (float)xyzExecutor.getPosition(t_section_elapsed,0);
					cmd.y       = (float)xyzExecutor.getPosition(t_section_elapsed,1);
					cmd.z       = (float)xyzExecutor.getPosition(t_section_elapsed,2);
				} else {

					if(!current_target.isTargetSetpoint()) 
						model.slam.setFlag(Slam.OFFBOARD_FLAG_XYZ_DIRECT, true);
					cmd.type_mask    = MAV_MASK.MASK_LOITER_SETPOINT_TYPE;
					cmd.x       = current_target.pos().x;
					cmd.y       = current_target.pos().y;
					cmd.z       = current_target.pos().z;

				}

				if(xyzExecutor.isPlanned() && t_section_elapsed <= xyzExecutor.getTotalTime()) {
					cmd.vx       = (float)xyzExecutor.getVelocity(t_section_elapsed,0);
					cmd.vy       = (float)xyzExecutor.getVelocity(t_section_elapsed,1);
					cmd.vz       = (float)xyzExecutor.getVelocity(t_section_elapsed,2);
				} else {
					cmd.vx       = Float.NaN;
					cmd.vy       = Float.NaN;
					cmd.vz       = Float.NaN;  
				}
				

				if(xyzExecutor.isPlanned() && t_section_elapsed <= xyzExecutor.getTotalTime()) {			
					cmd.afx      = (float)xyzExecutor.getAcceleration(t_section_elapsed,0);
					cmd.afy      = (float)xyzExecutor.getAcceleration(t_section_elapsed,1);
					cmd.afz      = (float)xyzExecutor.getAcceleration(t_section_elapsed,2);		
				} else {
					cmd.afx      = Float.NaN;
					cmd.afy      = Float.NaN;
					cmd.afz      = Float.NaN;
				}
				
				if(current_target.isAutoYaw()) {	

					// Yaw control aligns to path based on velocity direction or setpoint

					if(Math.abs( MSP3DUtils.angleXZ(current.pos(), current_target.pos()) ) < MAX_Z_SLOPE_FOR_YAW_CONTROL) {

						if(xyzExecutor.isPlanned() && t_section_elapsed <= xyzExecutor.getTotalTime()) {
							model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_CONTROL, true);
							cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
							if((cmd.vx*cmd.vx + cmd.vy*cmd.vy) > 0) {
								current_target.pos().w = MSP3DUtils.angleXY(cmd.vx,cmd.vy);
								cmd.yaw      = current_target.pos().w;
								cmd.yaw_rate = yawControl.update(MSPMathUtils.normAngle(cmd.yaw - current.pos().w), t_section_elapsed - t_section_elapsed_last,MAX_YAW_VEL);
								// System.err.println("YAW CONTROL1 "+MSPMathUtils.normAngle(cmd.yaw - current.pos().w)+":"+cmd.yaw_rate);
							} 
						} else {
							// XYZ Target reached but not yaw: Further yaw turning via YAWControl
							if(!yawExecutor.isPlanned() && !current_target.isYawReached(current.pos(), acceptance_yaw)) {
								model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_CONTROL, true);
								cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_IGNORE;
								cmd.yaw       = current_target.pos().w;
								cmd.yaw_rate  = yawControl.update(MSPMathUtils.normAngle(cmd.yaw - current.pos().w), t_section_elapsed - t_section_elapsed_last,MAX_YAW_VEL);	
							} else {
								model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
								cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
								cmd.yaw_rate = 0;
								cmd.yaw      = current_target.pos().w;
							}
						}

					} else {
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
						cmd.yaw_rate = 0;
						cmd.yaw      = current.pos().w;
					}

				} else {

					// Yaw controlled by planner

					if(t_section_elapsed <= yawExecutor.getTotalTime()) {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_PLANNER, true);
						cmd.yaw_rate = (float)yawExecutor.getVelocity(t_section_elapsed);
						cmd.yaw      = (float)yawExecutor.getPosition(t_section_elapsed);
					} else {
						model.slam.setFlag(Slam.OFFBOARD_FLAG_YAW_DIRECT, true);
						cmd.type_mask = cmd.type_mask |  MAV_MASK.MASK_YAW_RATE_IGNORE;
						cmd.yaw_rate = 0;
						cmd.yaw      = current_target.pos().w;
					}

				}



				if(isRunning) {
					cmd.time_boot_ms = model.sys.t_boot_ms;
					cmd.isValid  = true;
					cmd.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
					control.sendMAVLinkMessage(cmd);
				}
			}

			if(!offboardEnabled ) {
				enableOffboard();
			}

			//			model.debug.x = (float)Math.sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy +cmd.vz * cmd.vz);
			//			model.debug.y = (float)Math.sqrt(cmd.afx * cmd.afx + cmd.afy * cmd.afy +cmd.afz * cmd.afz);
			//			model.debug.z = cmd.yaw;


			if(t_section_elapsed < xyzExecutor.getTotalTime()) {
				model.slam.di = MSP3DUtils.distance3D(current_target.pos(), current.pos());
				updateTrajectoryModel(t_section_elapsed);
			}
		}

		private Offboard3AbstractTarget planNextSectionExecution(Offboard3Current current_state) {

			if(current_plan.isEmpty()) {
				return null;
			}

			Offboard3AbstractTarget new_target = current_plan.poll();

			return planSectionExecution(new_target, current_state);
		}

		private Offboard3AbstractTarget planSectionExecution(Offboard3AbstractTarget target, Offboard3Current current_state) {

			float estimated_yaw_duration = 0;

			//			control.writeLogMessage(new LogMessage("Next plan section",MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			//			System.err.println(current_state);

			xyzExecutor.reset(); t_planned_xyz = 0;

			xyzExecutor.setInitialState(current_state.pos(),current_state.vel(),current_state.acc());

			switch(target.getType()) {
			case Offboard3AbstractTarget.TYPE_POS:
				target.replaceNaNPositionBy(current_state.pos());
				if(isFinite(target.pos()) && !target.isPosReached(current_state.pos(),acceptance_radius,Float.NaN)) {
					xyzExecutor.setGoal(target.pos(), target.vel(), target.acc());
					t_planned_xyz = xyzExecutor.generate(target.getPlannedSectionTime());
//					                    MSPStringUtils.getInstance().err("XYZ POS    (Initial):   "+current_state);
//										MSPStringUtils.getInstance().err("XYZ POS    (Execution): "+target);
				}
				break;
			case Offboard3AbstractTarget.TYPE_POS_VEL:
				target.replaceNaNPositionBy(current_state.pos());
				if(isFinite(target.pos()) && !target.isPosReached(current_state.pos(),acceptance_radius,Float.NaN)) {
					target.determineTargetVelocity(current_state.pos());
					xyzExecutor.setGoal(target.pos(), target.vel(), target.acc());
					t_planned_xyz = xyzExecutor.generate(target.getPlannedSectionTime());
							//		MSPStringUtils.getInstance().out("XYZ POSVEL (Execution): "+target);
				}
				break;
			case Offboard3AbstractTarget.TYPE_VEL:
			//	target.replaceNaNPositionBy(current_state.pos());
				if(isValid(target.vel()) && !target.isPosReached(current_state.pos(),acceptance_radius,Float.NaN)) {
					target.determineTargetVelocity(current_state.pos());
					xyzExecutor.setGoal(null, target.vel(), target.acc());
					t_planned_xyz = xyzExecutor.generate(target.getPlannedSectionTime());
						//			MSPStringUtils.getInstance().err("XYZ VEL    (Execution): "+target);
				}
				break;
			case Offboard3AbstractTarget.TYPE_VEL_ACC:
				if(isValid(target.vel()) && !target.isVelReached(current_state.vel(),acceptance_radius_vel)) {
					if(target.isVelocityFinite())
						xyzExecutor.setGoal(null, target.vel(), target.acc());
					else
						xyzExecutor.setGoal(null, null, target.acc());
					t_planned_xyz = xyzExecutor.generate(target.getPlannedSectionTime());
				//	MSPStringUtils.getInstance().out("XYZ VELACC    (Execution): "+target);
				}
				break;
			default:
				System.err.println("Wrong target type:"+target.getType());
				xyzExecutor.setGoal(target.pos(), target.vel(), target.acc());
				t_planned_xyz = xyzExecutor.generate(target.getPlannedSectionTime());
				break;
			}


			// Yaw execturion planning 
			yawExecutor.reset(); t_planned_yaw = 0;
			if(Float.isFinite(target.pos().w)) {

				target.pos().w = normAngle(target.pos().w );  
				current_state.pos().w = normAngle(current_state.pos().w);

				//	        	System.err.println("1. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

				if((target.pos().w - current_state.pos().w) > (float)Math.PI) {
					target.pos().w  = (target.pos().w -(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
				}

				if((target.pos().w - current_state.pos().w) < -(float)Math.PI) {
					target.pos().w  = (target.pos().w +(float)MSPMathUtils.PI2) % (float)MSPMathUtils.PI2;
				}

				//				System.err.println("2. from " + pc.w +" to " + target.getTargetPosition().w + " delta " +(target.getTargetPosition().w - pc.w));

				yawExecutor.setInitialState(current_state.pos().w, current_state.vel().w, 0);
				yawExecutor.setTargetState(target.pos().w , 0, 0);

				if(target.getDuration() < 0)
					estimated_yaw_duration = Math.abs(target.pos().w  - current_state.pos().w)/MAX_YAW_VEL;
				else
					estimated_yaw_duration = target.getDuration();

				if(estimated_yaw_duration > MIN_YAW_PLANNING_DURATION) {
					if(estimated_yaw_duration < 3)
						estimated_yaw_duration = 3;
					t_planned_yaw = yawExecutor.generateTrajectory(estimated_yaw_duration);
					//					MSPStringUtils.getInstance().out("Yaw (Execution): "+MSPMathUtils.fromRad(target.pos().w )+" in "+estimated_yaw_duration+" secs");

				}
			} 
			else if(MSP3DUtils.distance3DSQ(target.pos(), current_state.pos()) < MIN_DISTANCE_FOR_YAW_CONTROL*MIN_DISTANCE_FOR_YAW_CONTROL) {
				// Do not auto control yaw if too close to the target
				target.setAutoYaw(false);
				target.pos().w = current_state.pos().w;
			}

			t_timeout = DEFAULT_TIMEOUT + (t_planned_yaw < t_planned_xyz ? t_planned_xyz  : t_planned_yaw) ;

			if(Double.isInfinite(xyzExecutor.getCost()) && Double.isInfinite(yawExecutor.getCost())) 
				return null;
			else 
				return target;
		}




		private void reset() {

			yawExecutor.reset();
			xyzExecutor.reset();
			yawControl.reset();

			//acceptance_radius = RADIUS_ACCEPT;
			acceptance_yaw    = YAW_ACCEPT;
			t_timeout         = DEFAULT_TIMEOUT;
			t_section_elapsed_last    = 0;
			t_planned_yaw     = 0;
			t_planned_xyz     = 0;

		}

		private float normAngle(float a) {
			return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5f) / (2*(float)Math.PI));
		}



		private boolean isFinite(GeoTuple4D_F32<?> p) {
			return Float.isFinite(p.x) && Float.isFinite(p.y) && Float.isFinite(p.z);
		}

		private boolean isValid(GeoTuple4D_F32<?> p) {
			return p.x != 0 || p.y != 0 || p.z != 0;
		}

		private void enableOffboard() {

			control.sendMAVLinkCmd(MAV_CMD.MAV_CMD_DO_SET_MODE, (cmd, result) -> {
				if(result != MAV_RESULT.MAV_RESULT_ACCEPTED) {
					worker.stopAndLoiter();
					control.writeLogMessage(new LogMessage("[msp] Switching to offboard failed ("+result+").", MAV_SEVERITY.MAV_SEVERITY_WARNING));
					offboardEnabled = false;

				} else {
					offboardEnabled = true;;
				}
			}, MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED,
					MAV_CUST_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0 );
		}


		private void updateTrajectoryModel(float elapsed_time) {

			if(!model.slam.isFlag(Slam.OFFBOARD_FLAG_XYZ_PLANNER)) {
				model.traj.clear();
				// Clear trajectory in MAVGCL
				control.sendMAVLinkMessage(new msg_msp_trajectory(2,1));
				return;
			}

			model.traj.ls = xyzExecutor.getTotalTime();
			model.traj.fs = elapsed_time;
			model.traj.ax = (float)xyzExecutor.getAxisParamAlpha(0);
			model.traj.ay = (float)xyzExecutor.getAxisParamAlpha(1);
			model.traj.az = (float)xyzExecutor.getAxisParamAlpha(2);


			model.traj.bx = (float)xyzExecutor.getAxisParamBeta(0);
			model.traj.by = (float)xyzExecutor.getAxisParamBeta(1);
			model.traj.bz = (float)xyzExecutor.getAxisParamBeta(2);


			model.traj.gx = (float)xyzExecutor.getAxisParamGamma(0);
			model.traj.gy = (float)xyzExecutor.getAxisParamGamma(1);
			model.traj.gz = (float)xyzExecutor.getAxisParamGamma(2);

			model.traj.sx = (float)xyzExecutor.getInitialPosition(0);
			model.traj.sy = (float)xyzExecutor.getInitialPosition(1);
			model.traj.sz = (float)xyzExecutor.getInitialPosition(2);

			model.traj.svx = (float)xyzExecutor.getInitialVelocity(0);
			model.traj.svy = (float)xyzExecutor.getInitialVelocity(1);
			model.traj.svz = (float)xyzExecutor.getInitialVelocity(2);

			model.traj.sax = (float)xyzExecutor.getInitialAcceleration(0);
			model.traj.say = (float)xyzExecutor.getInitialAcceleration(1);
			model.traj.saz = (float)xyzExecutor.getInitialAcceleration(2);

			model.traj.tms = DataModel.getSynchronizedPX4Time_us();

		}
	}
}
