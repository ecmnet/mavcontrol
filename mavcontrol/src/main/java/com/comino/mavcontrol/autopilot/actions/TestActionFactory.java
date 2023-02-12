package com.comino.mavcontrol.autopilot.actions;

import java.util.LinkedList;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.messaging.MessageBus;
import com.comino.mavcom.messaging.msgs.msp_msg_nn_object;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavcontrol.offboard2.Offboard2Manager;
import com.comino.mavcontrol.offboard3.Offboard3Manager;
import com.comino.mavcontrol.offboard3.plan.Offboard3Plan;
import com.comino.mavcontrol.offboard3.target.Offboard3VelAccTarget;
import com.comino.mavcontrol.scenario.ScenarioManager;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;
import com.comino.mavcontrol.scenario.items.MoveToItem;
import com.comino.mavcontrol.scenario.items.ObstacleItem;
import com.comino.mavcontrol.scenario.items.PrecisionLandItem;
import com.comino.mavcontrol.scenario.items.TakeOffItem;
import com.comino.mavcontrol.scenario.parser.Scenario;
import com.comino.mavcontrol.scenario.parser.ScenarioReader;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Vector4D_F32;

public class TestActionFactory {

	final static msg_msp_vision msg = new msg_msp_vision(2,1);

	public static void simulateFiducial(IMAVController control, float radius) {

		//		final DataModel model = control.getCurrentModel();
		//
		//		if(model.sys.isStatus(Status.MSP_LANDED)) {
		//			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
		//			msg.px    =  Float.NaN;
		//			msg.py    =  Float.NaN;
		//			msg.pz    =  Float.NaN;
		//			msg.pw    =  Float.NaN;
		//			msg.flags =  model.vision.flags;
		//			control.sendMAVLinkMessage(msg);
		//			return;
		//		}
		//
		//		if(model.hud.ar > 2.5f) {
		//			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
		//			msg.px    =  Float.NaN;
		//			msg.py    =  Float.NaN;
		//			msg.pz    =  Float.NaN;
		//			msg.pw    =  Float.NaN;
		//			msg.flags =  model.vision.flags;
		//			control.sendMAVLinkMessage(msg);
		//			return;
		//		}
		//
		//		if(!model.vision.isStatus(Vision.FIDUCIAL_LOCKED) && model.sys.isStatus(Status.MSP_LPOS_VALID) && 
		//				!model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_TAKEOFF) ) {
		//
		//			//			if(control.isSimulation())	
		//			//				TestActionFactory.setRandomObstacle();
		//
		//			model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
		//			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, true);
		//
		//			model.vision.px = model.state.l_x + ((float)Math.random()-0.5f)*radius;
		//			model.vision.py = model.state.l_y + ((float)Math.random()-0.5f)*radius;
		//			model.vision.pz = 0f;	
		//			model.vision.pw = ((float)Math.random()-0.5f)*12f;
		//
		//			//model.vision.pw = MSPMathUtils.toRad(135);
		//
		//			System.out.println("Simulated fiducial rotation: "+MSPMathUtils.fromRad(model.vision.pw));
		//
		//			msg.px    =  model.vision.px;
		//			msg.py    =  model.vision.py;
		//			msg.pz    =  model.vision.pz;
		//			msg.pw    =  model.vision.pw;
		//			msg.flags =  model.vision.flags;
		//			control.sendMAVLinkMessage(msg);
		//		} 

	}
	
	
	public static void test_circle(IMAVController control,boolean enable) {
		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		offboard.circle(1, 1, Float.NaN, Float.NaN, 2.0f, MSPMathUtils.toRad(90), null);
			
	}


	static int worker = 0; static float sign = 1.0f;
	static msp_msg_nn_object person = new msp_msg_nn_object();

	public static void test_simulate_person(boolean enable) {

		final WorkQueue  wq = WorkQueue.getInstance();
		final MessageBus bus = MessageBus.getInstance();

		if(enable) {

			if(!MSP3DUtils.isFinite(person.position))
				person.position.setTo(-2, 0, -1.5);

			worker = wq.addCyclicTask("LP", 100, () -> {

				person.object_id = 0;
				person.tms = System.currentTimeMillis();

				if(person.position.y > 4) 
					sign = -1.0f;
				if(person.position.y < -4) 
					sign = 1.0f;
				person.position.y += ( 0.002 * sign);

				bus.publish(person);

			});
		} else {

			person.tms = 0;
			bus.publish(person);

			wq.removeTask("LP", worker);
		}
	}

	public static void turnTest() {
		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		if(offboard!=null) {
			offboard.rotate((float)(Math.random()*2*Math.PI),null);
		}
	}
	
	public static void accTest() {
		
		final Offboard3Manager offboard = Offboard3Manager.getInstance();
		final Offboard3Plan plan = new Offboard3Plan();
		
		//final Vector4D_F32 v = new Vector4D_F32(Float.NaN,Float.NaN,Float.NaN,Float.NaN);
		final Vector4D_F32 a = new Vector4D_F32(Float.NaN,Float.NaN,Float.NaN,Float.NaN);
		
		float v = 1; float r = 1.5f;
		float o = (float)(Math.PI * 2) / 12f;
		float time = 0.5f;
		
		for(int i = 0; i <= 24; i++) {
			a.x = (o*i*(float)Math.sin(o*i) - (float)Math.cos(o*i)) ;
			a.y = (o*i*(float)Math.cos(o*i) + (float)Math.sin(o*i)) ;
			
			plan.add(new Offboard3VelAccTarget(a,time));	
		}
		
		offboard.executePlan(plan, null);
		
	}



	public static void setRandomObstacle() {

		final DataModel m  = AutoPilotBase.getInstance().getControl().getCurrentModel();

		m.slam.ox = (float)(Math.random()*3 -1.5);
		m.slam.oy = (float)(Math.random()*3 -1.5);
		m.slam.oz = -1.2f;

	}

	static int worker2 = 0; 
	static int skip = -1;
	public static void continuous_planning(DataModel model,boolean enable) {

		final WorkQueue  wq = WorkQueue.getInstance();
		final Offboard3Manager offboard = Offboard3Manager.getInstance();

		if(enable && ( model.sys.isNavState(Status.NAVIGATION_STATE_AUTO_LOITER) || model.sys.isNavState(Status.NAVIGATION_STATE_OFFBOARD))) {

			if(Float.isFinite(model.slam.ox)) {
				worker2 = wq.addCyclicTask("LP", 10000, () -> {

					// Obstacle 
					offboard.moveTo((float)(Math.random()*2f-1f+0.8f)*skip + model.slam.ox, model.slam.oy,
							(float)Math.random()*0.5f-1.5f, Float.NaN);
					skip = skip * -1;

				});
			} else {
				worker2 = wq.addCyclicTask("LP", 5000, () -> {
					offboard.moveTo((float)Math.random()*6f-3f, (float)Math.random()*6f-3f, 
							(float)Math.random()*3-3f-0.5f, Float.NaN);
				});

			}

		} else {

			wq.removeTask("LP", worker2);
		}
	}

}
