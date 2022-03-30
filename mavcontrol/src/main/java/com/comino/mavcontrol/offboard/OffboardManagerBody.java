package com.comino.mavcontrol.offboard;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.struct.image.GrayU16;
import georegression.geometry.ConvertRotation3D_F32;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector4D_F32;
import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;

public class OffboardManagerBody {
	
	private static final int  UPDATE_RATE                 			= 50;		
	
	private final MSPLogger 			    logger;
	private final DataModel 			    model;
	private final IMAVController            control;
	private final OffboardWorker            worker;
	
	private float acceptance_radius	        = 0.1f;
	private boolean check_acceptance_radius = true;

	
	private final DMatrixRMaj               to_body = CommonOps_DDRM.identity(3);
	private final DMatrixRMaj               tmp     = CommonOps_DDRM.identity(3);
	
    private final BlockingQueue<State> target_state_queue = new ArrayBlockingQueue<State>(10);
    
    private final WorkQueue wq = WorkQueue.getInstance();
    private int offboard_worker_id;
	
	
	public OffboardManagerBody(IMAVController control, PX4Parameters params) {
		
		this.control        = control;
		this.model          = control.getCurrentModel();
		this.logger         = MSPLogger.getInstance();
		this.worker         = new OffboardWorker();
	}
	
	public void start() {
		offboard_worker_id = wq.addCyclicTask("NP", UPDATE_RATE, worker);
	}
	
	public void stop() {
		wq.removeTask("NP", offboard_worker_id);
	}
	
		
	public void setTargetState(Vector4D_F64 pos_ned, Vector4D_F64 speed_ned) throws InterruptedException {
		State target_body = new State();
		target_body.setToAndRotate(pos_ned,speed_ned,to_body);
		target_state_queue.put(target_body);
	}
	
	private class OffboardWorker implements Runnable {
		
		private final State current        = new State();
		private final State current_ned    = new State();
		
		private State current_target_state;

		@Override
		public void run() {
			
			// Model updates
			updateCurrentStateFromModel(model);
			
			// TODO Safety checks
			
			if(current_target_state!= null && current_target_state.isValid()) {
				
				if((MSP3DUtils.distance3D(current.position(), current_target_state.position()) < acceptance_radius && check_acceptance_radius )) {
					
					// TODO: Perform target reached action
					
					current_target_state = null;
					return;
				}
				
			
				// TODO: Calculate new setpoint from trajetcory
				
				// TOOD: Send to PX4 in body frame	
				
			} else {
				
				if(!target_state_queue.isEmpty()) {
					try {
						current_target_state = target_state_queue.take();
						
						// TODO: generate trajectory in body frame
						
						return;
					} catch (InterruptedException e) { current_target_state = null; }
				}
				
				// TODO: Switch to loiter mode
				
			}
		}
		
		private void updateCurrentStateFromModel(DataModel model) {
			ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, model.attitude.r, model.attitude.p, model.attitude.y, tmp);
			CommonOps_DDRM.transpose(tmp,to_body);
			MSP3DUtils.convertCurrentState(model, current_ned.position(), current_ned.speed());
			current.setToAndRotate(current_ned, to_body);
		}
		
	}
	
	private class State {
		
		private final Vector4D_F64		        p = new Vector4D_F64();
		private final Vector4D_F64		        s = new Vector4D_F64();	
		
		private long    tms                       = 0;
		
		public State() {
			this.p.setTo(Double.NaN,Double.NaN,Double.NaN,Double.NaN);
			this.s.setTo(Double.NaN,Double.NaN,Double.NaN,Double.NaN);
			this.tms = System.currentTimeMillis();
		}
		
		public State(Vector4D_F64 p, Vector4D_F64 s) {
			this.p.setTo(p);
			this.s.setTo(s);
			this.tms = System.currentTimeMillis();
		}
		
		public Vector4D_F64 position() {
			return p;
		}
		
		public Vector4D_F64 speed() {
			return s;
		}
		
		public long elapsed_ms() {
			return System.currentTimeMillis() - tms;
		}
		
		public boolean isValid() {
			return MSP3DUtils.isValid(p) && MSP3DUtils.isValid(s);	
		}
		
		public void setTo(Vector4D_F64 pos, Vector4D_F64 speed) {
			p.setTo(pos);
			s.setTo(speed);
			this.tms = System.currentTimeMillis();
		}
		
		public void setToAndRotate(Vector4D_F64 pos, Vector4D_F64 speed, DMatrixRMaj rotation) {
			GeometryMath_F64.mult(rotation, pos, p);
			GeometryMath_F64.mult(rotation, speed, s);
			this.tms = System.currentTimeMillis();
		}
		
		public void setToAndRotate(State state, DMatrixRMaj rotation) {
			GeometryMath_F64.mult(rotation, state.p, p);
			GeometryMath_F64.mult(rotation, state.s, s);
			this.tms = System.currentTimeMillis();
		}
	}
	
	
	

}
