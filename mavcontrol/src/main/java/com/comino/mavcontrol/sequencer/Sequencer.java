package com.comino.mavcontrol.sequencer;

import java.util.LinkedList;
import java.util.concurrent.Future;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_ACTION;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.offboard.OffboardManager;
import com.comino.mavcontrol.struct.SeqItem;
import com.comino.mavutils.legacy.ExecutorService;

public class Sequencer {
	
	private OffboardManager          offboard = null;
	private MSPLogger 				 logger	  = null;
	private DataModel 				 model	  = null;
	private IMAVController         	 control  = null;
	
	protected LinkedList<SeqItem>    sequence = null;
	protected LinkedList<SeqItem>    appended = null;
	
	
	private Future<?> future;

	
	public Sequencer(OffboardManager offboard, MSPLogger logger, DataModel model, IMAVController control) {
		
		this.offboard = offboard;
		this.logger = logger;
		this.model = model;
		this.control = control;
		
		this.sequence = new LinkedList<SeqItem>();
		this.appended = new LinkedList<SeqItem>();
		
	}
	
	public void add(SeqItem item) {
		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
			sequence.add(item);
		else
			appended.add(item);
	}
	
	public void clear() {
		if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			sequence.clear(); appended.clear();
			model.slam.wpcount = 0;
		}
	}

	public void abort() {
		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			sequence.clear();  appended.clear();
			model.slam.wpcount = 0;
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
			offboard.abort();
			while(!future.isDone());
		}
	}
	
	public void execute(SeqItem item, ISeqAction completedAction) {
		sequence.clear();  appended.clear();
		sequence.add(item);
		execute();
	}

	public void execute() {
		execute((ISeqAction)null);
	}

	public void execute(ISeqAction completedAction) {


		if(!offboard.isEnabled()) {
			logger.writeLocalMsg("[msp] Offboard not enabled.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
			sequence.clear();
			return;
		}

		if(model.sys.isStatus(Status.MSP_LANDED)) {
			control.writeLogMessage(new LogMessage("[msp] Not executed. On ground/No offboard.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
			sequence.clear();  appended.clear();
			return;
		}

		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
			control.writeLogMessage(new LogMessage("[msp] Sequence already in execution.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			return;
		}
		if(sequence.isEmpty()) {
			control.writeLogMessage(new LogMessage("[msp] No valid sequence.", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
			return;
		}

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, true);

		future = ExecutorService.get().submit(() -> {
			int i=0;

			try { Thread.sleep(50); } catch (InterruptedException e) { }

			//		final ListIterator<SeqItem> i = sequence.listIterator();
			while(!sequence.isEmpty() && model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE)) {
				model.slam.wpcount  = ++i;
				control.writeLogMessage(new LogMessage("[msp] Step "+i+ " executed.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				SeqItem item = sequence.poll();
				if(item.hasTarget()) {
					if(item.getControlListener()!=null)
						offboard.registerSpeedControl(item.getControlListener());
					offboard.setTarget(item.getTarget(model));
					if(!offboard.start_wait(OffboardManager.MODE_TRAJECTORY, item.getTimeout_ms())) {
						model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
						control.writeLogMessage(new LogMessage("[msp] Sequence timeout occurred.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
						break;
					}
					if(!offboard.isEnabled()) {
						model.slam.wpcount = 0;
						sequence.clear();
						control.writeLogMessage(new LogMessage("[msp] Sequence aborted.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
						return;
					}
				}

				// Execute action
				if(!item.executeAction()) {
					control.writeLogMessage(new LogMessage("[msp] Sequence action request abort.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					break;
				}
				// Extend sequence by appended items
				if(!appended.isEmpty()) {
					control.writeLogMessage(new LogMessage("[msp] "+appended.size()+" added to sequence", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					appended.forEach((n) -> { sequence.add(n); });
					appended.clear();
				}
			}
			offboard.setTarget(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
			offboard.start(OffboardManager.MODE_LOITER);

			model.slam.wpcount = 0;
			sequence.clear();

			if(completedAction!=null && model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
				completedAction.execute();

			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE))
				control.writeLogMessage(new LogMessage("[msp] Sequence finished.", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			else
				control.writeLogMessage(new LogMessage("[msp] Sequence aborted.", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			model.sys.setAutopilotMode(MSP_AUTOCONTROL_ACTION.WAYPOINT_MODE, false);
		});
	}
	
}
