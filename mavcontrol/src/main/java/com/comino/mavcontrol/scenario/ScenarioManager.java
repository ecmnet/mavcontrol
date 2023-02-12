package com.comino.mavcontrol.scenario;

import java.util.Collection;
import java.util.LinkedList;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.offboard3.Offboard3Manager;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;

public class ScenarioManager {

	private final LinkedList<AbstractScenarioItem> itemList = new LinkedList<AbstractScenarioItem>();

	private static ScenarioManager instance;

	private final  ScenarioWorker    	scenarioWorker         = new ScenarioWorker();
	private final  IMAVController 		control;
	private final  Offboard3Manager   	offboard;
	private        Thread         		scenarioWorkerThread;
	private final  Status         		status;

	private boolean isRunning = false;

	public static ScenarioManager getInstance(IMAVController control) {
		if(instance == null)
			instance = new ScenarioManager(control);
		return instance;
	}

	private ScenarioManager(IMAVController control) {
		this.control = control;
		this.status  = control.getCurrentModel().sys;

		this.offboard = Offboard3Manager.getInstance(control);
		this.offboard.setTimeoutAction(() -> {
			abort();
		});
	}
	
	public void setMaxVelocity(float max_velocity_ms) {
		this.offboard.setMaxVelocity(max_velocity_ms);
	}

	public void addItem(AbstractScenarioItem item) {
		itemList.add(item);
	}

	public void addItems(Collection<AbstractScenarioItem> items) {
		if(items!=null)
			itemList.addAll(items);
	}

	public void start() {

		if(isRunning) {
			control.writeLogMessage(new LogMessage("[msp] Scenario already in progress.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			itemList.clear();
			return;
		}

		if(itemList.size() > 0) {

			control.writeLogMessage(new LogMessage("[msp] Execute scenario with "+itemList.size()+" items.", MAV_SEVERITY.MAV_SEVERITY_INFO));

			for(AbstractScenarioItem item : itemList) {
				item.initialize();
			}

			scenarioWorkerThread = new Thread(scenarioWorker);
			scenarioWorkerThread.setName("ScenarioWorker");
			scenarioWorkerThread.start();
		}
	}

	public void abort() {
		scenarioWorker.abortRequest();
		itemList.clear();
	}

	private class ScenarioWorker implements Runnable {

		private AbstractScenarioItem currentItem;
		private boolean              abortRequest;
		private int                  step_counter;

		@Override
		public void run() {

			long tms;

			isRunning = true;

			this.step_counter  = 0;

			while(itemList.size()>0 && isRunning) {



				control.getCurrentModel().slam.wpcount = ++step_counter;
				currentItem = itemList.poll();
				currentItem.setOwner(this);

				currentItem.execute();

				synchronized(this) {

					try {

						tms = System.currentTimeMillis()+currentItem.getTimeout_ms();
						while(!currentItem.isCompleted() && System.currentTimeMillis() < tms && !currentItem.isAborted() && !abortRequest) {
							wait(currentItem.getTimeout_ms());
						}

					} catch (InterruptedException e) { }

				}

				if(currentItem.isAborted() || abortRequest) {
					control.writeLogMessage(new LogMessage("[msp] Scenario aborted in step "+step_counter,
							MAV_SEVERITY.MAV_SEVERITY_ERROR));
					isRunning = false;
					break;
				}

				if(!currentItem.isCompleted()) {
					control.writeLogMessage(new LogMessage("[msp] Scenario Timeout occurred in step "+step_counter,
							MAV_SEVERITY.MAV_SEVERITY_ERROR));
					isRunning  = false;
					break;
				}
			}


			abortRequest = false;
			isRunning    = false;

			if(itemList.isEmpty())
				control.writeLogMessage(new LogMessage("[msp] Scenario execution completed.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			else
				itemList.clear();

			control.getCurrentModel().slam.wpcount = 0;

		}

		public synchronized void abortRequest() {
			this.abortRequest= true;
			this.notify();
		}
	}
}
