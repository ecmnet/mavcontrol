package com.comino.mavcontrol.scenario;

import java.util.Collection;
import java.util.LinkedList;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;

public class ScenarioManager {

	private final LinkedList<AbstractScenarioItem> itemList = new LinkedList<AbstractScenarioItem>();

	private final  ScenarioWorker scenarioWorker         = new ScenarioWorker();
	private final  IMAVController control;
	private        Thread         scenarioWorkerThread;
	private final  Status         status;

	private boolean isRunning = false;


	public ScenarioManager(IMAVController control) {
		this.control = control;
		this.status  = control.getCurrentModel().sys;
	}

	public void addItem(AbstractScenarioItem item) {
		itemList.add(item);
	}

	public void addItems(Collection<AbstractScenarioItem> items) {
		if(items!=null)
			itemList.addAll(items);
	}

	public void start() {
		start(1);	
	}

	public void start(int repeats) {

		if(isRunning) {
			control.writeLogMessage(new LogMessage("[msp] Scenario already in progress.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			itemList.clear();
			return;
		}

		scenarioWorker.setRepeats(repeats);

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
	}

	private class ScenarioWorker implements Runnable {

		private AbstractScenarioItem currentItem;
		private boolean              abort_request;
		private int                  step_counter;

		private int                  repeats = 1;

		public void setRepeats(int repeats) {
			this.repeats = repeats;
		}

		@Override
		public void run() {

			long tms;

			isRunning = true;


			this.abort_request = false;
			this.step_counter  = 0;

			while(itemList.size()>0 && !abort_request) {

				synchronized(this) {

					control.getCurrentModel().slam.wpcount = ++step_counter;
					currentItem = itemList.poll();
					currentItem.setOwner(this);
					currentItem.execute();

					try {
						tms = System.currentTimeMillis()+currentItem.getTimeout_ms();
						while(!currentItem.isCompleted() && System.currentTimeMillis() < tms && !currentItem.isAborted()) {
							wait(currentItem.getTimeout_ms());
						}

						if(currentItem.isAborted()) {
							itemList.clear();
							control.writeLogMessage(new LogMessage("[msp] Scenario aborted in step "+step_counter,
									MAV_SEVERITY.MAV_SEVERITY_ERROR));
							return;
						}

						if(!currentItem.isCompleted()) {
							itemList.clear();
							control.writeLogMessage(new LogMessage("[msp] Scenario Timeout occurred in step "+step_counter,
									MAV_SEVERITY.MAV_SEVERITY_ERROR));
							return;
						}

					} catch (InterruptedException e) { }

				}
			}



			itemList.clear();

			isRunning = false;
			control.writeLogMessage(new LogMessage("[msp] Scenario execution completed.", MAV_SEVERITY.MAV_SEVERITY_INFO));
			control.getCurrentModel().slam.wpcount = 0;
		}

		public synchronized void abortRequest() {
			this.abort_request= true;
			this.notify();
		}
	}
}
