package com.comino.mavcontrol.scenario;

import java.util.Collection;
import java.util.LinkedList;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;
import com.comino.mavcontrol.scenario.items.TestItem;

public class ScenarioManager {

	private final LinkedList<AbstractScenarioItem> itemList = new LinkedList<AbstractScenarioItem>();
	
	private final  ScenarioWorker scenarioWorker         = new ScenarioWorker();
	private        Thread         scenarioWorkerThread;


	public ScenarioManager(IMAVController control) {

	}
	
	public void addItem(AbstractScenarioItem item) {
		itemList.add(item);
	}
	
	public void addItem(Collection<AbstractScenarioItem> items) {
		itemList.addAll(items);
	}

	public void start() {

		if(itemList.size() > 0) {

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

		@Override
		public void run() {
			
			this.abort_request = false;
			this.step_counter  = 0;

			while(itemList.size()>0 && !abort_request) {
				step_counter++;
				currentItem = itemList.poll();
				currentItem.execute();
			}
		}
		
		public void abortRequest() {
			this.abort_request= true;
		}
	}
	
	
	public static void main(String[] args) {
		
		ScenarioManager s = new ScenarioManager(null);
		
		s.addItem(new TestItem(1));
		s.addItem(new TestItem(2));
		s.addItem(new TestItem(3));
		s.addItem(new TestItem(4));
		
		s.start();
		
	}


}
