package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class TestItem extends AbstractScenarioItem {

	private final GeoTuple4D_F32<?> target_position = new Vector4D_F32();
	private final int step;

	public TestItem(int step) {
		super(AbstractScenarioItem.TYPE_TEST,null);
		this.step = step;
	}

	public void setPositionLocal(float x, float y, float z, float w) {
       target_position.setTo(x,y,z,w);
	}

	@Override
	public void execute() {
		System.out.println("TestItem "+step+" with position "+target_position);
		try { Thread.sleep(1000);} catch (InterruptedException e) { }
		completed();
	}

	@Override
	public void initialize() {
       System.out.println("TestItem "+this.step+" initialized");
	}

}
