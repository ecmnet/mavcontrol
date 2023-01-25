package com.comino.mavcontrol.scenario.parser;

import java.util.LinkedList;

import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;

public class Scenario {

	public static final int TYPE_DEFAULT = 0;
	public static final int TYPE_SITL    = 1;

	private LinkedList<AbstractScenarioItem> list;
	private String  name = "Default";
	private int     type = TYPE_DEFAULT;
	
	private float   time_factor = 1;
	private float   max_speed   = 1;
	private int     repeats     = 1;


	public Scenario() {
		super();
	}

	public void setList(final LinkedList<AbstractScenarioItem> list) {
		this.list = list;
	}

	public boolean hasItems() {
		return list.size() > 0;
	}

	public boolean isSITL() {
		return type == TYPE_SITL;
	}

	public String getName() {
		return name;
	}

	public int getRepeats() {
		return repeats;
	}

	public void setRepeats(int repeats) {
		this.repeats = repeats;
	}

	public void setName(String name) {
		this.name = name;
	}

	public void setType(int type) {
		this.type = type;
	}

	public float getTimeFactor() {
		return time_factor;
	}

	public void setTimeFactor(float time_factor) {
		this.time_factor = time_factor;
	}

	public float getMaxSpeed() {
		return max_speed;
	}

	public void setMaxSpeed(float max_speed) {
		this.max_speed = max_speed;
	}

	public LinkedList<AbstractScenarioItem> getList() {
		return list;
	}

	public String toString() {
		if(isSITL())
			return repeats+"x "+name+" with "+list.size()+" items (SITL)";
		else
			return repeats+"x "+name+" with "+list.size()+" items";
	}

}
