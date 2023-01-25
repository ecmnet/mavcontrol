package com.comino.mavcontrol.scenario.parser;

import java.util.LinkedList;

import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;

public class Scenario {

	public static final int TYPE_DEFAULT = 0;
	public static final int TYPE_SITL    = 1;

	private LinkedList<AbstractScenarioItem> list;
	private String  name = "Default";
	private int     type = TYPE_DEFAULT;


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

	public void setName(String name) {
		this.name = name;
	}

	public void setType(int type) {
		this.type = type;
	}

	public LinkedList<AbstractScenarioItem> getList() {
		return list;
	}

	public String toString() {
		if(isSITL())
			return name+" with "+list.size()+" items (SITL)";
		else
			return name+" with "+list.size()+" items";
	}

}
