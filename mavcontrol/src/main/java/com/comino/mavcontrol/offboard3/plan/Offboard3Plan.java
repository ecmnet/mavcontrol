package com.comino.mavcontrol.offboard3.plan;

import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;

public class Offboard3Plan<T> extends LinkedList<T> {

	private static final long serialVersionUID = -2180412674433672222L;

	private float estimated_time = 0;
	private float total_time     = 0;
	private float total_costs    = 0;


	@Override
	public boolean add(T e) {
		return super.add(e);
	}

	@Override
	public void clear() {
		super.clear();
		total_time     = 0;
		total_costs    = 0;
		estimated_time = 0;
	}
	
	public void replaceWith(int index, @SuppressWarnings("unchecked") T... sections) {
		replaceWith(index,Arrays.asList(sections));
	}

	public void replaceWith(int index, Collection<? extends T> sections) {
		if(index >= this.size())
			return;
		super.remove(index);
		super.addAll(index,sections);
	}
	

	public void setEstimatedTime(float t) {
		estimated_time = t;
	}

	public void addCostsAndTime(float costs, float time) {
		this.total_costs += costs;
		this.total_time  += time;
	}

	public float getTotalTime() {
		return total_time;
	}
	
	public float getEstimatedTime() {
		return estimated_time;
	}


	public float getTotalCosts() {
		return total_costs;
	}

	public String toString() {
		final StringBuilder b = new StringBuilder();
		int index = 0;
		b.append("\n");
		b.append("Plan ===================================================================================================================================\n");
		for(T s : this) {
			b.append(++index); b.append(".:"); b.append(s); b.append("\n");
		}
		b.append("Total costs    : "); b.append(total_costs);    b.append("\n");
		b.append("Estimated time : "); b.append(estimated_time); b.append("\n");
		b.append("Planned   time : "); b.append(total_time);     b.append("\n");

		b.append("========================================================================================================================================\n");
		return b.toString();
	}


}
