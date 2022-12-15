package com.comino.mavcontrol.offboard3.plan;

import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.offboard3.target.Offboard3AbstractTarget;

public class Offboard3Plan extends LinkedList<Offboard3AbstractTarget> implements Comparable<Offboard3Plan>  {

	private static final long serialVersionUID = -2180412674433672222L;

	private float estimated_time = 0;
	private float total_time     = 0;
	private float total_costs    = 0;
	

	@Override
	public boolean add(final Offboard3AbstractTarget e) {
		return super.add(e);
	}

	@Override
	public void clear() {
		super.clear();
		total_time     = 0;
		total_costs    = 0;
		estimated_time = 0;
	}


	public void set(final Offboard3Plan plan) {
		this.clear(); this.addAll(plan);
		total_time      = plan.total_time;
		total_costs     = plan.total_costs;
		estimated_time  = plan.estimated_time;
	}

	public void replace(int index, @SuppressWarnings("unchecked") Offboard3AbstractTarget... sections) {
		replace(index,Arrays.asList(sections));
	}

	public void replace(int index, Collection<? extends Offboard3AbstractTarget> sections) {
		if(index > this.size())
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

	public float getTotalTimeUpTo(int section) {
		float time = 0; 
		for(int i = 0; i<section;i++) 
			time += this.get(i).getPlannedSectionTime();
		return time;
	}

	public float getEstimatedTime() {
		return estimated_time;
	}

	public float getTotalCosts() {
		return total_costs;
	}

	public Offboard3Plan clone() {
		Offboard3Plan n = new Offboard3Plan();
		n.estimated_time = estimated_time;
		n.total_costs    = total_costs;
		n.total_time     = total_time;
		n.addAll(this);	
		return n;
	}

	public String toString() {
		final StringBuilder b = new StringBuilder();
		int index = 0;
		b.append("\n");
		b.append("Plan "+DataModel.getSynchronizedPX4Time_us()+" ===================================================================================================================================\n");
		for(Offboard3AbstractTarget s : this) {
			b.append(++index); b.append(".:"); b.append(s); b.append("\n");
		}
		b.append("Total costs    : "); b.append(total_costs);    b.append("\n");
		b.append("Estimated time : "); b.append(estimated_time); b.append("\n");
		b.append("Planned   time : "); b.append(total_time);     b.append("\n");

		b.append("=========================================================================================================================================================\n");
		return b.toString();
	}

	@Override
	public int compareTo(Offboard3Plan o) {
		return Float.compare(this.total_costs, o.getTotalCosts());
	}
	
	


}
