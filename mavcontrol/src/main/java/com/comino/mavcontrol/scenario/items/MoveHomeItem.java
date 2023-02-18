package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcontrol.autopilot.AutoPilotBase;
import com.comino.mavutils.MSPMathUtils;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class MoveHomeItem extends AbstractScenarioItem {

	private float acceptance_radius_m = Float.NaN;


	public MoveHomeItem(IMAVController control) {
		super(control);
	}

	public void setAcceptanceRadius(float acceptance_radius_m) {
		this.acceptance_radius_m = acceptance_radius_m;
	}
	
	

	@Override
	public long getTimeout_ms() {
		GeoTuple4D_F32<?> position = AutoPilotBase.getInstance().getTakoffPosition();
		return (long)(Math.sqrt(position.x*position.x + position.y*position.y)/max_xyz_velocity)*5000L
		       + super.getTimeout_ms();
	}

	@Override
	public void execute() {

		    GeoTuple4D_F32<?> position = AutoPilotBase.getInstance().getTakoffPosition();
		    
		    if(position.z> -2.0) {
		    	System.err.println("Z should be takeoff altitude but is "+position.z);
		    	position.z = -2.0f;
		    }
		    
			if(Float.isFinite(acceptance_radius_m))
				offboard.moveTo(position.x, 
						position.y, 
						position.z, 
						position.w, 
						(m) -> completed(),acceptance_radius_m);
			else
				offboard.moveTo(position.x, 
						position.y, 
						position.z, 
						position.w, 
						(m) -> completed());


	}


}
