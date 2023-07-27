package com.comino.mavcontrol.scenario.items;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.param.PX4Parameters;
import com.comino.mavcom.param.ParameterAttributes;
import com.comino.mavcontrol.IOffboardControl;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Vector4D_F32;

public class MoveHomeItem extends AbstractScenarioItem {
	
	private static float MAX_ALTITUDE_FOR_PRECLAND = - 2.0f;   // As visibility of the fiducial is limited, limit the target altitude of the approach

	private float acceptance_radius_m = Float.NaN;
	private GeoTuple4D_F32<?> position = new Vector4D_F32();


	public MoveHomeItem(IMAVController control,IOffboardControl offboard) {
		super(control,offboard);
		final PX4Parameters params = PX4Parameters.getInstance();
		ParameterAttributes takeoff_alt_param = params.getParam("MIS_TAKEOFF_ALT");
		position.setTo(0, 0, -(float)takeoff_alt_param.value, Float.NaN );
	}

	public void setAcceptanceRadius(float acceptance_radius_m) {
		this.acceptance_radius_m = acceptance_radius_m;
	}
	
	

	@Override
	public long getTimeout_ms() {
		return (long)(Math.sqrt(position.x*position.x + position.y*position.y)/max_xyz_velocity)*5000L
		       + super.getTimeout_ms();
	}

	@Override
	public void execute() {
		
		    if(position.z < MAX_ALTITUDE_FOR_PRECLAND)
		    	position.z = MAX_ALTITUDE_FOR_PRECLAND;
		    
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
