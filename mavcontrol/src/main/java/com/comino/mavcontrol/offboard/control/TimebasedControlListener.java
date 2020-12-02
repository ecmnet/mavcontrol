package com.comino.mavcontrol.offboard.control;

import com.comino.mavcom.struct.Polar3D_F32;
import com.comino.mavcontrol.offboard.IExtSpeedControl;
import com.comino.mavmap.libs.TrajMathLib;
import com.comino.mavutils.MSPMathUtils;

public class TimebasedControlListener implements IExtSpeedControl {

	private static final float MAX_ACCELERATION		                = 0.5f;                   // Max acceleration in m/s2
	private static final float MAX_SPEED					        = 1.00f;          	      // Default Max speed in m/s
	private static final float MIN_SPEED					        = 0.10f;          	      // Default Min speed in m/s


	private float[] timestamps = null;
	private int     phase      = 0;
	private float   phase_time = 0;
	private float   delta_time = 0;
	private float   ela_phase  = 0;
	private float   a          = 0;
	private float   a_phase    = 0;


	public boolean determineSpeedAnDirection(float delta_sec, float ela_sec, float eta_sec, Polar3D_F32 spd, Polar3D_F32 path, Polar3D_F32 ctl) {

		ctl.angle_xy =  path.angle_xy;
		ctl.angle_xz =  path.angle_xz;

		if(eta_sec < 0.1f)
			return true;

       if(ela_sec > timestamps[phase] + ela_phase && phase < timestamps.length-1) {
    	   ela_phase = ela_sec;
    	   phase++;
    	   a_phase = a;
    	   System.out.println(phase+":"+phase_time);

       }

       delta_time = ela_sec - ela_phase;

       if(phase < 1 )
    	   return true;

       switch(phase) {
       case 1:
    	   ctl.value = ctl.value + delta_sec * a_phase + 0.5f * ( delta_sec * delta_sec ) * 1;
    	   a = a + delta_sec  * 1;
    	   break;
       case 2:
    	   ctl.value = ctl.value + delta_sec * a_phase;
    	   break;
       case 3:
    	   ctl.value = ctl.value + delta_sec * a_phase + 0.5f * ( delta_sec * delta_sec ) * -1;
    	   a = a + delta_sec  * - 1;
    	   break;
       case 4:
    	   ctl.value = ctl.value + delta_sec * a_phase;
    	   break;
       case 5:
    	   ctl.value = ctl.value + delta_sec * a_phase + 0.5f * ( delta_sec * delta_sec ) * -1;
    	   a = a + delta_sec  * - 1;
    	   break;
       case 6:
    	   ctl.value = ctl.value + delta_sec * a_phase;
    	   break;
       case 7:
    	   ctl.value = ctl.value + delta_sec * a_phase + 0.5f * ( delta_sec * delta_sec ) * 1;

    	   break;

       }

       System.out.println(ela_sec+" :: \t" + phase+":"+delta_time+"\tof "+timestamps[phase]+ "\t=> "+ctl.value+ "\t/ "+a+"\t/ "+a_phase);

       ctl.value = MSPMathUtils.constraint(ctl.value, MAX_SPEED, 0);

       return true;
	}


	public void reset() {


	}


	public void initialize(Polar3D_F32 spd, Polar3D_F32 path) {
		timestamps = TrajMathLib.computeTrajectoryTimepoints(MAX_ACCELERATION, MAX_SPEED, path.value);
		phase = 0; phase_time = 0; a = 0; a_phase= 0; ela_phase = 0;
		System.out.println("New setpoint with distance: "+path.value);
		for(int i=0; i<timestamps.length;i++)
			System.out.println(timestamps[i]);
		System.out.println();
	}

}
