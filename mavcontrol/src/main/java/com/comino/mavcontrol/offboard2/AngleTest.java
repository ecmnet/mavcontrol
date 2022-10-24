package com.comino.mavcontrol.offboard2;

import com.comino.mavutils.MSPMathUtils;

public class AngleTest {
	
	private static float normAngle(float a) {
		return a - (2*(float)Math.PI) * (float)Math.floor((a + (float)Math.PI - 0.5) / (2*(float)Math.PI));
	}

	private static float normAngle(float a, float b) {
		return normAngle(b - a);
	}

	private static float normAngleAbs(float a, float b) {
		return (float)Math.abs(normAngle(a,b));
	}
	
	
	public static void test(float t, float c) {
		float b = 0;
		t = MSPMathUtils.toRad(t);
		c = MSPMathUtils.toRad(c);
		
		t = normAngle(t);
		c = normAngle(c);
		
		float r = normAngle(t,c);
		float s = normAngle(c,t);
		
		b = t;
		if(c < 0)
			b = (t - 2*(float)Math.PI) % (2*(float)Math.PI);
		
		System.out.println("["+MSPMathUtils.fromRad(t)+","+MSPMathUtils.fromRad(c)+"]"+MSPMathUtils.fromRad(r)+" ("+r+ ") / "+ 
		                       MSPMathUtils.fromRad(s)+" ("+s + ") --> from "+MSPMathUtils.fromRad(c)+" ("+c+") to "+ MSPMathUtils.fromRad(b)+" ("+b+") : " +
				               Math.abs(b - c)
		                       
				);
		
	}

	public static void main(String[] args) {
		
		test(45,20);
		System.out.println();
		test(180,-90);
		test(180,270);
		System.out.println();
		test(45,180);
		test(45,-180);
		System.out.println();
		test(180,45);
		test(180,45);
		System.out.println();

	}

}
