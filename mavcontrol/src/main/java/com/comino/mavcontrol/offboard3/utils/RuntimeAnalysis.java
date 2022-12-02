package com.comino.mavcontrol.offboard3.utils;

public class RuntimeAnalysis {
	
	private static long t_start_ns = 0;
	private static long t_last_ns  = 0;
	
	private static long t1_us       = 0;
	private static long t2_us       = 0;
	
	private static String s2;
	
	public static void start() {
		s2 = "Start";
		t_start_ns = System.nanoTime();
		t_last_ns = System.nanoTime();
		
	}
	
	public static void eval(String s) {
		t1_us = (System.nanoTime()-t_start_ns)/1000; t2_us = (System.nanoTime()-t_last_ns)/1000;    
		t_last_ns = System.nanoTime();
		System.out.println(s+": TotalTime at '"+s+"': "+t1_us+"us"+" DeltaTime since '"+s2+"': "+t2_us);
		s2 = s;
	}
	
	public static void end() {
		System.out.println("Elapsed Time: "+((System.nanoTime()-t_start_ns)/1000)+"us");
	}

}
