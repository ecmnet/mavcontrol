package com.comino.mavcontrol.offboard3.utils;


// Java port of https://github.com/nlbucki/RapidQuadcopterCollisionDetection/blob/master/src/quartic.cpp

public class Quartic {


	private static final double M_2PI = 2*Math.PI;
	private static final double EPS   = 1e-12;

	public static int solveP3(double a,double b,double c, double[] x) {

		double a2 = a*a;
		double q  = (a2 - 3*b)/9;
		double r  = (a*(2*a2-9*b) + 27*c)/54;
		double r2 = r*r;
		double q3 = q*q*q;
		double A,B;

		if(r2<q3)
		{
			double t=r/Math.sqrt(q3);
			if( t<-1) t=-1;
			if( t> 1) t= 1;
			t=Math.acos(t);
			a/=3; q=-2*Math.sqrt(q);
			x[0]=q*Math.cos(t/3.0)-a;
			x[1]=q*Math.cos((t+M_2PI)/3)-a;
			x[2]=q*Math.cos((t-M_2PI)/3)-a;
			return 3;
		}
		else
		{
			A =-Math.pow(Math.abs(r)+Math.sqrt(r2-q3),1./3);
			if( r<0 ) A=-A;
			B = (0==A ? 0 : q/A);

			a/=3;
			x[0] =(A+B)-a;
			x[1] =-0.5*(A+B)-a;
			x[2] = 0.5*Math.sqrt(3.)*(A-B);
			if(Math.abs(x[2])<EPS) { x[2]=x[1]; return 2; }

			return 1;
		}
	}

	public static int solve_quartic(double a,double b,double c, double d, double[] root) {

		double a3 = -b;
		double b3 =  a*c -4.*d;
		double c3 = -a*a*d - c*c + 4.*b*d;


		//initialise counters for real and imaginary roots
		int rCnt = 0;
		// cubic resolvent
		// y^3 − b*y^2 + (ac−4d)*y − a^2*d−c^2+4*b*d = 0

		double[] x3 = new double[3];
		int iZeroes = Quartic.solveP3(a3, b3, c3, x3);

		double q1, q2, p1, p2, D, sqD, y;

		y = x3[0];
		// The essence - choosing Y with maximal absolute value.
		if(iZeroes != 1)
		{
			if(Math.abs(x3[1]) > Math.abs(y)) y = x3[1];
			if(Math.abs(x3[2]) > Math.abs(y)) y = x3[2];
		}

		// h1+h2 = y && h1*h2 = d  <=>  h^2 -y*h + d = 0    (h === q)

		D = y*y - 4*d;
		if(Math.abs(D) < EPS) //in other words - D==0
		{
			q1 = q2 = y * 0.5;
			// g1+g2 = a && g1+g2 = b-y   <=>   g^2 - a*g + b-y = 0    (p === g)
			D = a*a - 4*(b-y);
			if(Math.abs(D) < EPS) //in other words - D==0
				p1 = p2 = a * 0.5;

			else
			{
				sqD = Math.sqrt(D);
				p1 = (a + sqD) * 0.5;
				p2 = (a - sqD) * 0.5;
			}
		}
		else
		{
			sqD = Math.sqrt(D);
			q1 = (y + sqD) * 0.5;
			q2 = (y - sqD) * 0.5;
			// g1+g2 = a && g1*h2 + g2*h1 = c       ( && g === p )  Krammer
			p1 = (a*q1-c)/(q1-q2);
			p2 = (c-a*q2)/(q1-q2);
		}

		// solving quadratic eq. - x^2 + p1*x + q1 = 0
		D = p1*p1 - 4*q1;
		if(!(D < 0.0))
		{
			// real roots filled from left
			sqD = Math.sqrt(D);
			root[rCnt] = (-p1 + sqD) * 0.5 ;
			++rCnt;
			root[rCnt] = (-p1 - sqD) * 0.5 ;
			++rCnt;
		}

		// solving quadratic eq. - x^2 + p2*x + q2 = 0
		D = p2*p2 - 4*q2;
		if (!(D < 0.0))
		{
			sqD = Math.sqrt(D);
			root[rCnt] = (-p2 + sqD) * 0.5 ;
			++rCnt;
			root[rCnt] = (-p2 - sqD) * 0.5 ;
			++rCnt;
		}

		return rCnt;
	}

}
