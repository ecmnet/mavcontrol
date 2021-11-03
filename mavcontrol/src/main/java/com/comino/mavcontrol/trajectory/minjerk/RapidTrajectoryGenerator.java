
/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * This is a port to Java from minimum_jerkt_trajectories in
 * https://zenodo.org/record/5517791#.YW_kBS-21B1
 * 
 * The algorithm is described in the following paper: 
 * M.W. Mueller, M. Hehn, and R. D'Andrea, "A computationally efficient motion 
 * primitive for quadrocopter trajectory generation," 
 * IEEE Transactions on Robotics Volume 31, no.8, pages 1294-1310, 2015.
 * 
 * The paper may be downloaded from 
 * http://muellerlab.berkeley.edu/publications/
 *
 ****************************************************************************/


package com.comino.mavcontrol.trajectory.minjerk;

import org.ddogleg.solver.PolynomialSolver;
import org.ejml.data.Complex_F64;

import com.comino.mavcom.model.DataModel;

import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F32;

public class RapidTrajectoryGenerator {
	
	private final double MIN_ACC		=  5.0;
	private final double MAX_ACC		= 10.0;
	private final double MAX_BODY_RATE  = 2;
	private final double TIME_STEP      = 0.02;
	

	private final SingleAxisTrajectory _axis[] = new SingleAxisTrajectory[3];

	private Point3D_F64 _gravity;
	private double      _tf;

	private final Point3D_F64 _tmp1 = new Point3D_F64();
	private final Point3D_F64 _tmp2 = new Point3D_F64();

	public RapidTrajectoryGenerator() {
		super();
		for(int i=0;i<3;i++) {
			_axis[i] = new SingleAxisTrajectory();
		}
	}

	public RapidTrajectoryGenerator(Point3D_F64 gravity) {
		super();
		for(int i=0;i<3;i++) {
			_axis[i] = new SingleAxisTrajectory();
		}
		this._gravity = gravity;
	}

	public RapidTrajectoryGenerator(Point3D_F64 x0, Point3D_F64 v0, Point3D_F64 a0, Point3D_F64 gravity) {
		reset();
		for(int i=0;i<3;i++) {
			_axis[i] = new SingleAxisTrajectory();
			_axis[i].setInitialState(x0.getIdx(i),v0.getIdx(i),a0.getIdx(i));
		}
		this._gravity = gravity;
	}

	public void setInitialState(Point3D_F64 x0, Point3D_F64 v0, Point3D_F64 a0, Point3D_F64 gravity) {
		reset();
		for(int i=0;i<3;i++) {
			_axis[i].setInitialState(x0.getIdx(i),v0.getIdx(i),a0.getIdx(i));
		}
		this._gravity = gravity;
	}

	public void setGoal(Point3D_F64 p, Point3D_F64 v, Point3D_F64 a) {
		for(int i=0;i<3;i++) {
			_axis[i].setGoalPosition(p.getIdx(i));
			_axis[i].setGoalVelocity(v.getIdx(i));
			_axis[i].SetGoalAcceleration(a.getIdx(i));
		}	
	}

	public void setGoalPosition(Point3D_F64 in) {
		for(int i=0;i<3;i++)
			_axis[i].setGoalPosition(in.getIdx(i));
	}

	public void setGoalVelocity(Point3D_F64 in) {
		for(int i=0;i<3;i++)
			_axis[i].setGoalVelocity(in.getIdx(i));
	}

	public void setGoalAcceleration(Point3D_F64 in) {
		for(int i=0;i<3;i++)
			_axis[i].SetGoalAcceleration(in.getIdx(i));
	}

	public void reset() {
		for(int i=0;i<3;i++)
			_axis[i].reset();
		_tf = 0;
	}

	public double getCost() {
		return _axis[0].getCost() + _axis[1].getCost() + _axis[2].getCost();
	}

	public void generate(double timeToFinish) {
		_tf = timeToFinish;
		for(int i=0;i<3;i++)
			_axis[i].generateTrajectory(_tf);
	}

	public boolean generate(double timeToFinish, DataModel model, Vector4D_F32 target, Vector4D_F32 velocity) {
		
		if(timeToFinish<0)
			return false;
		
		reset();
		
		_axis[0].setInitialState(model.state.l_x, model.state.l_vx, model.state.l_ax);
		_axis[1].setInitialState(model.state.l_y, model.state.l_vy, model.state.l_ay);
		_axis[2].setInitialState(model.state.l_z, model.state.l_vz, model.state.l_az);

		for(int i= 0; i<3;i++) {
			_axis[i].setGoalPosition(target.getIdx(i));
			if(velocity!=null)
				_axis[i].setGoalVelocity(velocity.getIdx(i));
			else
				_axis[i].setGoalVelocity(0);
			_axis[i].SetGoalAcceleration(0);
		}

		generate(timeToFinish);
		if(!checkInputFeasibility(MIN_ACC,MAX_ACC,MAX_BODY_RATE,TIME_STEP))
			return false;

		return true;

	}

	public Point3D_F64 getBodyRates(double t, double timeStep, Point3D_F64 crossProd) {

		Point3D_F64 n0 = getNormalVector(t,_tmp1);
		Point3D_F64 n1 = getNormalVector(t+timeStep,_tmp2);

		if(crossProd==null)
			crossProd = new Point3D_F64();

		GeometryMath_F64.cross(n0, n1, crossProd);

		if(crossProd.norm() == 0)
			crossProd.setTo(0,0,0);
		else {
			crossProd.divideIP(crossProd.norm());
			crossProd.timesIP(Math.acos(GeometryMath_F64.dot(n0,n1))/timeStep);
		}

		return crossProd;
	}

	public boolean checkInputFeasibilitySection(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double t1, double t2, double minTimeSection) {


		if (t2 - t1 < minTimeSection) return false;
		if(Math.max(getThrust(t1,_tmp1), getThrust(t2,_tmp2)) > fmaxAllowed) 
			return false;
		if(Math.min(getThrust(t1,_tmp1), getThrust(t2,_tmp2)) < fminAllowed) 
			return false;

		double fminSqr = 0;
		double fmaxSqr = 0;
		double jmaxSqr = 0;

		for (int i = 0; i < 3; i++) {
			_axis[i].calcMinMaxAcc(t1, t2);
			double v1 = _axis[i].getMinAcc() - _gravity.getIdx(i); //left
			double v2 = _axis[i].getMaxAcc() - _gravity.getIdx(i); //right

			if(Math.max(v1*v1, v2*v2) > (fmaxAllowed * fmaxAllowed)) return false;

			if (v1 * v2 < 0)
				fminSqr += 0; //sign of acceleration changes, so we've gone through zero
			else
				fminSqr += Math.pow(Math.min(Math.abs(v1), Math.abs(v1)),2);

			fmaxSqr += Math.pow(Math.max(Math.abs(v1), Math.abs(v2)),2);	
			jmaxSqr += _axis[i].getMaxJerkSquared(t1, t2);

		}

		double fmin = Math.sqrt(fminSqr);
		double fmax = Math.sqrt(fmaxSqr);

		double wBound;
		if (fminSqr > 1e-6) 
			wBound = Math.sqrt(jmaxSqr / fminSqr);  //the 1e-6 is a divide-by-zero protection
		else 
			wBound = Double.MAX_VALUE;

		if(fmax < fminAllowed) 
			return false;
		if(fmin > fmaxAllowed) 
			return false;

		//possibly infeasible:
		if (fmin < fminAllowed || fmax > fmaxAllowed || wBound > wmaxAllowed)
		{ //indeterminate: must check more closely:
			double tHalf = (t1 + t2) / 2;
			boolean r1 = checkInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1, tHalf, minTimeSection);
			if(r1 == true)
				//continue with second half
				return checkInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, tHalf, t2, minTimeSection);
			//first section is already infeasible, or indeterminate:
			return r1;
		}

		return true;
	}

	public boolean checkInputFeasibility(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double minTimeSection)
	{
		//required thrust limits along trajectory
		double t1 = 0;
		double t2 = _tf;

		return checkInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1, t2, minTimeSection);
	}

	public boolean checkPositionFeasibility(Point3D_F64 boundaryPoint, Point3D_F64 boundaryNormal) {

		boundaryNormal.divideIP(boundaryNormal.norm());
		double c[] = { 0, 0, 0, 0, 0 };

		for(int dim=0; dim<3; dim++) {
			c[0] += boundaryNormal.getIdx(dim)*_axis[dim].getParamAlpha() / 24.0; //t**4
			c[2] += boundaryNormal.getIdx(dim)*_axis[dim].getParamGamma() / 2.0;  //t**2
			c[3] += boundaryNormal.getIdx(dim)*_axis[dim].getInitialAcc();        //t
			c[4] += boundaryNormal.getIdx(dim)*_axis[dim].getInitialVel();        //1
		}

		boundaryPoint.scale(-1);

		getPosition(0,_tmp1).plusIP(boundaryPoint);
		if(GeometryMath_F64.dot(_tmp1,boundaryNormal) <= 0)
			return false;

		getPosition(_tf,_tmp1).plusIP(boundaryPoint);
		if(GeometryMath_F64.dot(_tmp1,boundaryNormal) <= 0)
			return false;

		Complex_F64[] roots;

		if(Math.abs(c[0]) > 1e-6)
			roots = PolynomialSolver.polynomialRootsEVD(c[1] / c[0], c[2] / c[0], c[3] / c[0], c[4] / c[0]);
		else
			roots = PolynomialSolver.polynomialRootsEVD(c[2] / c[1], c[3] / c[1], c[4] / c[1]);

		for(int i=0; i<roots.length;i++) {

			//don't evaluate points outside the domain
			if(roots[i].real < 0) continue;
			if(roots[i].real > _tf) continue;

			getPosition(roots[i].real,_tmp1).plusIP(boundaryPoint);
			if(GeometryMath_F64.dot(_tmp1,boundaryNormal) <= 0)
				return false;

		}

		return true;
	}

	public void getState(double t, Point3D_F64 p, Point3D_F64 v, Point3D_F64 a) {
		for(int i=0;i<3;i++) {
			a.setIdx(i, _axis[i].getAcceleration(t));
			v.setIdx(i, _axis[i].getVelocity(t));
			p.setIdx(i, _axis[i].getPosition(t));
		}
	}

	public Point3D_F64 getJerk(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getJerk(t));
		return out;
	}

	public Point3D_F64 getAcceleration(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getAcceleration(t));
		return out;
	}

	public Point3D_F64 getVelocity(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getVelocity(t));
		return out;
	}

	public Point3D_F64 getPosition(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getPosition(t));
		return out;
	}
	
	public double getInitialPosition(int i) {
		return _axis[i].getInitialPos();
	}
	
	public double getInitialVelocity(int i) {
		return _axis[i].getInitialVel();
	}
	
	public double getInitialAcceleration(int i) {
		return _axis[i].getInitialAcc();
	}


	public Point3D_F64 getNormalVector(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getAcceleration(t) - _gravity.getIdx(i));	
		out.divideIP(out.norm());
		return out;
	}

	public double getThrust(double t, Point3D_F64 out) {
		if(out == null)
			out = new Point3D_F64();
		for(int i=0;i<3;i++)
			out.setIdx(i, _axis[i].getAcceleration(t) - _gravity.getIdx(i));	
		return out.norm();	
	}

	public double getAxisParamAlpha(int i) {
		return _axis[i].getParamAlpha();
	}

	public double getAxisParamBeta(int i) {
		return _axis[i].getParamBeta();
	}

	public double getAxisParamGamma(int i) {
		return _axis[i].getParamGamma();
	}



}
