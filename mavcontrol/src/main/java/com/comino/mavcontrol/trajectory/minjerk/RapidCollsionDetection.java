package com.comino.mavcontrol.trajectory.minjerk;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.comino.mavcontrol.trajectory.minjerk.math.Quartic;
import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;
import com.comino.mavcontrol.trajectory.minjerk.struct.Boundary;
import com.comino.mavcontrol.trajectory.minjerk.struct.Sphere;

import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector3D_F32;

public class RapidCollsionDetection {
	
	// ported from https://github.com/nlbucki/RapidQuadcopterCollisionDetection/blob/master/src/CollisionChecker.cpp
	
	public static final float NO_COLLISION   = -1.0f;
	public static final float NOT_DETERMINED = -2.0f;

	private final Vector3D_F32 pos = new Vector3D_F32();
	private final Point3D_F32[] trajDerivativeCoeffs = new Point3D_F32[6];
	
	public RapidCollsionDetection() {
		
		for(int i=0; i< 6; i++)
			trajDerivativeCoeffs[i] = new Point3D_F32();
	}

	public boolean collisionCheck(Boundary boundary, RapidTrajectoryGenerator traj) {

		boundary.n.normalize();

		float c[] = new float[5];

		traj.getDerivatives(trajDerivativeCoeffs);

		for (int dim = 0; dim < 3; dim++) {
			c[0] += (boundary.n.getIdx(dim) * trajDerivativeCoeffs[0].getIdx(dim));  //t**4
			c[1] += (boundary.n.getIdx(dim) * trajDerivativeCoeffs[1].getIdx(dim));  //t**4
			c[2] += (boundary.n.getIdx(dim) * trajDerivativeCoeffs[2].getIdx(dim));  //t**4
			c[3] += (boundary.n.getIdx(dim) * trajDerivativeCoeffs[3].getIdx(dim));  //t**4
			c[4] += (boundary.n.getIdx(dim) * trajDerivativeCoeffs[4].getIdx(dim));  //t**4
		}

		float roots[] = new float[6];
		roots[0] = 0;
		roots[1] = traj.getTotalTime();

		int rootCount;

		if (Math.abs(c[0]) > 1e-6f) 
			rootCount = Quartic.solve_quartic(c[1] / c[0], c[2] / c[0], c[3] / c[0], c[4] / c[0], roots);
		else 
			rootCount = Quartic.solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);

		for (int i = 0; i < (rootCount + 2); i++) {
			//don't evaluate points outside the domain
			if (roots[i] < 0)
				continue;
			if (roots[i] > traj.getTotalTime())
				continue;

			traj.getPosition(roots[i], pos);
			pos.setTo(pos.x - boundary.p.x, pos.y - boundary.p.y, pos.z - boundary.p.z);

			if (pos.dot(boundary.n) <= 0) {
				//touching, or on the wrong side of, the boundary!
				return true;
			}
		}

		return false;
	}
	
	public float collisionCheck(RapidTrajectoryGenerator traj, float ts, AbstractConvexObject obstacle, float minTimeSection) {
		traj.getPosition(ts, pos);
		if (obstacle.isPointInside(pos)) {
			return ts;
		}
		
		traj.getPosition(traj.getTotalTime(), pos);
		if (obstacle.isPointInside(pos)) {
			return ts;
		}
		
		return collisionCheckSection(traj, ts, traj.getTotalTime(),obstacle, minTimeSection);
	}


	public float collisionCheckSection(RapidTrajectoryGenerator traj, float ts, float tf, AbstractConvexObject obstacle, float minTimeSection ) 
	    {

		float midtime = (ts + tf) / 2;
		Point3D_F32 midpoint = traj.getPosition(midtime,new Point3D_F32());

		if (obstacle.isPointInside(midpoint)) {
			return midtime;
		}

		if (tf - ts < minTimeSection) {
			// Our time resolution is too small, just give up (trajectory is likely tangent to obstacle surface)
			System.err.println("Give up");
			return NOT_DETERMINED;
		}

		Boundary tangentPlane = obstacle.getTangentPlane(midpoint);

		float c[] = new float[5];

		traj.getDerivatives(trajDerivativeCoeffs);

		for (int dim = 0; dim < 3; dim++) {
			c[0] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[0].getIdx(dim));  //t**4
			c[1] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[1].getIdx(dim));  //t**3
			c[2] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[2].getIdx(dim));  //t**2
			c[3] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[3].getIdx(dim));  //t
			c[4] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[4].getIdx(dim));  //1
		}

		float roots[] = new float[4]; int rootCount;

		if (Math.abs(c[0]) > 1e-6f) 
			rootCount = Quartic.solve_quartic(c[1] / c[0], c[2] / c[0], c[3] / c[0], c[4] / c[0], roots);
		else 
			rootCount = Quartic.solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);

		Arrays.sort(roots);

		List<Float> testPointsLo = new ArrayList<Float>(6);
		List<Float> testPointsHi = new ArrayList<Float>(6);

		testPointsLo.add(ts);
		testPointsHi.add(midtime);

		for (int i = 0; i < rootCount; i++) {
			if (roots[i] <= ts) {
				// Skip root if it's before ts
				continue;
			} else if (roots[i] < midtime) {
				// Root is between ts and midTime
				testPointsLo.add(roots[i]);
			} else if (roots[i] < tf) {
				// Root is between midTime and tf
				testPointsHi.add(roots[i]);
			} else {
				// Because the roots are in ascending order, there are no more roots are on (ts,tf)
				break;
			}
		}
		
		testPointsLo.add(midtime);
		testPointsHi.add(tf);
		
		for(int i = 1;i < testPointsHi.size(); i++) {
			traj.getPosition(testPointsHi.get(i), pos);
			pos.setTo(pos.x - tangentPlane.p.x, pos.y - tangentPlane.p.y, pos.z - tangentPlane.p.z);
			if (pos.dot(tangentPlane.n) <= 0) {
				if(collisionCheckSection(traj,testPointsHi.get(i-1),tf, obstacle, minTimeSection)<= 0.0) {
					break;
				} else {
					return testPointsHi.get(i);
				}
			}
		}
		
		for(int i = testPointsLo.size()-1;i >0 ; --i) {
			traj.getPosition(testPointsLo.get(i-1), pos);
			pos.setTo(pos.x - tangentPlane.p.x, pos.y - tangentPlane.p.y, pos.z - tangentPlane.p.z);
			if (pos.dot(tangentPlane.n) <= 0) {
				return collisionCheckSection(traj,ts,testPointsLo.get(i), obstacle, minTimeSection);
			}
		}

		return NO_COLLISION;
	}







	public static void main(String[] args) {

		Point3D_F32 tmp = new Point3D_F32(0,0,0);

		RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator(new Point3D_F64(0,0,0));

		Point4D_F32 x0  = new Point4D_F32(0,0,0,0);
		Point4D_F32 vx0 = new Point4D_F32(0,0,0,0);
		Point4D_F32 ax0 = new Point4D_F32(0,0,0,0);

		Point4D_F32 x1  = new Point4D_F32(20,0,0,0);
		Point4D_F32 vx1 = new Point4D_F32(0,0,0,0);
		Point4D_F32 ax1 = new Point4D_F32(0,0,0,0);

		traj.setInitialState(x0, vx0, ax0);
		traj.setGoal(x1, vx1, ax1);

		traj.generate(10f);

		AbstractConvexObject obstacle = new Sphere(14f,1.8f,0,0.99f);

		System.out.println(obstacle);

		RapidCollsionDetection detector = new RapidCollsionDetection();

		float collision_time = detector.collisionCheckSection(traj,0.0f,10.0f,obstacle,0.01f);
			System.out.println(collision_time);
	
	}


}
