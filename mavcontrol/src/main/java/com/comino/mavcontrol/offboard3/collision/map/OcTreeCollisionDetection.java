package com.comino.mavcontrol.offboard3.collision.map;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavcontrol.trajectory.minjerk.math.Quartic;
import com.comino.mavcontrol.trajectory.minjerk.struct.AbstractConvexObject;
import com.comino.mavcontrol.trajectory.minjerk.struct.Boundary;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTreeNode;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Vector3D_F32;
import us.ihmc.euclid.tuple3D.Point3D;

public class OcTreeCollisionDetection {
	

	public static final float NO_COLLISION   = -1.0f;
	public static final float NOT_DETERMINED = -2.0f;
	
	private final MAVOccupancyOcTree  tree;
	
	private final Point3D_F32[] trajDerivativeCoeffs = new Point3D_F32[6];
	private final float         coef[]               = new float[5];
	private final List<Float>   testPointsLo         = new ArrayList<Float>(6);
	private final List<Float>   testPointsHi         = new ArrayList<Float>(6);
	
	private final float res_2;
	
	private final Boundary tangentPlane              = new Boundary();
	private final Point3D  tmp;
	
	private final Vector3D_F32  midpoint             = new Vector3D_F32();
	private final Vector3D_F32  pos                  = new Vector3D_F32();

	
	public OcTreeCollisionDetection(MAVOccupancyOcTree tree, float resolution) {
		
		this.tree   = tree;
		this.res_2 = resolution / 2.0f;
		
		this.tmp    = new Point3D();
		
		for(int i=0; i< 6; i++)
			trajDerivativeCoeffs[i] = new Point3D_F32();
		
	}
	
	public float collisionCheck(RapidTrajectoryGenerator traj, float ts, float minTimeSection) {
		traj.getPosition(ts, pos);
		tmp.set(midpoint.x,midpoint.y,-midpoint.z);
		MAVOccupancyOcTreeNode node = tree.search(tmp);
		if (node !=null && tree.isNodeOccupied(node)) {
			return ts;
		}

		traj.getPosition(traj.getTotalTime(), pos);
		tmp.set(midpoint.x,midpoint.y,-midpoint.z);
		node = tree.search(tmp);
		if (node!=null && tree.isNodeOccupied(node)) {
			return traj.getTotalTime();
		}

		return collisionCheckSection(traj, ts, traj.getTotalTime(),minTimeSection);
	}
	
	public float collisionCheckSection(RapidTrajectoryGenerator traj, float ts, float tf, float minTimeSection ) 
	{

		float midtime = (ts + tf) / 2;
		traj.getPosition(midtime,midpoint);

		tmp.set(midpoint.x,midpoint.y,-midpoint.z);
		MAVOccupancyOcTreeNode node = tree.search(tmp);
		
		
		if(node!=null && tree.isNodeOccupied(node)) {
			return midtime;
		}

		if (tf - ts < minTimeSection) {
			// Our time resolution is too small, just give up (trajectory is likely tangent to obstacle surface)
			return NOT_DETERMINED;
		}
		
		getTangentPlane(node, midpoint);

		traj.getDerivatives(trajDerivativeCoeffs);

		for (int dim = 0; dim < 3; dim++) {
			coef[0] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[0].getIdx(dim));  //t**4
			coef[1] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[1].getIdx(dim));  //t**3
			coef[2] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[2].getIdx(dim));  //t**2
			coef[3] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[3].getIdx(dim));  //t
			coef[4] += (tangentPlane.n.getIdx(dim) * trajDerivativeCoeffs[4].getIdx(dim));  //1
		}

		float roots[] = new float[4]; int rootCount;

		if (Math.abs(coef[0]) > 1e-6f) 
			rootCount = Quartic.solve_quartic(coef[1] / coef[0], coef[2] / coef[0], coef[3] / coef[0], coef[4] / coef[0], roots);
		else 
			rootCount = Quartic.solveP3(coef[2] / coef[1], coef[3] / coef[1], coef[4] / coef[1], roots);

		Arrays.sort(roots);

		testPointsLo.clear(); testPointsHi.clear();

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

		for(int i = 1;i < testPointsHi.size();i++) {
			traj.getPosition(testPointsHi.get(i), pos);
			pos.setTo(pos.x - tangentPlane.p.x, pos.y - tangentPlane.p.y, pos.z - tangentPlane.p.z);
			if (pos.dot(tangentPlane.n) <= 0) {
				if(collisionCheckSection(traj,testPointsHi.get(i-1),tf, minTimeSection)<= 0.0) {
					break;
				} else {
					return testPointsHi.get(i-1);
				}
			}
		}

		for(int i = testPointsLo.size()-1;i >0 ; --i) {
			traj.getPosition(testPointsLo.get(i-1), pos);
			pos.setTo(pos.x - tangentPlane.p.x, pos.y - tangentPlane.p.y, pos.z - tangentPlane.p.z);
			if (pos.dot(tangentPlane.n) <= 0) {
				return collisionCheckSection(traj,ts,testPointsLo.get(i), minTimeSection);
			}
		}

		return NO_COLLISION;
	}
	
	
	private void getTangentPlane(MAVOccupancyOcTreeNode node, GeoTuple3D_F32<?> p) {
	
		
		tangentPlane.n.setTo(p.x - (float)node.getX(), p.y - (float)node.getY(),p.z - (float)node.getZ()); tangentPlane.n.normalize();
       
        
        for (int i = 0; i < 3; i++) {
            if (tangentPlane.p.getIdx(i) < -res_2) {
            	tangentPlane.p.setIdx(i, - res_2);
            } else if (tangentPlane.p.getIdx(i) > res_2) {
            	tangentPlane.p.setIdx(i, res_2);
            } 
        }
        
        // Calculates boundary.n = (p - boundary.p).GetUnitVector();
        tangentPlane.n.setTo(tangentPlane.p); tangentPlane.n.scale(-1.0f); tangentPlane.n.plusIP(p); tangentPlane.n.normalize();
        
	}

}
