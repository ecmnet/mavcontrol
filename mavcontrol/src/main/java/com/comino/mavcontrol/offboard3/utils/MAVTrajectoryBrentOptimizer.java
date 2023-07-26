package com.comino.mavcontrol.offboard3.utils;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.optimization.GoalType;
import org.apache.commons.math3.optimization.univariate.BrentOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariatePointValuePair;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcontrol.offboard3.states.Offboard3Current;
import com.comino.mavcontrol.trajectory.minjerk.RapidTrajectoryGenerator;
import com.comino.mavmap.map.map3D.impl.octomap.esdf.D2.DynamicESDF2D;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.GeoTuple4D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F32;

@SuppressWarnings("deprecation")
public class MAVTrajectoryBrentOptimizer {
	
	private YawCostFunc  yaw;
    private NormCostFunc norm;
  
	private final BrentOptimizer brent;
	private final LinkedList<RapidTrajectoryGenerator> trajs = new LinkedList<RapidTrajectoryGenerator>();
	private final Offboard3Current current;


	public MAVTrajectoryBrentOptimizer(Offboard3Current current,DynamicESDF2D edf) {
       this.brent     = new BrentOptimizer(1e-10, 1e-14);
       this.yaw       = new YawCostFunc(current, edf, trajs);
       this.norm      = new NormCostFunc(current, edf, trajs);
       this.current   = current;
	}
	
	public RapidTrajectoryGenerator optimize(final GeoTuple4D_F32<?> goalPos, final GeoTuple4D_F32<?> goalDir, double tf) {
		
		this.current.update();
		goalDir.normalize();
		
		double norm0 = current.vel().norm();
		double yaw0  = Math.atan2(current.vel().y, current.vel().x);
		
		yaw.set(goalPos, goalDir, (float)Math.max(.1, norm0 - 1/tf), 0, current.pos().z, tf);
		UnivariatePointValuePair r1 = brent.optimize(100, yaw, GoalType.MINIMIZE, yaw0 - Math.PI * .5, yaw0 + Math.PI * .5);
		double tmpYaw = r1.getPoint();
		
		norm.set(goalPos, goalDir, 0, (float)tmpYaw, current.pos().z, tf);
		UnivariatePointValuePair r2 = brent.optimize(100, norm, GoalType.MINIMIZE, Math.max(norm0 - 4/tf, 0.1), norm0 + .5/tf);
		double norm = r2.getPoint();
		
		yaw.set(goalPos, goalDir, (float)norm, 0, current.pos().z, tf);
		UnivariatePointValuePair r3 = brent.optimize(100, yaw, GoalType.MINIMIZE, tmpYaw - Math.PI * .4, tmpYaw + Math.PI * .4);
		double yaw = r3.getPoint();
		
		RapidTrajectoryGenerator traj = buildTrajectory(current.pos(),current.vel(),current.acc(),(float)norm, (float)yaw, current.pos().z, tf);
		return traj;
		
	}
	
	protected RapidTrajectoryGenerator buildTrajectory(final GeoTuple4D_F32<?>  pos0, final GeoTuple4D_F32<?> vel0, final GeoTuple3D_F32<?> acc0,
			final float normVelf, final float yawf,final float zf, final double tf) {

		RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator(new Point3D_F64(0,0,-9.80665));
		traj.setInitialState(pos0, vel0, acc0);
		traj.setGoalPosition(new Point3D_F64(Double.NaN,Double.NaN,zf));
		traj.setGoalVelocity(new Point3D_F64(normVelf*Math.cos(yawf), normVelf*Math.sin(yawf),0));
		traj.setGoalAcceleration(new Point3D_F64(0,0,0));
		traj.generate(tf);
		return traj;		

	}

	
	private class YawCostFunc extends CostFunction {

		public YawCostFunc(Offboard3Current current, DynamicESDF2D edf, LinkedList<RapidTrajectoryGenerator> trajs) {
			super(current, edf, trajs);
		}

		@Override
		public double value(double yaw) {
			RapidTrajectoryGenerator traj = buildTrajectory(current.pos(),current.vel(),current.acc(),normVelf,(float)yaw,zf,tf);
			return getCost(traj,tf);
		}
		
	}
	
	private class NormCostFunc extends CostFunction {

		public NormCostFunc(Offboard3Current current, DynamicESDF2D edf, List<RapidTrajectoryGenerator> trajs) {
			super(current, edf, trajs);
		}

		@Override
		public double value(double norm) {
			RapidTrajectoryGenerator traj = buildTrajectory(current.pos(),current.vel(),current.acc(),(float)norm,yawf,zf,tf);
		//	trajs.add(traj);
			return getCost(traj,tf);
		}
		
	}
	
	private abstract class CostFunction implements UnivariateFunction {
		
		protected final List<RapidTrajectoryGenerator> trajs;
		protected final Offboard3Current current;
		protected float normVelf;
		protected float yawf;
		protected float zf;
		protected double tf;
		

		private final GeoTuple3D_F64<?> pf = new Point3D_F64();
		private final GeoTuple3D_F64<?> vf = new Point3D_F64();
		private final GeoTuple3D_F64<?> vu = new Point3D_F64();
		private final GeoTuple3D_F64<?> p0 = new Point3D_F64();
		private final GeoTuple3D_F64<?> pt = new Point3D_F64();
		private final GeoTuple3D_F64<?> pc = new Point3D_F64();
		private final DynamicESDF2D edf;
		
	    private GeoTuple4D_F32<?>   goalPos;
		private GeoTuple4D_F32<?>   goalDir;
		
		
		public CostFunction(Offboard3Current current, DynamicESDF2D edf,List<RapidTrajectoryGenerator> trajs) {
			this.edf = edf;
			this.trajs = trajs;
			this.current = current;
		}

		public void set(final GeoTuple4D_F32<?> goalPos, final GeoTuple4D_F32<?> goalDir, final float normVelf, final float yawf,final float zf, final double tf) {
			this.current.update();
			this.normVelf = normVelf;
			this.yawf = yawf;
			this.zf  = zf;
			this.tf = tf;
			this.goalDir = goalDir;
			this.goalPos = goalPos;
		}
		
		protected double getCost(final RapidTrajectoryGenerator traj, final double tf) {
			
			traj.getPosition(tf, pf); traj.getPosition(0, p0);
			
			pt.setTo(pf.x-p0.x,pf.y-p0.y,pf.z-p0.z);
			int n = (int)Math.max(pt.norm()*50, 50);
			
			double sum = 1e-10;
			for (double t = 0.; t < tf; t += tf/n) {
				 traj.getPosition(t, pc);
				 sum += 1 / Math.max(0.0001, edf.getDistanceAt(pc) - 0.2);
			}
		
			traj.getVelocity(tf, vf);
			double vfn = vf.norm();
			double ec = vfn * sum * tf / n;
			
			if (ec > 1e8) {
		        return ec;
		    }
			
			vu.setTo(vf.x,vf.y,vf.z);	
			vu.timesIP(1/vfn);
			vu.setTo(vu.x-goalDir.x,vu.y-goalDir.y,vu.z-goalDir.z);
			
			pt.setTo(goalPos.x-pf.x,goalPos.y-pf.y,goalPos.z-pf.z);
			
			return 5.0 * pt.norm() + 0.1 * ec + 1.0 * vu.norm();
			
		}
		
		protected RapidTrajectoryGenerator buildTrajectory(final GeoTuple4D_F32<?>  pos0, final GeoTuple4D_F32<?> vel0, final GeoTuple3D_F32<?> acc0,
				final float normVelf, final float yawf,final float zf, final double tf) {

			RapidTrajectoryGenerator traj = new RapidTrajectoryGenerator(new Point3D_F64(0,0,-9.80665));
			traj.setInitialState(pos0, vel0, acc0);
			traj.setGoalPosition(new Point3D_F64(Double.NaN,Double.NaN,zf));
			traj.setGoalVelocity(new Point3D_F64(normVelf*Math.cos(yawf), normVelf*Math.sin(yawf),0));
			traj.setGoalAcceleration(new Point3D_F64(0,0,0));
			traj.generate(tf);
			return traj;		

		}
	}

	public static void main(String[] args) {

		UnivariateFunction functionToOptimize = new UnivariateFunction() {

			@Override
			public double value(double x) {
				return (x-2)*(x-2) + 5;
			}
		};

		UnivariatePointValuePair result;

		var brent      = new BrentOptimizer(1e-10, 1e-14);
		try {
			result = brent.optimize(200, functionToOptimize, GoalType.MINIMIZE, -10.0, 10);
			System.out.println(result.getPoint());
		} catch (TooManyEvaluationsException e) // _maxAssigmentAttemptsPerBranchParam exceeded
		{

		}	

	}
}
