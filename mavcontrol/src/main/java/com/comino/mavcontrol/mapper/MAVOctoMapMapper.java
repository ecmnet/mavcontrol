package com.comino.mavcontrol.mapper;

import java.util.stream.Stream;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_micro_grid;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTreeNode;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavmap.map.map3D.impl.octomap.store.OctoMap3DStorage;
import com.comino.mavutils.workqueue.WorkQueue;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OccupancyTools;

/*
 * Map handling
 * Ideas:
 *  - short term long term maps
 *     - short term map: Collision Avoidance, short term planning, lasts about 5 secs
 *     - long term map: Global plan, remains 
 *  - visualize both in MAVGCL
 *  
 *  - Question: Collision check here ?
 *  
 */
public class MAVOctoMapMapper {

	private final WorkQueue wq;

	private final MAVOctoMap3D      short_term_map;
	//	private final MAVOctoMap3D      long_term_map;
	private final IMAVController    control;
	private final DataModel         model;
	private final MSPLogger         logger;

	private boolean			        mapForget         = false;
	private boolean                 publish_microgrid = true;

	public MAVOctoMapMapper(IMAVController control,MSPConfig config) {
		super();

		this.wq      = WorkQueue.getInstance();
		this.logger  = MSPLogger.getInstance();
		this.control = control;
		this.model   = control.getCurrentModel();

		this.short_term_map  = new MAVOctoMap3D(0.2f,true);
		//		this.long_term_map   = new MAVOctoMap3D(1.0f,false);

		this.setupConfig(config);


		if(publish_microgrid)
			wq.addCyclicTask("NP",20, new MapToModelTransfer());

	}

	public MAVOctoMap3D getShorTermMap() {
		return short_term_map;
	}

	public void resetMap() {
		logger.writeLocalMsg("[msp] resetting maps",MAV_SEVERITY.MAV_SEVERITY_NOTICE);
		short_term_map.clear();
		short_term_map.enableRemoveOutdated();
		msg_msp_micro_grid grid = new msg_msp_micro_grid(2,1);
		grid.count = -1;
		control.sendMAVLinkMessage(grid);
	}

	public void saveMap2D() {
		OctoMap3DStorage store = new OctoMap3DStorage(short_term_map, model.state.g_lat, model.state.g_lon);
		store.write();
		logger.writeLocalMsg("[msp] Map for this home position stored.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
	}

	public void loadMap2D() {


		OctoMap3DStorage store = new OctoMap3DStorage(short_term_map, model.state.g_lat, model.state.g_lon);
		//						try {
		//		//		store.importOctomap("euroc_1.bt");
		//							store.importOctomap("octomap.bt");
		//		//					model.grid.count = map.getNumberOfNodes();
		//		//		
		//						} catch (IOException e) {
		//							// TODO Auto-generated catch block
		//							e.printStackTrace();
		//						}
		//
		//		//store.readLegacyM3D("test.m3D");

		if(store.locateAndRead()) {
			short_term_map.disableRemoveOutdated();
			logger.writeLocalMsg("[msp] Map for this home position loaded.",MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		}
		else {
			logger.writeLocalMsg("[msp] No Map for this home position found.",MAV_SEVERITY.MAV_SEVERITY_WARNING);
		}
	}

	public void invalidate_map_transfer() {

		if(!publish_microgrid)
			return;

		msg_msp_micro_grid grid = new msg_msp_micro_grid(2,1);
		grid.count = -1;
		control.sendMAVLinkMessage(grid);

		short_term_map.invalidateAllOccupied();
	}

	/*
	 * Setup parameters by MSPConfig 
	 */
	private void setupConfig(MSPConfig config) {

		this.publish_microgrid = config.getBoolProperty(MSPParams.PUBLISH_MICROGRID, "true");
		System.out.println("[map] Publishing microGrid enabled: "+publish_microgrid);

		this.mapForget = config.getBoolProperty(MSPParams.AUTOPILOT_FORGET_MAP, "true");
		System.out.println("[map] Map forget enabled: "+mapForget);

	}

	/*
	 * Transfer short term map to MAVGCL
	 */
	private class MapToModelTransfer implements Runnable {

		private final msg_msp_micro_grid  grid     = new msg_msp_micro_grid(2,1);

		@Override
		public void run() {

			if(!publish_microgrid || !model.sys.isStatus(Status.MSP_GCL_CONNECTED)) {
				return;
			}

			model.sys.setSensor(Status.MSP_GRID_AVAILABILITY, true);

			if(mapForget) {
				short_term_map.removeOutdatedNodes(125,30000);
			}
			model.grid.count = short_term_map.getNumberOfNodes();

			if(short_term_map.getTree().numberOfChangesDetected() == 0)
				return;

			Stream<OcTreeKeyReadOnly> stream = short_term_map.getTree().getChangedKeys().keySet().stream().limit(125);

			stream.forEach((key) ->  {
				model.grid.add(encodeKey(key));
				short_term_map.getTree().getChangedKeys().remove(key);
			});

			sendGridMessage();

		}

		private long encodeKey(OcTreeKeyReadOnly key) {
			MAVOccupancyOcTreeNode node = short_term_map.getTree().search(key);
			if(node==null)
				return 0;
			if(OccupancyTools.isNodeOccupied(short_term_map.getTree().getOccupancyParameters(), node))
				return short_term_map.encode(key,1);
			else
				return short_term_map.encode(key,0);
		}

		private void sendGridMessage() {
			while(model.grid.hasTransfers()) {
				if(model.grid.toArray(grid.data)) {
					grid.tms        = DataModel.getSynchronizedPX4Time_us();
					grid.count      = model.grid.count;
					grid.resolution = short_term_map.getResolution();
					control.sendMAVLinkMessage(grid);
				}
			}	
		}

	}

}
