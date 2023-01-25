package com.comino.mavcontrol.scenario.parser;

import java.util.LinkedList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.control.impl.MAVSimController;
import com.comino.mavcontrol.scenario.items.AbstractScenarioItem;
import com.comino.mavcontrol.scenario.items.ArmingItem;
import com.comino.mavcontrol.scenario.items.DisarmingItem;
import com.comino.mavcontrol.scenario.items.FiducialItem;
import com.comino.mavcontrol.scenario.items.MoveToItem;
import com.comino.mavcontrol.scenario.items.ObstacleItem;
import com.comino.mavcontrol.scenario.items.PrecisionLandItem;
import com.comino.mavcontrol.scenario.items.RotateItem;
import com.comino.mavcontrol.scenario.items.TakeOffItem;

public class ScenarioReader {

	private final IMAVController control;

	public ScenarioReader(IMAVController control) {
		this.control = control;
	}


	public Scenario readScenario(String filename) {

		Scenario scenario = new Scenario();

		DocumentBuilder dBuilder;
		try {
			dBuilder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
			Document doc = dBuilder.parse(getClass().getResourceAsStream("/" + filename));
			if (!doc.hasChildNodes())
				return null;
			NodeList scenarios = doc.getElementsByTagName("scenario");
			if(scenarios!=null && scenarios.getLength()>0) {
				parseScenario(scenarios.item(0).getChildNodes(),scenario);	
				return scenario;
			}
			return scenario;

		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}

	private void parseScenario(NodeList scenario_childs, Scenario s) {

		for(int i=0; i<scenario_childs.getLength();i++) {
			Node scenario_child = scenario_childs.item(i);
			switch(scenario_child.getNodeName().toLowerCase()) {
			case "steps":
				s.setList(parseSteps(scenario_child.getChildNodes())); 
			case "name":
				s.setName(scenario_child.getTextContent().trim());
				break;
			case "description":
				break;
			case "type":
				if(scenario_child.getTextContent().toLowerCase().contains("sitl"))
					s.setType(Scenario.TYPE_SITL);

			}
		}
	}

	private LinkedList<AbstractScenarioItem> parseSteps(NodeList steps) {

		LinkedList<AbstractScenarioItem> list = new LinkedList<AbstractScenarioItem>();

		for(int i=0; i<steps.getLength();i++) {
			Node step = steps.item(i);
			switch(step.getNodeName().toLowerCase()) {
			case "arm":
				ArmingItem arming = new ArmingItem(control);
				list.add(arming);
				break;	
			case "disarm":
				DisarmingItem disarming = new DisarmingItem(control);
				list.add(disarming);
				break;			
			case "takeoff":
				TakeOffItem takeoff = new TakeOffItem(control);
				if(step.hasChildNodes()) {
					NodeList params = step.getChildNodes();
					for(int j=0;j<params.getLength();j++) {
						Node param = params.item(j);
						switch(param.getNodeName().toLowerCase()) {
						case "altitude":
							takeoff.setTakeoffAltitude(parseFloatAttribute(param,"a"));
							break;
						}
					}
				}
				list.add(takeoff);   
				break;
			case "moveto":
				MoveToItem moveto = new MoveToItem(control);
				if(step.hasChildNodes()) {
					NodeList params = step.getChildNodes();
					for(int j=0;j<params.getLength();j++) {
						Node param = params.item(j);
						switch(param.getNodeName().toLowerCase()) {
						case "position_local":
							parsePositionLocal(param, moveto);
							break;
						case "acceptance_radius":
							moveto.setAcceptanceRadius(parseFloatAttribute(param,"r"));
							break;
						case "delay":
							moveto.setDelay((int)parseFloatAttribute(param,"d"));
							break;
						}
					}
					list.add(moveto);
				}
				break;
			case "rotate":
				RotateItem rotate = new RotateItem(control);
				if(step.hasChildNodes()) {
					NodeList params = step.getChildNodes();
					for(int j=0;j<params.getLength();j++) {
						Node param = params.item(j);
						switch(param.getNodeName().toLowerCase()) {
						case "angle":
							rotate.setYaw(parseFloatAttribute(param,"a"));
							break;
						case "delay":
							rotate.setDelay((int)parseFloatAttribute(param,"d"));
							break;
						}
					}
					list.add(rotate);
				}
				break;
			case "precision_land":
				PrecisionLandItem precision_land = new PrecisionLandItem(control);
				list.add(precision_land);
				break;

			case "obstacle":
				ObstacleItem obstacle = new ObstacleItem(control);
				if(step.hasChildNodes()) {
					NodeList params = step.getChildNodes();
					for(int j=0;j<params.getLength();j++) {
						Node param = params.item(j);
						switch(param.getNodeName().toLowerCase()) {
						case "position_local":
							parsePositionLocal(param, obstacle);
							break;
						case "size":
							obstacle.setSize(parseFloatAttribute(param,"s"));
							break;
						}
					}
				}
				list.add(obstacle);
				break;
			case "fiducial":
				FiducialItem fiducial = new FiducialItem(control);
				if(step.hasChildNodes()) {
					NodeList params = step.getChildNodes();
					for(int j=0;j<params.getLength();j++) {
						Node param = params.item(j);
						switch(param.getNodeName().toLowerCase()) {
						case "position_local":
							parsePositionLocal(param, fiducial);
							break;
						}
					}
				}
				list.add(fiducial);
				break;
			}
		}

		return list;
	}

	private void parsePositionLocal(Node p, AbstractScenarioItem item ) {

		item.setPositionLocal(
				parseFloatAttribute(p,"x"),
				parseFloatAttribute(p,"y"),
				parseFloatAttribute(p,"z"),
				parseFloatAttribute(p,"w")
				);
	}

	private float parseFloatAttribute(Node p, String attributeName) {
		try {
			return Float.parseFloat(p.getAttributes().getNamedItem(attributeName).getNodeValue());
		} catch(NullPointerException n) {
		}
		return Float.NaN;
	}

	//	private void parsePositionGlobal(Node p, AbstractScenarioItem item ) {
	//		item.setPositionGlobal(
	//				Float.parseFloat(p.getAttributes().getNamedItem("lat").getNodeValue()),
	//				Float.parseFloat(p.getAttributes().getNamedItem("lon").getNodeValue()),
	//				Float.parseFloat(p.getAttributes().getNamedItem("alt").getNodeValue()),
	//				Float.parseFloat(p.getAttributes().getNamedItem("yaw").getNodeValue())
	//				);
	//	}



}
