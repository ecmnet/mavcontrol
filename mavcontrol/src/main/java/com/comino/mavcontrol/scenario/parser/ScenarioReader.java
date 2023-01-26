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
import com.comino.mavcontrol.scenario.items.LogMessageItem;
import com.comino.mavcontrol.scenario.items.MoveToItem;
import com.comino.mavcontrol.scenario.items.ObstacleItem;
import com.comino.mavcontrol.scenario.items.PauseItem;
import com.comino.mavcontrol.scenario.items.PrecisionLandItem;
import com.comino.mavcontrol.scenario.items.RotateItem;
import com.comino.mavcontrol.scenario.items.SearchObjectItem;
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
				break;
			case "name":
				s.setName(scenario_child.getTextContent().trim());
				break;
			case "description":
				break;
			case "max_speed":
				s.setMaxSpeed(Float.parseFloat(scenario_child.getTextContent()));
				break;
			case "time_factor":
				s.setTimeFactor(Float.parseFloat(scenario_child.getTextContent()));
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
			case "loop":
				int count = Integer.parseInt(step.getAttributes().getNamedItem("c").getNodeValue());
				if(count > 0) {
					NodeList loop_steps = step.getChildNodes();
					for(int j = 0; j < count; j++) {
						LinkedList<AbstractScenarioItem> loop_list = parseSteps(loop_steps);
						list.addAll(loop_list);
					}
				}
				break;
			case "include":
				Scenario include = readScenario(step.getAttributes().getNamedItem("f").getNodeValue());
				if(include!=null) {
					list.addAll(include.getList());
				}
				break;
			default:
				AbstractScenarioItem item = parseStep(step);
				if(item!=null)
					list.add(parseStep(step));
			}

		}

		return list;
	}


	private AbstractScenarioItem parseStep(Node step) {

		AbstractScenarioItem item = null;

		switch(step.getNodeName().toLowerCase()) {
		case "search_object":
			SearchObjectItem search = new SearchObjectItem(control);
			item = search;
			break;	
		case "log_message":
			LogMessageItem log = new LogMessageItem(control);
			log.setMessage(step.getAttributes().getNamedItem("m").getNodeValue(), 
					step.getAttributes().getNamedItem("t").getNodeValue().charAt(0));
			item = log;
			break;	
		case "arm":
			ArmingItem arming = new ArmingItem(control);
			item = arming;
			break;	
		case "disarm":
			DisarmingItem disarming = new DisarmingItem(control);
			item = disarming;
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
					case "delay":
						takeoff.setDelay((int)parseFloatAttribute(param,"d"));
					}
				}
			}
			item = takeoff;
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
						moveto.setType(MoveToItem.TYPE_ABSOLUTE);
						break;
					case "position_relative":
						parsePositionLocal(param, moveto);
						moveto.setType(MoveToItem.TYPE_RELATIVE);
						break;
					case "acceptance_radius":
						moveto.setAcceptanceRadius(parseFloatAttribute(param,"r"));
						break;
					case "delay":
						moveto.setDelay((int)parseFloatAttribute(param,"d"));
						break;
					}
				}
				item = moveto;
			}
			break;
		case "pause":
			PauseItem pause = new PauseItem(control);
			if(step.hasChildNodes()) {
				NodeList params = step.getChildNodes();
				for(int j=0;j<params.getLength();j++) {
					Node param = params.item(j);
					switch(param.getNodeName().toLowerCase()) {
					case "delay":
						pause.setDelay((int)parseFloatAttribute(param,"d"));
						item = pause;
						break;
					}
				}
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
				item = rotate;
			}
			break;
		case "precision_land":
			PrecisionLandItem precision_land = new PrecisionLandItem(control);
			item = precision_land;
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
			item = obstacle;
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
			item = fiducial;
			break;
		}

		return item;

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
}
