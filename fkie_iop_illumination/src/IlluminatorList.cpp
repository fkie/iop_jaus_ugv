/**
ROS/IOP Bridge
Copyright (c) 2018 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include <fkie_iop_illumination/IlluminatorList.h>
#include <fkie_iop_component/iop_config.hpp>

using namespace iop;

void parse_illuminator_def(std::string& value, bool& supported, bool& state)
{
	supported = false;
	state = false;
	if (value.compare("no") == 0) {
		return;
	}
	if (value.compare("yes") == 0) {
		supported = true;
		state = false;
		return;
	}
	if (value.compare("OFF") == 0) {
		supported = true;
		state = false;
		return;
	}
	if (value.compare("ON") == 0) {
		supported = true;
		state = true;
		return;
	}
}

IlluminatorList::IlluminatorList()
{
	p_initialized = false;
}

void IlluminatorList::init(std::shared_ptr<iop::Component> cmp)
{
	iop::Config cfg(cmp, "IlluminatorList");
	auto logger = cmp->get_logger().get_child("IlluminatorList");
	lock_type lock(p_mutex);
	p_illuminator_map.clear();
	p_illuminator_map["Headlights"] = new Illuminator();
	p_illuminator_map["LeftTurnSignal"]= new Illuminator();
	p_illuminator_map["RightTurnSignal"] = new Illuminator();
	p_illuminator_map["RunningLights"] = new Illuminator();
	p_illuminator_map["BrakeLights"] = new Illuminator();
	p_illuminator_map["BackupLights"] = new Illuminator();
	p_illuminator_map["VisibleLightSource"] = new Illuminator();
	p_illuminator_map["IRLightSource"] = new Illuminator();
	p_illuminator_map["VariableLight1"] = new Illuminator();
	p_illuminator_map["VariableLight2"] = new Illuminator();
	p_illuminator_map["VariableLight3"] = new Illuminator();
	p_illuminator_map["VariableLight4"] = new Illuminator();
	p_illuminator_map["HighBeams"] = new Illuminator();
	p_illuminator_map["ParkingLights"] = new Illuminator();
	p_illuminator_map["FogLights"] = new Illuminator();
	p_illuminator_map["HazardLights"] = new Illuminator();

	std::string value = "no";
	cfg.declare_param<std::string>("head_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Headlights", "Default: no");
	cfg.declare_param<std::string>("left_turn_signal", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"LeftTurnSignal", "Default: no");
	cfg.declare_param<std::string>("right_turn_signal", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"RightTurnSignal", "Default: no");
	cfg.declare_param<std::string>("running_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"RunningLights", "Default: no");
	cfg.declare_param<std::string>("brake_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"BrakeLights", "Default: no");
	cfg.declare_param<std::string>("backup_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"BackupLights", "Default: no");
	cfg.declare_param<std::string>("visible_light_source", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"VisibleLightSource", "Default: no");
	cfg.declare_param<std::string>("ir_light_source", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"IRLightSource", "Default: no");
	cfg.declare_param<std::string>("variable_light_1", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"VariableLight1", "Default: no");
	cfg.declare_param<std::string>("variable_light_2", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"VariableLight2", "Default: no");
	cfg.declare_param<std::string>("variable_light_3", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"VariableLight3", "Default: no");
	cfg.declare_param<std::string>("variable_light_4", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"VariableLight4", "Default: no");
	cfg.declare_param<std::string>("high_beams", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"HighBeams", "Default: no");
	cfg.declare_param<std::string>("parking_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"ParkingLights", "Default: no");
	cfg.declare_param<std::string>("fog_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"FogLights", "Default: no");
	cfg.declare_param<std::string>("hazard_lights", value, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"HazardLights", "Default: no");

	value = "no";
	bool supported = false;
	bool state = false;
	cfg.param<std::string>("head_lights", value, value);
	cfg.param<std::string>("left_turn_signal", value, value);
	cfg.param<std::string>("right_turn_signal", value, value);
	cfg.param<std::string>("running_lights", value, value);
	cfg.param<std::string>("brake_lights", value, value);
	cfg.param<std::string>("backup_lights", value, value);
	cfg.param<std::string>("visible_light_source", value, value);
	cfg.param<std::string>("ir_light_source", value, value);
	cfg.param<std::string>("variable_light_1", value, value);
	cfg.param<std::string>("variable_light_2", value, value);
	cfg.param<std::string>("variable_light_3", value, value);
	cfg.param<std::string>("variable_light_4", value, value);
	cfg.param<std::string>("high_beams", value, value);
	cfg.param<std::string>("parking_lights", value, value);
	cfg.param<std::string>("fog_lights", value, value);
	cfg.param<std::string>("hazard_lights", value, value);

	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["Headlights"]->init(cmp, "head_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["LeftTurnSignal"]->init(cmp, "left_turn_signal", state, "LeftTurnSignal");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["RightTurnSignal"]->init(cmp, "right_turn_signal", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["RunningLights"]->init(cmp, "running_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["BrakeLights"]->init(cmp, "brake_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["BackupLights"]->init(cmp, "backup_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["VisibleLightSource"]->init(cmp, "visible_light_source", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["IRLightSource"]->init(cmp, "ir_light_source", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["VariableLight1"]->init(cmp, "variable_light_1", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["VariableLight2"]->init(cmp, "variable_light_2", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["VariableLight3"]->init(cmp, "variable_light_3", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["VariableLight4"]->init(cmp, "variable_light_4", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["HighBeams"]->init(cmp, "high_beams", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["ParkingLights"]->init(cmp, "parking_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["FogLights"]->init(cmp, "fog_lights", state, "Headlights");
	parse_illuminator_def(value, supported, state);
	if (supported) p_illuminator_map["HazardLights"]->init(cmp, "hazard_lights", state, "Headlights");
	
	// set callback after all states are set
	p_illuminator_map["Headlights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["LeftTurnSignal"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["RightTurnSignal"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["RunningLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["BrakeLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["BackupLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["VisibleLightSource"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["IRLightSource"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["VariableLight1"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["VariableLight2"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["VariableLight3"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["VariableLight4"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["HighBeams"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["ParkingLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["FogLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
	p_illuminator_map["HazardLights"]->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);

	p_initialized = true;
}

IlluminatorList::~IlluminatorList()
{
	p_clear_map();
}

void IlluminatorList::p_clear_map() {
	lock_type lock(p_mutex);
	std::map<std::string, iop::Illuminator*>::iterator it;
	for (it = p_illuminator_map.begin(); it != p_illuminator_map.end(); ++it) {
		delete it->second;
	}
	p_illuminator_map.clear();
}

urn_jaus_jss_ugv_IlluminationService::ReportIlluminationConfiguration IlluminatorList::get_configuration_report()
{
	lock_type lock(p_mutex);
	urn_jaus_jss_ugv_IlluminationService::ReportIlluminationConfiguration config;
	if (p_initialized) {
		config.getBody()->getIlluminatorTypes()->getTypes()->setHeadlights(p_get_illuminator_support("Headlights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setLeftTurnSignal(p_get_illuminator_support("LeftTurnSignal"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setRightTurnSignal(p_get_illuminator_support("RightTurnSignal"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setRunningLights(p_get_illuminator_support("RunningLights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setBrakeLights(p_get_illuminator_support("BrakeLights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setBackupLights(p_get_illuminator_support("BackupLights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setVisibleLightSource(p_get_illuminator_support("VisibleLightSource"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setIRLightSource(p_get_illuminator_support("IRLightSource"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setVariableLight1(p_get_illuminator_support("VariableLight1"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setVariableLight2(p_get_illuminator_support("VariableLight2"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setVariableLight3(p_get_illuminator_support("VariableLight3"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setVariableLight4(p_get_illuminator_support("VariableLight4"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setHighBeams(p_get_illuminator_support("HighBeams"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setParkingLights(p_get_illuminator_support("ParkingLights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setFogLights(p_get_illuminator_support("FogLights"));
		config.getBody()->getIlluminatorTypes()->getTypes()->setHazardLights(p_get_illuminator_support("HazardLights"));
	}
	return config;
}

urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState IlluminatorList::get_state_report()
{
	lock_type lock(p_mutex);
	urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState report;
	if (p_initialized) {
		report.getBody()->getIlluminationRec()->getIllumination()->setHeadlights(p_get_illuminator_state("Headlights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setLeftTurnSignal(p_get_illuminator_state("LeftTurnSignal"));
		report.getBody()->getIlluminationRec()->getIllumination()->setRightTurnSignal(p_get_illuminator_state("RightTurnSignal"));
		report.getBody()->getIlluminationRec()->getIllumination()->setRunningLights(p_get_illuminator_state("RunningLights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setBrakeLights(p_get_illuminator_state("BrakeLights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setBackupLights(p_get_illuminator_state("BackupLights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setVisibleLightSource(p_get_illuminator_state("VisibleLightSource"));
		report.getBody()->getIlluminationRec()->getIllumination()->setIRLightSource(p_get_illuminator_state("IRLightSource"));
		report.getBody()->getIlluminationRec()->getIllumination()->setVariableLight1(p_get_illuminator_state("VariableLight1"));
		report.getBody()->getIlluminationRec()->getIllumination()->setVariableLight2(p_get_illuminator_state("VariableLight2"));
		report.getBody()->getIlluminationRec()->getIllumination()->setVariableLight3(p_get_illuminator_state("VariableLight3"));
		report.getBody()->getIlluminationRec()->getIllumination()->setVariableLight4(p_get_illuminator_state("VariableLight4"));
		report.getBody()->getIlluminationRec()->getIllumination()->setHighBeams(p_get_illuminator_state("HighBeams"));
		report.getBody()->getIlluminationRec()->getIllumination()->setParkingLights(p_get_illuminator_state("ParkingLights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setFogLights(p_get_illuminator_state("FogLights"));
		report.getBody()->getIlluminationRec()->getIllumination()->setHazardLights(p_get_illuminator_state("HazardLights"));
	}
	return report;
}

void IlluminatorList::set_state(urn_jaus_jss_ugv_IlluminationService::SetIlluminationState::body::illuminationRec::illumination &state)
{
	lock_type lock(p_mutex);
	if (p_initialized) {
		p_illuminator_map["Headlights"]->set_state(state.getHeadlights());
		p_illuminator_map["LeftTurnSignal"]->set_state(state.getLeftTurnSignal());
		p_illuminator_map["RightTurnSignal"]->set_state(state.getRightTurnSignal());
		p_illuminator_map["RunningLights"]->set_state(state.getRunningLights());
		p_illuminator_map["BrakeLights"]->set_state(state.getBrakeLights());
		p_illuminator_map["BackupLights"]->set_state(state.getBackupLights());
		p_illuminator_map["VisibleLightSource"]->set_state(state.getVisibleLightSource());
		p_illuminator_map["IRLightSource"]->set_state(state.getIRLightSource());
		p_illuminator_map["VariableLight1"]->set_state(state.getVariableLight1());
		p_illuminator_map["VariableLight2"]->set_state(state.getVariableLight2());
		p_illuminator_map["VariableLight3"]->set_state(state.getVariableLight3());
		p_illuminator_map["VariableLight4"]->set_state(state.getVariableLight4());
		p_illuminator_map["HighBeams"]->set_state(state.getHighBeams());
		p_illuminator_map["ParkingLights"]->set_state(state.getParkingLights());
		p_illuminator_map["FogLights"]->set_state(state.getFogLights());
		p_illuminator_map["HazardLights"]->set_state(state.getHazardLights());
	}
}

void IlluminatorList::p_illuminator_state_callback(std::string /* iop_key */, bool /* state */)
{
	lock_type lock(p_mutex);
	if (p_state_callback) {
		p_state_callback(get_state_report());
	}
}

// void IlluminatorList::p_ros_diagnostic_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr state)
// {
// 	//TODO: update current state
// }

jUnsignedInteger IlluminatorList::p_get_illuminator_state(std::string iop_key)
{
	jUnsignedInteger result = 0;
	iop::Illuminator* il = p_illuminator_map[iop_key];
	if (il->is_supported() && il->get_state()) {
		result = 1;
	}
	return result;
}

jUnsignedInteger IlluminatorList::p_get_illuminator_support(std::string iop_key)
{
	jUnsignedInteger result = 0;
	iop::Illuminator* il = p_illuminator_map[iop_key];
	if (il->is_supported()) {
		result = 1;
	}
	return result;
}
