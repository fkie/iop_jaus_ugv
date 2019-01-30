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
#include <fkie_iop_component/iop_config.h>

using namespace iop;


IlluminatorList::IlluminatorList()
{
	p_initialized = false;
}

void IlluminatorList::init()
{
	iop::Config cfg("~Illumination");
	XmlRpc::XmlRpcValue illuminations;
	cfg.param("illuminations", illuminations, illuminations);
	if (!illuminations.valid()) {
		ROS_ERROR("wrong '~illuminations' format, expected list of ROS_KEY: STATE{ON, OFF, 0, 1} or [STATE{ON, OFF, 0, 1}, DIAGNOSTIC_KEY]");
		ROS_BREAK();
	}
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

	// parse the parameter
	for(int i = 0; i < illuminations.size(); i++) {
		if (illuminations[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = illuminations[i].begin(); iterator != illuminations[i].end(); iterator++) {
				std::string il_ros_type = iterator->first;
				bool supported = false;
				bool state = false;
				std::string il_diag_type = "";
				XmlRpc::XmlRpcValue il_state_struct = iterator->second;
				if (il_state_struct.valid()) {
					if (il_state_struct.getType() == XmlRpc::XmlRpcValue::TypeArray) {
						if (il_state_struct.size() > 0) {
							if (il_state_struct[0].getType() == XmlRpc::XmlRpcValue::TypeString) {
								supported = true;
								std::string normstr = il_state_struct[0];
								std::transform(normstr.begin(), normstr.end(), normstr.begin(), ::tolower);
								state = normstr.compare("on") == 0;
							} else if (il_state_struct[0].getType() == XmlRpc::XmlRpcValue::TypeInt) {
								supported = (int)il_state_struct[0] > 0;
							} else if (il_state_struct[0].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
								supported = true;
								state = (bool)il_state_struct[0];
							}
						}
						if (il_state_struct.size() > 1) {
							il_diag_type = std::string(il_state_struct[1]);
						}
					} else if (il_state_struct.getType() == XmlRpc::XmlRpcValue::TypeString) {
						supported = true;
						std::string normstr = il_state_struct;
						std::transform(normstr.begin(), normstr.end(), normstr.begin(), ::tolower);
						state = normstr.compare("on") == 0;
					} else if (il_state_struct.getType() == XmlRpc::XmlRpcValue::TypeInt) {
						supported = (int)il_state_struct > 0;
					} else if (il_state_struct.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
						supported = true;
						state = (bool)il_state_struct;
					}
				} else {
					ROS_WARN("invalid value for illumination key '%s', ignored!", il_ros_type.c_str());
				}
				// add supported key
				if (supported) {
					try {
						std::string il_iop_type = Illuminator::get_iop_key(il_ros_type);
						iop::Illuminator* illuminator = p_illuminator_map[il_iop_type];
						illuminator->set_state_callback(&IlluminatorList::p_illuminator_state_callback, this);
						illuminator->init(il_ros_type, state, il_diag_type);
					} catch (std::exception &e) {
						ROS_WARN("invalid key value for illumination '%s', ignored!", il_ros_type.c_str());
					}
				}
			}
		} else {
			ROS_ERROR("wrong entry of '~illuminations', expected list of ROS_KEY: STATE{ON, OFF, 0, 1} or [STATE{ON, OFF, 0, 1}, DIAGNOSTIC_KEY]");
		}
	}
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

void IlluminatorList::p_illuminator_state_callback(std::string iop_key, bool state)
{
	lock_type lock(p_mutex);
	if (p_state_callback) {
		p_state_callback(get_state_report());
	}
}

void IlluminatorList::p_ros_diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& state)
{
	//TODO: update current state
}

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
