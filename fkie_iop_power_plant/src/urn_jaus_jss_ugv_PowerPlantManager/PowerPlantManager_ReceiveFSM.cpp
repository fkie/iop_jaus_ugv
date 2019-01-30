

#include "urn_jaus_jss_ugv_PowerPlantManager/PowerPlantManager_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_ugv_PowerPlantManager
{



PowerPlantManager_ReceiveFSM::PowerPlantManager_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PowerPlantManager_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	p_battery_supported = false;
	p_battery_max_volt = 0;
	p_battery_id = 0;
	p_battery_name = "";
	p_battery_voltage = 0;
	p_battery_capacity_percent = 0;
	p_voltage_received = false;
}



PowerPlantManager_ReceiveFSM::~PowerPlantManager_ReceiveFSM()
{
	delete context;
}

void PowerPlantManager_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_PowerPlantManager_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_PowerPlantManager_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PowerPlantManager_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PowerPlantManager_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "PowerPlantManager_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "PowerPlantManager_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "PowerPlantManager_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "PowerPlantManager_ReceiveFSM");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPowerPlantStatus::ID);
	iop::Config cfg("~PowerPlantManager");
	// read sensor configuration
	XmlRpc::XmlRpcValue caps;
	cfg.param("power_plants", caps, caps);
	if (caps.valid()) {
		// parse the paramete
		for(int i = 0; i < caps.size(); i++) {
			if (caps[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
				for(XmlRpc::XmlRpcValue::ValueStruct::iterator itid = caps[i].begin(); itid != caps[i].end(); itid++) {
					int id = std::atoi(itid->first.c_str());
					std::stringstream ss;
					ss << (int)id;
					std::string idstr("powerplant_");
					idstr += ss.str();
					if (itid->second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
						for(int b = 0; b < itid->second.size(); b++) {
							if (itid->second[b].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
								for(XmlRpc::XmlRpcValue::ValueStruct::iterator itpp = itid->second[b].begin(); itpp != itid->second[b].end(); itpp++) {
									std::string pptype = itpp->first;
									if (pptype.compare("battery") == 0) {
										p_battery_id = id;
										p_battery_supported = true;
										if (itpp->second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
											for(int bp = 0; bp < itpp->second.size(); bp++) {
												for(XmlRpc::XmlRpcValue::ValueStruct::iterator itbp = itpp->second[b].begin(); itbp != itpp->second[b].end(); itbp++) {
													std::string param_name = itbp->first;
													if (param_name.compare("voltage") == 0) {
														p_battery_max_volt = static_cast<int>(itbp->second);
													}
												}
											}
										} else {
											ROS_ERROR("wrong parameter definition for battery in '~power_plants' format, expected list of: string: [parameters]");
										}
										p_sub_battery_voltage = cfg.subscribe<std_msgs::Float32>(idstr + "/voltage", 2, &PowerPlantManager_ReceiveFSM::p_ros_battery_voltage, this);
										p_sub_battery_capacity_percent = cfg.subscribe<std_msgs::Int8>(idstr + "/capacity_percent", 2, &PowerPlantManager_ReceiveFSM::p_ros_battery_capacity_percent, this);
									} else {
										ROS_ERROR("PowerPlant %s found, currently on battery is supported!", pptype.c_str());
									}
								}
							} else {
								ROS_ERROR("wrong definition for powerplant name in '~power_plants' format, expected list of: string: [parameters]");
							}
						}
					} else {
						ROS_ERROR("wrong definition for ID in '~power_plants' format, expected list of: string: [parameters]");
					}
				}
			} else {
				ROS_ERROR("wrong entry of '~power_plants' format, expected list of: ID: [POWER PANTS]");
			}
		}
	}
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPowerPlantStatus::ID, &p_report_status);
}

void PowerPlantManager_ReceiveFSM::sendReportPowerPlantCapabilitiesAction(QueryPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PowerPlantManager", "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
	ReportPowerPlantCapabilities report;
	if (p_battery_supported) {
		ReportPowerPlantCapabilities::body::powerPlantCapabilitiesList::powerPlantCapabilitiesSeq batsec;
		batsec.getPowerPlantDescRec()->setPowerPlantID(p_battery_id);
		batsec.getPowerPlantDescRec()->setDescription(p_battery_name);
		batsec.getPowerPlantCapabilitiesVar()->setFieldValue(2);
		ReportPowerPlantCapabilities::body::powerPlantCapabilitiesList::powerPlantCapabilitiesSeq::powerPlantCapabilitiesVar::batteryCapabilitiesList::batteryCapabilitiesRec batteryrec;
		batteryrec.setNominalVoltage(p_battery_max_volt);
		batsec.getPowerPlantCapabilitiesVar()->getBatteryCapabilitiesList()->addElement(batteryrec);
		report.getBody()->getPowerPlantCapabilitiesList()->addElement(batsec);
	}
	sendJausMessage(report, sender);
}

void PowerPlantManager_ReceiveFSM::sendReportPowerPlantConfigurationAction(QueryPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PowerPlantManager", "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
	ReportPowerPlantConfiguration report;
	if (p_battery_supported) {
		ReportPowerPlantConfiguration::body::powerPlantConfigurationList::powerPlantConfigurationSeq batsec;
		batsec.getPowerPlantID()->setPowerPlantID(p_battery_id);
		batsec.getPowerPlantConfigurationVar()->setFieldValue(2);
		batsec.getPowerPlantConfigurationVar()->getBatteryConfigurationRec()->setPowerState(1);
		report.getBody()->getPowerPlantConfigurationList()->addElement(batsec);
	}
	sendJausMessage(report, sender);
}

void PowerPlantManager_ReceiveFSM::sendReportPowerPlantStatusAction(QueryPowerPlantStatus msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PowerPlantManager", "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
	sendJausMessage(p_report_status, sender);
}

void PowerPlantManager_ReceiveFSM::setPowerPlantConfigurationAction(SetPowerPlantConfiguration msg)
{
	ROS_DEBUG_NAMED("PowerPlantManager", "setPowerPlantConfiguration not implemented");
}



bool PowerPlantManager_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}
bool PowerPlantManager_ReceiveFSM::isSupported(SetPowerPlantConfiguration msg)
{
	// TODO: add check for supported powerplants. Currently changes not supported!
	return false;
}

void PowerPlantManager_ReceiveFSM::p_ros_battery_voltage(const std_msgs::Float32::ConstPtr& msg)
{
	p_battery_voltage = msg->data;
	if (p_battery_supported) {
		ReportPowerPlantStatus report;
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus ppstatus;
		ppstatus.getPowerPlantDescRec()->setPowerPlantID(p_battery_id);
		ppstatus.getPowerPlantDescRec()->setDescription(p_battery_name);
		ppstatus.getPowerPlantStatusVar()->setFieldValue(2);
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus::powerPlantStatusVar::batteryStatus batlist;
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus::powerPlantStatusVar::batteryStatus::batteryStatusRec batrec;
		batrec.setVoltage(p_battery_voltage);
		batrec.setPercentChargeRemaining(p_battery_capacity_percent);
		batlist.addElement(batrec);
		ppstatus.getPowerPlantStatusVar()->setBatteryStatus(batlist);
		report.getBody()->getPowerPlantStatusList()->addElement(ppstatus);
		p_report_status = report;
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryPowerPlantStatus::ID, &p_report_status);
		p_voltage_received = true;
	}
}

void PowerPlantManager_ReceiveFSM::p_ros_battery_capacity_percent(const std_msgs::Int8::ConstPtr& msg)
{
	p_battery_capacity_percent = msg->data;
	if (p_battery_supported) {
		ReportPowerPlantStatus report;
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus ppstatus;
		ppstatus.getPowerPlantDescRec()->setPowerPlantID(p_battery_id);
		ppstatus.getPowerPlantDescRec()->setDescription(p_battery_name);
		ppstatus.getPowerPlantStatusVar()->setFieldValue(2);
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus::powerPlantStatusVar::batteryStatus batlist;
		ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus::powerPlantStatusVar::batteryStatus::batteryStatusRec batrec;
		batrec.setVoltage(p_battery_voltage);
		batrec.setPercentChargeRemaining(p_battery_capacity_percent);
		batlist.addElement(batrec);
		ppstatus.getPowerPlantStatusVar()->setBatteryStatus(batlist);
		report.getBody()->getPowerPlantStatusList()->addElement(ppstatus);
		p_report_status = report;
		if (!p_voltage_received) {
			pEvents_ReceiveFSM->get_event_handler().set_report(QueryPowerPlantStatus::ID, &p_report_status);
		} else {
			p_voltage_received = false;
		}
	}
}

};
