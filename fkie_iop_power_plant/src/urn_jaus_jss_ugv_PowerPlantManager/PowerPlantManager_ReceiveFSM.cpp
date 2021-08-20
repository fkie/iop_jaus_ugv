

#include "urn_jaus_jss_ugv_PowerPlantManager/PowerPlantManager_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>




using namespace JTS;

namespace urn_jaus_jss_ugv_PowerPlantManager
{



PowerPlantManager_ReceiveFSM::PowerPlantManager_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("PowerPlantManager"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PowerPlantManager_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
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

}


void PowerPlantManager_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PowerPlantManager");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPowerPlantStatus::ID);
	// read sensor configuration
	std::vector<std::string> power_plants;
	cfg.declare_param<std::vector<std::string> >("power_plants", power_plants, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"A list with powerplants and their capabilities. Format: ID.POWERPLANT.PARAMETER.VALUE",
		"Default: []");

	cfg.param_vector<std::vector<std::string> >("power_plants", power_plants, power_plants);
	for (unsigned int i = 0; i < power_plants.size(); i++) {
		auto pp_entry = iop::split(iop::trim(power_plants[i]), '.', 4);
		if (pp_entry.size() == 4) {
			int id = std::atoi(pp_entry[0].c_str());
			std::stringstream ss;
			ss << (int)id;
			std::string idstr("powerplant_");
			idstr += ss.str();
			std::string pptype = pp_entry[1];
			std::string param = pp_entry[2];
			std::string value = pp_entry[3];
			if (pptype.compare("battery") == 0) {
				p_battery_id = id;
				p_battery_supported = true;
				if (param.compare("voltage") == 0) {
					p_battery_max_volt = std::atoi(value.c_str());
				} else {
					RCLCPP_WARN(logger, "unknown parameter '%s' for battery in 'power_plants'", param.c_str());
				}
				p_sub_battery_voltage = cfg.create_subscription<std_msgs::msg::Float32>(idstr + "/voltage", 2, std::bind(&PowerPlantManager_ReceiveFSM::p_ros_battery_voltage, this, std::placeholders::_1));
				p_sub_battery_capacity_percent = cfg.create_subscription<std_msgs::msg::Int8>(idstr + "/capacity_percent", 2, std::bind(&PowerPlantManager_ReceiveFSM::p_ros_battery_capacity_percent, this, std::placeholders::_1));
			} else {
				RCLCPP_WARN(logger, "unknown pawer_plant type '%s' in 'power_plants'", pptype.c_str());
			}
		} else {
			RCLCPP_WARN(logger, "skipped power_plant entry '%s' because of invalid format", power_plants[i].c_str());
		}
	}
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryPowerPlantStatus::ID, &p_report_status);
}

void PowerPlantManager_ReceiveFSM::sendReportPowerPlantCapabilitiesAction(QueryPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger,  "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
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
	RCLCPP_DEBUG(logger,  "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
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
	RCLCPP_DEBUG(logger,  "send ReportPowerPlantCapabilities to %s", sender.str().c_str());
	sendJausMessage(p_report_status, sender);
}

void PowerPlantManager_ReceiveFSM::setPowerPlantConfigurationAction(SetPowerPlantConfiguration msg)
{
	RCLCPP_DEBUG(logger,  "setPowerPlantConfiguration not implemented");
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

void PowerPlantManager_ReceiveFSM::p_ros_battery_voltage(const std_msgs::msg::Float32::SharedPtr msg)
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

void PowerPlantManager_ReceiveFSM::p_ros_battery_capacity_percent(const std_msgs::msg::Int8::SharedPtr msg)
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

}
