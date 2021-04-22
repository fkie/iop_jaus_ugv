

#ifndef POWERPLANTMANAGER_RECEIVEFSM_H
#define POWERPLANTMANAGER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_PowerPlantManager/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_PowerPlantManager/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"

#include "PowerPlantManager_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>


namespace urn_jaus_jss_ugv_PowerPlantManager
{

class DllExport PowerPlantManager_ReceiveFSM : public JTS::StateMachine
{
public:
	PowerPlantManager_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PowerPlantManager_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void sendReportPowerPlantCapabilitiesAction(QueryPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportPowerPlantConfigurationAction(QueryPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportPowerPlantStatusAction(QueryPowerPlantStatus msg, Receive::Body::ReceiveRec transportData);
	virtual void setPowerPlantConfigurationAction(SetPowerPlantConfiguration msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isSupported(SetPowerPlantConfiguration msg);



	PowerPlantManager_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	ReportPowerPlantStatus p_report_status;

	bool p_battery_supported;
	bool p_voltage_received;
	int p_battery_id;
	std::string p_battery_name;
	int p_battery_max_volt;
	double p_battery_voltage;
	int p_battery_capacity_percent;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr p_sub_battery_voltage;
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr p_sub_battery_capacity_percent;
	void p_ros_battery_voltage(const std_msgs::msg::Float32::SharedPtr msg);
	void p_ros_battery_capacity_percent(const std_msgs::msg::Int8::SharedPtr msg);

};

}

#endif // PowerPlantManager_RECEIVEFSM_H
