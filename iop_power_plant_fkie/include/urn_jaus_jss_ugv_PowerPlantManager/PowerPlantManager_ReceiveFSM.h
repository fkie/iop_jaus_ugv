

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

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include "PowerPlantManager_ReceiveFSM_sm.h"

namespace urn_jaus_jss_ugv_PowerPlantManager
{

class DllExport PowerPlantManager_ReceiveFSM : public JTS::StateMachine
{
public:
	PowerPlantManager_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~PowerPlantManager_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	ReportPowerPlantStatus p_report_status;

	bool p_battery_supported;
	bool p_voltage_received;
	int p_battery_id;
	std::string p_battery_name;
	int p_battery_max_volt;
	double p_battery_voltage;
	int p_battery_capacity_percent;
	ros::Subscriber p_sub_battery_voltage;
	ros::Subscriber p_sub_battery_capacity_percent;
	void p_ros_battery_voltage(const std_msgs::Float32::ConstPtr& msg);
	void p_ros_battery_capacity_percent(const std_msgs::Int8::ConstPtr& msg);

};

};

#endif // PowerPlantManager_RECEIVEFSM_H
