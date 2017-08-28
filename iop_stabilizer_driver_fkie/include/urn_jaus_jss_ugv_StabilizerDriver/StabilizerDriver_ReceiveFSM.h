/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

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


#ifndef STABILIZERDRIVER_RECEIVEFSM_H
#define STABILIZERDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_StabilizerDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_StabilizerDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/thread/recursive_mutex.hpp>
//#include "iop_manipulator_core_fkie/ManipulatorUrdfReader.h"

#include "StabilizerDriver_ReceiveFSM_sm.h"

namespace urn_jaus_jss_ugv_StabilizerDriver
{

class DllExport StabilizerDriver_ReceiveFSM : public JTS::StateMachine
{
public:
	StabilizerDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM);
	virtual ~StabilizerDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendReportStabilizerCapabilitiesAction(QueryStabilizerCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStabilizerEffortAction(QueryStabilizerEffort msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportStabilizerPositionAction(QueryStabilizerPosition msg, Receive::Body::ReceiveRec transportData);
	virtual void setStabilizerEffortAction(SetStabilizerEffort msg);
	virtual void setStabilizerPositionAction(SetStabilizerPosition msg);
	virtual void stopMotionAction();


	/// Guard Methods
	virtual bool areReachable(SetStabilizerPosition msg);
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool stabilizersExist(SetStabilizerEffort msg);
	virtual bool stabilizersExist(SetStabilizerPosition msg);



	StabilizerDriver_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;

	boost::recursive_mutex p_mutex;
	ReportStabilizerPosition p_stabilizer_position_report;
//	iop_manipulator_core_fkie::ManipulatorUrdfReader p_urdf_reader;
	std::vector<std::string> p_joint_names;
	double max_up_angle;
	double max_down_angle;
	std::map<std::string, float> p_joint_positions;
	std::map<std::string, float> p_joint_velocities;
	ros::Subscriber p_sub_jointstates;
	ros::Publisher p_pub_cmd_jointstates;
	ros::Publisher p_pub_cmd_vel;

	void pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);

};

};

#endif // STABILIZERDRIVER_RECEIVEFSM_H
