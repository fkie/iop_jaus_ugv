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

#ifndef DRIVETRAINDRIVER_RECEIVEFSM_H
#define DRIVETRAINDRIVER_RECEIVEFSM_H

#include "InternalEvents/InternalEventHandler.h"
#include "JTSStateMachine.h"
#include "JausUtils.h"
#include "Transport/JausTransport.h"
#include "urn_jaus_jss_ugv_DriveTrainDriver/InternalEvents/InternalEventsSet.h"
#include "urn_jaus_jss_ugv_DriveTrainDriver/Messages/MessageSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "DriveTrainDriver_ReceiveFSM_sm.h"
#include <fkie_iop_component/iop_component.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace urn_jaus_jss_ugv_DriveTrainDriver {

class DllExport DriveTrainDriver_ReceiveFSM : public JTS::StateMachine {
public:
    DriveTrainDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
    virtual ~DriveTrainDriver_ReceiveFSM();

    /// Handle notifications on parent state changes
    virtual void setupNotifications();
    virtual void setupIopConfiguration();

    /// Action Methods
    virtual void sendReportTransferCaseStateAction(QueryTransferCaseState msg, Receive::Body::ReceiveRec transportData);
    virtual void sendReportTransmissionCapabilitiesAction(QueryTransmissionCapabilities msg, Receive::Body::ReceiveRec transportData);
    virtual void sendReportTransmissionStateAction(QueryTransmissionState msg, Receive::Body::ReceiveRec transportData);
    virtual void setTransferCaseStateAction(SetTransferCaseState msg);
    virtual void setTransmissionStateAction(SetTransmissionState msg);
    virtual void shiftToParkAction();

    /// Guard Methods
    virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
    virtual bool isPark(SetTransmissionState msg);
    virtual bool isSupported(SetTransferCaseState msg);
    virtual bool isSupported(SetTransmissionState msg);

    DriveTrainDriver_ReceiveFSMContext* context;

protected:
    /// References to parent FSMs
    urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;
    urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
    urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
    urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

    std::shared_ptr<iop::Component> cmp;
    rclcpp::Logger logger;

    typedef std::recursive_mutex mutex_type;
    typedef std::unique_lock<mutex_type> lock_type;
    mutable mutex_type p_mutex;
    ReportTransmissionState p_transmission_state_report;
    ReportTransferCaseState p_transfer_case_report;
    ReportTransmissionCapabilities p_capabilities_report;
    std::vector<std::string> p_supported_transmissions;
    std::vector<std::string> p_supported_transfer_cases;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr p_sub_transmission_state;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_pub_cmd_transmission_state;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr p_sub_transfer_case;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_pub_cmd_transfer_case;

    void pTransmissionStateCallback(const std_msgs::msg::String::SharedPtr state);
    void pTransferCaseCallback(const std_msgs::msg::String::SharedPtr state);

    int pGetIndex(const std::vector<std::string> list, std::string value);
};

}

#endif // DRIVETRAINDRIVER_RECEIVEFSM_H
