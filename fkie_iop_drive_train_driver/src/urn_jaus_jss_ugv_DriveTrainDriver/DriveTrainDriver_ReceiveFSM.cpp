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
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#include "urn_jaus_jss_ugv_DriveTrainDriver/DriveTrainDriver_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;

namespace urn_jaus_jss_ugv_DriveTrainDriver {

std::vector<std::string> DEFAULT_TRANSMISSIONS = { "DEFAULT", "PARK", "NEUTRAL", "REVERSE", "DRIVE", "OVERDRIVE", "L1", "L2", "L3", "L4", "L5", "L6", "L7", "L8", "L9", "L10" };
std::unordered_set<std::string> DEFAULT_TRANSMISSIONS_SET(
    DEFAULT_TRANSMISSIONS.begin(),
    DEFAULT_TRANSMISSIONS.end());

std::vector<std::string> DEFAULT_TRANSFER_CASE = { "DEFAULT", "FWD", "AUTO_4WD", "MANUAL_LOW_4WD", "MANUAL_HIGH_4WD", "LOW_AWD", "HIGH_AWD" };
std::unordered_set<std::string> DEFAULT_TRANSFER_CASE_SET(
    DEFAULT_TRANSFER_CASE.begin(),
    DEFAULT_TRANSFER_CASE.end());

std::string make_default_string(const std::vector<std::string>& vec)
{
    std::ostringstream oss;
    oss << "Default: [";

    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i != vec.size() - 1) {
            oss << ", ";
        }
    }

    oss << "]";
    return oss.str();
}

DriveTrainDriver_ReceiveFSM::DriveTrainDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
    : logger(cmp->get_logger().get_child("DriveTrainDriver"))
{

    /*
     * If there are other variables, context must be constructed last so that all
     * class variables are available if an EntryAction of the InitialState of the
     * statemachine needs them.
     */
    context = new DriveTrainDriver_ReceiveFSMContext(*this);

    this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
    this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
    this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
    this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
    this->cmp = cmp;
}

DriveTrainDriver_ReceiveFSM::~DriveTrainDriver_ReceiveFSM()
{
    delete context;
}

void DriveTrainDriver_ReceiveFSM::setupNotifications()
{
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Ready", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Failure", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_Controlled_Failure", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_Controlled_Available_Ready", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Ready", "Management_ReceiveFSM");
    pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DriveTrainDriver_ReceiveFSM_Receiving_Ready_NotControlled_Ready", "Management_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Available_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Available", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Failure", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "DriveTrainDriver_ReceiveFSM");
    registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "DriveTrainDriver_ReceiveFSM");
}

void DriveTrainDriver_ReceiveFSM::setupIopConfiguration()
{
    iop::Config cfg(cmp, "DriveTrainDriver");
    pEvents_ReceiveFSM->get_event_handler().register_query(QueryTransmissionState::ID);
    pEvents_ReceiveFSM->get_event_handler().register_query(QueryTransferCaseState::ID);
    // read configuration for transmissions
    p_supported_transmissions.clear();
    std::vector<std::string> supported_transmissions;
    cfg.param_vector<std::vector<std::string>>("supported_transmissions", supported_transmissions, supported_transmissions, false,
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
        "Specifies a list with valid gears.",
        make_default_string(DEFAULT_TRANSMISSIONS));
    for (const auto& val : supported_transmissions) {
        if (DEFAULT_TRANSMISSIONS_SET.count(val)) {
            p_supported_transmissions.push_back(val);
        } else {
            RCLCPP_WARN(logger, "Invalid transmission: %s", val.c_str());
        }
    }
    // read configuration for transfer cases
    p_supported_transfer_cases.clear();
    std::vector<std::string> supported_transfer_cases;
    cfg.param_vector<std::vector<std::string>>("supported_transfer_cases", supported_transfer_cases, supported_transfer_cases, false,
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
        "Specifies a list with valid gears.",
        make_default_string(DEFAULT_TRANSFER_CASE));
    for (const auto& val : supported_transfer_cases) {
        if (DEFAULT_TRANSFER_CASE_SET.count(val)) {
            p_supported_transfer_cases.push_back(val);
        } else {
            RCLCPP_WARN(logger, "Invalid transfer case: %s", val.c_str());
        }
    }
    if (p_supported_transmissions.size() > 0) {
        int index = pGetIndex(DEFAULT_TRANSMISSIONS, p_supported_transmissions.at(0));
        if (index != -1) {
            p_transmission_state_report.getBody()->getReportTransmissionStateRec()->setActualTransmissionState(index);
        } else {
            p_transmission_state_report.getBody()->getReportTransmissionStateRec()->setActualTransmissionState(0); // DEFAULT
        }
    }
    if (p_supported_transfer_cases.size() > 0) {
        int index = pGetIndex(DEFAULT_TRANSFER_CASE, p_supported_transfer_cases.at(0));
        if (index != -1) {
            p_transfer_case_report.getBody()->getTransferCaseState()->setTransferCaseState(index);
        } else {
            p_transfer_case_report.getBody()->getTransferCaseState()->setTransferCaseState(0); // DEFAULT
        }
    }

    for (const auto& val : p_supported_transmissions) {
        int index = pGetIndex(DEFAULT_TRANSMISSIONS, val);
        switch (index) {
        case 1: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setPARK(1);
            break;
        }
        case 2: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setNEUTRAL(1);
            break;
        }
        case 3: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setREVERSE(1);
            break;
        }
        case 4: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setDRIVE(1);
            break;
        }
        case 5: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setOVERDRIVE(1);
            break;
        }
        case 6: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL1(1);
            break;
        }
        case 7: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL2(1);
            break;
        }
        case 8: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL3(1);
            break;
        }
        case 9: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL4(1);
            break;
        }
        case 10: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL5(1);
            break;
        }
        case 11: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL6(1);
            break;
        }
        case 12: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL7(1);
            break;
        }
        case 13: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL8(1);
            break;
        }
        case 14: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL9(1);
            break;
        }
        case 15: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransmissionBF()->setL10(1);
            break;
        }
        }
    }

    for (const auto& val : p_supported_transfer_cases) {
        int index = pGetIndex(DEFAULT_TRANSFER_CASE, val);
        switch (index) {
        case 1: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setFWD(1);
        }
        case 2: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setAUTO_4WD(1);
            break;
        }
        case 3: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setMANUAL_LOW_4WD(1);
            break;
        }
        case 4: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setMANUAL_HIGH_4WD(1);
            break;
        }
        case 5: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setLOW_AWD(1);
            break;
        }
        case 6: {
            p_capabilities_report.getBody()->getTransmissionCapabilities()->getTransferCaseBF()->setHIGH_AWD(1);
            break;
        }
        }
    }

    p_sub_transmission_state = cfg.create_subscription<std_msgs::msg::String>("transmission_state", 2, std::bind(&DriveTrainDriver_ReceiveFSM::pTransmissionStateCallback, this, std::placeholders::_1));
    p_pub_cmd_transmission_state = cfg.create_publisher<std_msgs::msg::String>("cmd_transmission_state", 2);
    p_sub_transfer_case = cfg.create_subscription<std_msgs::msg::String>("transfer_case", 2, std::bind(&DriveTrainDriver_ReceiveFSM::pTransferCaseCallback, this, std::placeholders::_1));
    p_pub_cmd_transfer_case = cfg.create_publisher<std_msgs::msg::String>("cmd_transfer_case", 2);
}

void DriveTrainDriver_ReceiveFSM::sendReportTransferCaseStateAction(QueryTransferCaseState msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    uint16_t subsystem_id = transportData.getSrcSubsystemID();
    uint8_t node_id = transportData.getSrcNodeID();
    uint8_t component_id = transportData.getSrcComponentID();
    JausAddress sender(subsystem_id, node_id, component_id);
    RCLCPP_DEBUG(logger, "sendReportTransferCaseStateAction to %d.%d.%d",
        subsystem_id, node_id, component_id);
    sendJausMessage(p_transfer_case_report, sender);
}

void DriveTrainDriver_ReceiveFSM::sendReportTransmissionCapabilitiesAction(QueryTransmissionCapabilities msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    uint16_t subsystem_id = transportData.getSrcSubsystemID();
    uint8_t node_id = transportData.getSrcNodeID();
    uint8_t component_id = transportData.getSrcComponentID();
    JausAddress sender(subsystem_id, node_id, component_id);
    RCLCPP_DEBUG(logger, "sendReportTransmissionCapabilitiesAction to %d.%d.%d",
        subsystem_id, node_id, component_id);
    sendJausMessage(p_capabilities_report, sender);
}

void DriveTrainDriver_ReceiveFSM::sendReportTransmissionStateAction(QueryTransmissionState msg, Receive::Body::ReceiveRec transportData)
{
    /// Insert User Code HERE
    uint16_t subsystem_id = transportData.getSrcSubsystemID();
    uint8_t node_id = transportData.getSrcNodeID();
    uint8_t component_id = transportData.getSrcComponentID();
    JausAddress sender(subsystem_id, node_id, component_id);
    RCLCPP_DEBUG(logger, "sendReportTransmissionStateAction to %d.%d.%d",
        subsystem_id, node_id, component_id);
    sendJausMessage(p_transmission_state_report, sender);
}

void DriveTrainDriver_ReceiveFSM::setTransferCaseStateAction(SetTransferCaseState msg)
{
    /// Insert User Code HERE
    int stateIndex = msg.getBody()->getTransferCaseState()->getTransferCaseState();
    if (stateIndex < DEFAULT_TRANSFER_CASE.size()) {
        p_transfer_case_report.getBody()->getTransferCaseState()->setTransferCaseState(stateIndex);
        int supIndex = pGetIndex(p_supported_transfer_cases, DEFAULT_TRANSFER_CASE[stateIndex]);
        if (supIndex != -1) {
            std_msgs::msg::String ros_msg;
            ros_msg.data = DEFAULT_TRANSFER_CASE[stateIndex];
            p_pub_cmd_transfer_case->publish(ros_msg);
        }
    } else {
        RCLCPP_INFO(logger, "   not supported transfer case: %d", stateIndex);
    }
    pEvents_ReceiveFSM->get_event_handler().set_report(QueryTransferCaseState::ID, &p_transfer_case_report);
}

void DriveTrainDriver_ReceiveFSM::setTransmissionStateAction(SetTransmissionState msg)
{
    /// Insert User Code HERE
    int stateIndex = msg.getBody()->getTransmissionStateRec()->getTransmissionState();
    if (stateIndex < DEFAULT_TRANSMISSIONS.size()) {
        p_transmission_state_report.getBody()->getReportTransmissionStateRec()->setRequestedTransmissionState(stateIndex);
        int supIndex = pGetIndex(p_supported_transmissions, DEFAULT_TRANSMISSIONS[stateIndex]);
        if (supIndex != -1) {
            std_msgs::msg::String ros_msg;
            ros_msg.data = DEFAULT_TRANSMISSIONS[stateIndex];
            p_pub_cmd_transmission_state->publish(ros_msg);
        } else {
            RCLCPP_WARN(logger, "transmission %d not supported", stateIndex);
        }
    }
    pEvents_ReceiveFSM->get_event_handler().set_report(QueryTransmissionState::ID, &p_transmission_state_report);
}

void DriveTrainDriver_ReceiveFSM::shiftToParkAction()
{
    /// Insert User Code HERE
    p_transmission_state_report.getBody()->getReportTransmissionStateRec()->setRequestedTransmissionState(1);
    std_msgs::msg::String ros_msg;
    ros_msg.data = "PARK";
    p_pub_cmd_transmission_state->publish(ros_msg);
    pEvents_ReceiveFSM->get_event_handler().set_report(QueryTransmissionState::ID, &p_transmission_state_report);
}

bool DriveTrainDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
    //// By default, inherited guards call the parent function.
    //// This can be replaced or modified as needed.
    return pAccessControl_ReceiveFSM->isControllingClient(transportData);
}

bool DriveTrainDriver_ReceiveFSM::isPark(SetTransmissionState msg)
{
    /// Insert User Code HERE
    return true;
    // return msg.getBody()->getTransmissionStateRec()->getTransmissionState() <= 1;
}

bool DriveTrainDriver_ReceiveFSM::isSupported(SetTransferCaseState msg)
{
    /// Insert User Code HERE
    int stateIndex = msg.getBody()->getTransferCaseState()->getTransferCaseState();
    if (stateIndex < DEFAULT_TRANSFER_CASE.size()) {
        int supIndex = pGetIndex(p_supported_transfer_cases, DEFAULT_TRANSFER_CASE[stateIndex]);
        if (supIndex != -1) {
            return true;
        }
    } else {
        RCLCPP_INFO(logger, " Transfer case not supported: %d", stateIndex);
    }
    return false;
}

bool DriveTrainDriver_ReceiveFSM::isSupported(SetTransmissionState msg)
{
    /// Insert User Code HERE
    int stateIndex = msg.getBody()->getTransmissionStateRec()->getTransmissionState();
    if (stateIndex < DEFAULT_TRANSMISSIONS.size()) {
        int supIndex = pGetIndex(p_supported_transmissions, DEFAULT_TRANSMISSIONS[stateIndex]);
        if (supIndex != -1) {
            return true;
        }
    }
    RCLCPP_INFO(logger, " transmission not supported: %d", stateIndex);
    return false;
}

void DriveTrainDriver_ReceiveFSM::pTransmissionStateCallback(const std_msgs::msg::String::SharedPtr state)
{
    int stateIndex = pGetIndex(DEFAULT_TRANSMISSIONS, state->data);
    if (stateIndex != -1) {
        p_transmission_state_report.getBody()->getReportTransmissionStateRec()->setActualTransmissionState(stateIndex);
        pEvents_ReceiveFSM->get_event_handler().set_report(QueryTransmissionState::ID, &p_transmission_state_report);
    }
}

void DriveTrainDriver_ReceiveFSM::pTransferCaseCallback(const std_msgs::msg::String::SharedPtr state)
{
    int stateIndex = pGetIndex(DEFAULT_TRANSFER_CASE, state->data);
    if (stateIndex != -1) {
        p_transfer_case_report.getBody()->getTransferCaseState()->setTransferCaseState(stateIndex);
        pEvents_ReceiveFSM->get_event_handler().set_report(QueryTransferCaseState::ID, &p_transfer_case_report);
    }
}

int DriveTrainDriver_ReceiveFSM::pGetIndex(const std::vector<std::string> list, std::string value)
{
    auto it = std::find(list.begin(),
        list.end(),
        value);

    if (it != list.end()) {
        return std::distance(list.begin(), it);
    }
    return -1;
}

}
