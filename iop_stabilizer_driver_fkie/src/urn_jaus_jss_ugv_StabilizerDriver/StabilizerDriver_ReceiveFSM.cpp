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


#include "urn_jaus_jss_ugv_StabilizerDriver/StabilizerDriver_ReceiveFSM.h"




using namespace JTS;

namespace urn_jaus_jss_ugv_StabilizerDriver
{



StabilizerDriver_ReceiveFSM::StabilizerDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StabilizerDriver_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
}



StabilizerDriver_ReceiveFSM::~StabilizerDriver_ReceiveFSM()
{
	delete context;
}

void StabilizerDriver_ReceiveFSM::setupNotifications()
{
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Emergency", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_StabilizerDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "StabilizerDriver_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "StabilizerDriver_ReceiveFSM");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryStabilizerPosition::ID);
	ros::NodeHandle pnh("~");
	std::string manipulator_id;
	pnh.param("max_up_angle", max_up_angle, 1.5708);
	ROS_INFO("max_up_angle: %.4f", max_up_angle);
	pnh.param("max_down_angle", max_down_angle, -1.5708);
	ROS_INFO("max_down_angle: %.4f", max_down_angle);
	XmlRpc::XmlRpcValue v;
	pnh.param("joint_names", v, v);
	ROS_INFO("Used joint_names:");
	for(unsigned int i = 0; i < v.size(); i++) {
		p_joint_names.push_back(v[i]);
		ROS_INFO("  %s", p_joint_names[i].c_str());
	}
	// TODO: get limits and positions of each flipper from URDF
	//   integrate into iop_manipulator_core_fkie::ManipulatorUrdfReader
	for (unsigned int index = 0; index < p_joint_names.size(); index++) {
		p_joint_velocities[p_joint_names[index]] = 0.;
		p_joint_positions[p_joint_names[index]] = 0.;
	}
	ros::NodeHandle nh;
	p_sub_jointstates = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &StabilizerDriver_ReceiveFSM::pJoinStateCallback, this);
	p_pub_cmd_jointstates = nh.advertise<sensor_msgs::JointState>("cmd_joint_states", 1, false);
	p_pub_cmd_vel = nh.advertise<std_msgs::Float64MultiArray>("flipper_velocity_controller/command", 1, false);
}

void StabilizerDriver_ReceiveFSM::sendReportStabilizerCapabilitiesAction(QueryStabilizerCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
  uint16_t subsystem_id = transportData.getSrcSubsystemID();
  uint8_t node_id = transportData.getSrcNodeID();
  uint8_t component_id = transportData.getSrcComponentID();
  JausAddress sender(subsystem_id, node_id, component_id);
  ROS_DEBUG_NAMED("StabilizerDriver", "sendReportStabilizerCapabilitiesAction to %d.%d.%d",
                  subsystem_id, node_id, component_id);
  p_mutex.lock();
  ReportStabilizerCapabilities response;
  for (unsigned int index = 0; index < p_joint_names.size(); index++) {
    ReportStabilizerCapabilities::Body::StabilizerCapabilities::StabilizerCapabilitiesSeq flipper;
    flipper.getStabilizerCapabilitiesRec()->setStabilizerID(index);
    // TODO: set values from URDF
    flipper.getStabilizerCapabilitiesRec()->setMaximumUpAngle(max_up_angle);
    flipper.getStabilizerCapabilitiesRec()->setMaximumDownAngle(max_down_angle);
    response.getBody()->getStabilizerCapabilities()->addElement(flipper);
  }
  sendJausMessage(response, sender);
  p_mutex.unlock();
}

void StabilizerDriver_ReceiveFSM::sendReportStabilizerEffortAction(QueryStabilizerEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
  uint16_t subsystem_id = transportData.getSrcSubsystemID();
  uint8_t node_id = transportData.getSrcNodeID();
  uint8_t component_id = transportData.getSrcComponentID();
  JausAddress sender(subsystem_id, node_id, component_id);
  ROS_DEBUG_NAMED("StabilizerDriver", "sendReportStabilizerEffortAction to %d.%d.%d", subsystem_id, node_id, component_id);

  p_mutex.lock();
  ReportStabilizerEffort response;
  std::map<std::string, float>::iterator it_ps;
  for (unsigned int index = 0; index < p_joint_names.size(); index++) {
    ReportStabilizerEffort::Body::StabilizerEffort::StabilizerEffortRec flipper_effort;
    flipper_effort.setEffort(p_joint_velocities[p_joint_names[index]]);
    flipper_effort.setStabilizerID(index);
    response.getBody()->getStabilizerEffort()->addElement(flipper_effort);
  }
  this->sendJausMessage(response, sender);
  p_mutex.unlock();
}

void StabilizerDriver_ReceiveFSM::sendReportStabilizerPositionAction(QueryStabilizerPosition msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("StabilizerDriver", "sendReportStabilizerPositionAction to %d.%d.%d", subsystem_id, node_id, component_id);

	p_mutex.lock();
	this->sendJausMessage(p_stabilizer_position_report, sender);
	p_mutex.unlock();
}

void StabilizerDriver_ReceiveFSM::setStabilizerEffortAction(SetStabilizerEffort msg)
{
	/// Insert User Code HERE
  p_mutex.lock();
  sensor_msgs::JointState ros_msg;
  ros_msg.header.stamp = ros::Time::now();
  std_msgs::Float64MultiArray ros_msg_vel;
  // prefill the array for each flipper
  for (unsigned int index = 0; index < p_joint_names.size(); index++) {
    ros_msg_vel.data.push_back(0.);
  }
  for (unsigned int index = 0; index < msg.getBody()->getStabilizerEffort()->getNumberOfElements(); index++) {
    SetStabilizerEffort::Body::StabilizerEffort::StabilizerEffortRec *effort_rec;
    effort_rec = msg.getBody()->getStabilizerEffort()->getElement(index);
    if (effort_rec->getStabilizerID() < p_joint_names.size()) {
      std::string joint_name = p_joint_names[effort_rec->getStabilizerID()];
      double vel = effort_rec->getEffort();
      double vel_ok = (int)(vel*100)/100.;
      ros_msg.name.push_back(joint_name);
      ros_msg.velocity.push_back(vel_ok);
      ros_msg_vel.data[effort_rec->getStabilizerID()] = vel_ok;
    }
  }
  p_pub_cmd_jointstates.publish(ros_msg);
  p_pub_cmd_vel.publish(ros_msg_vel);
  p_mutex.unlock();
}

void StabilizerDriver_ReceiveFSM::setStabilizerPositionAction(SetStabilizerPosition msg)
{
	/// Insert User Code HERE
  p_mutex.lock();
  sensor_msgs::JointState ros_msg;
  ros_msg.header.stamp = ros::Time::now();
  for (unsigned int index = 0; index < msg.getBody()->getStabilizerPosition()->getNumberOfElements(); index++) {
    SetStabilizerPosition::Body::StabilizerPosition::StabilizerPositionRec *pos_rec;
    pos_rec = msg.getBody()->getStabilizerPosition()->getElement(index);
    if (pos_rec->getStabilizerID() < p_joint_names.size()) {
      std::string joint_name = p_joint_names[pos_rec->getStabilizerID()];
      double pos = pos_rec->getPosition();
      pos = (int)(pos*100)/100.;
      ros_msg.name.push_back(joint_name);
      ros_msg.position.push_back(pos);
    }
  }
  p_pub_cmd_jointstates.publish(ros_msg);
  p_mutex.unlock();
}

void StabilizerDriver_ReceiveFSM::stopMotionAction()
{
	/// Insert User Code HERE
  p_mutex.lock();
  sensor_msgs::JointState ros_msg;
  ros_msg.header.stamp = ros::Time::now();
  std_msgs::Float64MultiArray ros_msg_vel;
  for (unsigned int index = 0; index < p_joint_names.size(); index++) {
    std::string joint_name = p_joint_names[index];
    ros_msg.name.push_back(joint_name);
    ros_msg.velocity.push_back(0.);
    ros_msg_vel.data.push_back(0.);
  }
  p_pub_cmd_jointstates.publish(ros_msg);
  p_pub_cmd_vel.publish(ros_msg_vel);
  p_mutex.unlock();
}



bool StabilizerDriver_ReceiveFSM::areReachable(SetStabilizerPosition msg)
{
	/// Insert User Code HERE
  for (unsigned int index = 0; index < msg.getBody()->getStabilizerPosition()->getNumberOfElements(); index++) {
    double pos = msg.getBody()->getStabilizerPosition()->getElement(index)->getPosition();
    // TODO: use values from URDF
    if (pos < -1.57079632679 or pos > 1.3962634016) {
      return false;
    }
  }
  return true;
}

bool StabilizerDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool StabilizerDriver_ReceiveFSM::stabilizersExist(SetStabilizerEffort msg)
{
	/// Insert User Code HERE
  for (unsigned int index = 0; index < msg.getBody()->getStabilizerEffort()->getNumberOfElements(); index++) {
    unsigned char sid = msg.getBody()->getStabilizerEffort()->getElement(index)->getStabilizerID();
    if (sid >= p_joint_names.size()) {
      return false;
    }
  }
  return true;
}

bool StabilizerDriver_ReceiveFSM::stabilizersExist(SetStabilizerPosition msg)
{
	/// Insert User Code HERE
  for (unsigned int index = 0; index < msg.getBody()->getStabilizerPosition()->getNumberOfElements(); index++) {
    unsigned char sid = msg.getBody()->getStabilizerPosition()->getElement(index)->getStabilizerID();
    if (sid >= p_joint_names.size()) {
      return false;
    }
  }
  return true;
}

void StabilizerDriver_ReceiveFSM::pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  // create index map
	p_mutex.lock();
	std::map<std::string, int> indexes;
	std::vector<std::string>::iterator it_jn;
	for (it_jn=p_joint_names.begin(); it_jn != p_joint_names.end(); ++it_jn) {
		int index = -1;
		for (unsigned int i = 0; i < joint_state->name.size(); i++) {
			if (it_jn->compare(joint_state->name[i]) == 0) {
				index = i;
				break;
			}
		}
		indexes[*it_jn] = index;
	}
	// get joint positions from joint_state
	std::map<std::string, int>::iterator it_ids;
	for (it_ids=indexes.begin(); it_ids != indexes.end(); ++it_ids) {
		if (it_ids->second > -1) {
			if (joint_state->position.size() > it_ids->second) {
				p_joint_positions[it_ids->first] = joint_state->position[it_ids->second];
			} else {
				p_joint_positions[it_ids->first] = 0.;
			}
			if (joint_state->velocity.size() > it_ids->second) {
				p_joint_velocities[it_ids->first] = joint_state->velocity[it_ids->second];
			} else {
				p_joint_velocities[it_ids->first] = 0.;
			}
		  } else {
			  p_joint_positions[it_ids->first] = 0.;
			  p_joint_velocities[it_ids->first] = 0.;
		  }
	}
	while (p_stabilizer_position_report.getBody()->getStabilizerPosition()->getNumberOfElements() > 0) {
		p_stabilizer_position_report.getBody()->getStabilizerPosition()->deleteLastElement();
	}
	std::map<std::string, float>::iterator it_ps;
	for (unsigned int index = 0; index < p_joint_names.size(); index++) {
		ReportStabilizerPosition::Body::StabilizerPosition::StabilizerPositionRec flipper_pos;
		flipper_pos.setPosition(p_joint_positions[p_joint_names[index]]);
		flipper_pos.setStabilizerID(index);
		p_stabilizer_position_report.getBody()->getStabilizerPosition()->addElement(flipper_pos);
	}
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryStabilizerPosition::ID, &p_stabilizer_position_report);
//  printf("[ManipulatorJointPositionSensor] positions:\n");
//  std::map<std::string, float>::iterator it_ps;
//  for (unsigned int index = 0; index < p_joint_names.size(); index++) {
//    printf("  %s: %f\n",  p_joint_names[index].c_str(), p_joint_positions[p_joint_names[index]]);
//  }
	p_mutex.unlock();
}


};
