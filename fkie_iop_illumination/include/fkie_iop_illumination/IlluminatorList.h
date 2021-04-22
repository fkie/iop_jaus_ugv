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


#ifndef ILLUMINATORLIST_H
#define ILLUMINATORLIST_H

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "urn_jaus_jss_ugv_IlluminationService/Messages/ReportIlluminationState.h"
#include "urn_jaus_jss_ugv_IlluminationService/Messages/ReportIlluminationConfiguration.h"
#include "urn_jaus_jss_ugv_IlluminationService/Messages/SetIlluminationState.h"
#include "Illuminator.h"

namespace iop
{

class IlluminatorList
{
public:
	static void parse_illuminator_def(std::string& value, bool& supported, bool& state);
	IlluminatorList();
	~IlluminatorList();
	void init(std::shared_ptr<iop::Component> cmp);
	urn_jaus_jss_ugv_IlluminationService::ReportIlluminationConfiguration get_configuration_report();
	urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState get_state_report();
	void set_state(urn_jaus_jss_ugv_IlluminationService::SetIlluminationState::body::illuminationRec::illumination &state);

	template<class T>
	void set_state_callback(void(T::*handler)(urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState), T*obj) {
		p_state_callback = std::bind(handler, obj, std::placeholders::_1);
	}

protected:
	bool p_initialized;
	std::map<std::string, iop::Illuminator*> p_illuminator_map;
//	rclcpp::Subscription p_sub_diagnostic;
	std::function<void (urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState)> p_state_callback;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;  //lock_type lock(p_mutex);

	void p_clear_map();
	jUnsignedInteger p_get_illuminator_state(std::string iop_key);
	jUnsignedInteger p_get_illuminator_support(std::string iop_key);
	void p_illuminator_state_callback(std::string iop_key, bool state);
	// void p_ros_diagnostic_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr state);
};

}

#endif
