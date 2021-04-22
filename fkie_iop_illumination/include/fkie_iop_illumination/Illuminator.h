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


#ifndef ILLUMINATOR_H
#define ILLUMINATOR_H


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fkie_iop_component/iop_component.hpp>

namespace iop
{

class Illuminator
{
public:
	static std::string get_iop_key(std::string ros_key);
	static std::string get_ros_key(std::string iop_key);

	Illuminator();

	void init(std::shared_ptr<iop::Component> cmp, std::string ros_key, std::string state, std::string diagnostic_key="");
	void init(std::shared_ptr<iop::Component> cmp, std::string ros_key, bool state, std::string diagnostic_key="");
	template<class T>
	void set_state_callback(void(T::*handler)(std::string iop_key, bool state), T*obj) {
		p_state_callback = std::bind(handler, obj, std::placeholders::_1, std::placeholders::_2);
	}
	void set_state_callback();
	bool is_valid();
	bool is_supported();
	bool get_state();
	bool set_state(bool state);
	std::string get_iop_key() { return p_iop_key; }
	std::string get_ros_key() { return p_ros_key; }
	bool operator==(Illuminator &value);
	bool operator!=(Illuminator &value);

protected:
	static std::map<std::string, std::string> p_iop_ros_map;
	static std::map<std::string, std::string> p_ros_iop_map;
	std::string p_ros_key;
	std::string p_iop_key;
	std::string p_diagnostic_key;
	bool p_supported;
	bool p_state;
	std::string p_state_str;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr p_sub_state;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pub_cmd;
	std::function<void (std::string iop_key, bool state)> p_state_callback;

	void p_ros_state_callback(const std_msgs::msg::Bool::SharedPtr state);

};

}

#endif
