

#include "urn_jaus_jss_ugv_PlatformSpecifications/PlatformSpecificationsService_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <cmath>



using namespace JTS;

namespace urn_jaus_jss_ugv_PlatformSpecifications
{



PlatformSpecificationsService_ReceiveFSM::PlatformSpecificationsService_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("PlatformSpecificationsService"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PlatformSpecificationsService_ReceiveFSMContext(*this);

	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;

      	specifics_ackermann_steering_angle_range = NAN;
	specifics_ackermann_min_turn_radius = NAN;
        specifics_ackermann_wheel_separation = NAN;
        specifics_ackermann_drive_wheel_radius = NAN;
        specifics_skidsteer_track_separation = NAN;
        specifics_skidsteer_drive_wheel_radius = NAN;
      	inertial_maximum_forward_speed = NAN;
        inertial_maximum_reverse_speed = NAN;
        inertial_maximum_rotational_speed = NAN;
        inertial_maximum_forward_acceleration = NAN;
        inertial_maximum_reverse_acceleration = NAN;
        inertial_maximum_forward_deceleration = NAN;
        inertial_maximum_reverse_deceleration = NAN;
      	spec_mobility_platform_name = "unset";
        spec_front = NAN;
        spec_back = NAN;
        spec_right = NAN;
        spec_left = NAN;
        spec_bottom = NAN;
        spec_top = NAN;
        spec_xcg = NAN;
        spec_ycg = NAN;
        spec_zcg = NAN;
        spec_wheel_base = NAN;
        spec_static_pitch_over = NAN;
        spec_static_roll_over = NAN;
        spec_vehicle_weight = NAN;
        spec_approach_angle = NAN;
        spec_departure_angle = NAN;
        spec_break_over_angle = NAN;

}


PlatformSpecificationsService_ReceiveFSM::~PlatformSpecificationsService_ReceiveFSM()
{
	delete context;
}

void PlatformSpecificationsService_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PlatformSpecificationsService_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PlatformSpecificationsService_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "PlatformSpecificationsService_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "PlatformSpecificationsService_ReceiveFSM");

}


void PlatformSpecificationsService_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PlatformSpecificationsService");
	// ACKERMANN
	cfg.declare_param<double>("specifics.ackermann.steering_angle_range", specifics_ackermann_steering_angle_range, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians] The maximum achievable steering angles, as measured by a virtual wheel at the center of the front axle. Negative values cause the vehicle to turn left; positive values cause the vehicle to turn right.",
		"Default: NAN");
	cfg.param<double>("specifics.ackermann.steering_angle_range", specifics_ackermann_steering_angle_range, specifics_ackermann_steering_angle_range);
	if (!std::isnan(specifics_ackermann_steering_angle_range))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setSteeringAngleRange(specifics_ackermann_steering_angle_range);
	cfg.declare_param<double>("specifics.ackermann.min_turn_radius", specifics_ackermann_min_turn_radius, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] The radius of the smallest turn the vehicle is able to make.",
		"Default: NAN");
	cfg.param<double>("specifics.ackermann.min_turn_radius", specifics_ackermann_min_turn_radius, specifics_ackermann_min_turn_radius);
	if (!std::isnan(specifics_ackermann_min_turn_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setMinTurnRadius(specifics_ackermann_min_turn_radius);
	cfg.declare_param<double>("specifics.ackermann.wheel_separation", specifics_ackermann_wheel_separation, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Distance between the centers of the left and right wheels.",
		"Default: NAN");
	cfg.param<double>("specifics.ackermann.wheel_separation", specifics_ackermann_wheel_separation, specifics_ackermann_wheel_separation);
	if (!std::isnan(specifics_ackermann_wheel_separation))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setWheelSeparation(specifics_ackermann_wheel_separation);
	cfg.declare_param<double>("specifics.ackermann.drive_wheel_radius", specifics_ackermann_drive_wheel_radius, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Radius of the drive wheel (distance from center of the drive shaft to the outer edge of the drive wheel) in meters.",
		"Default: NAN");
	cfg.param<double>("specifics.ackermann.drive_wheel_radius", specifics_ackermann_drive_wheel_radius, specifics_ackermann_drive_wheel_radius);
	if (!std::isnan(specifics_ackermann_drive_wheel_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setDriveWheelRadius(specifics_ackermann_drive_wheel_radius);
	// SKIDSTEER
	cfg.declare_param<double>("specifics.skidsteer.track_separation", specifics_skidsteer_track_separation, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"// [meter] Distance between the two tracks.",
		"Default: NAN");
	cfg.param<double>("specifics.skidsteer.track_separation", specifics_skidsteer_track_separation, specifics_skidsteer_track_separation);
	if (!std::isnan(specifics_skidsteer_track_separation))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getSkidsteer()->setTrackSeparation(specifics_skidsteer_track_separation);
	cfg.declare_param<double>("specifics.skidsteer.drive_wheel_radius", specifics_skidsteer_drive_wheel_radius, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Radius of the drive wheel (distance from center of the drive shaft to the outer edge of the drive wheel) in meters.",
		"Default: NAN");
	cfg.param<double>("specifics.skidsteer.drive_wheel_radius", specifics_skidsteer_drive_wheel_radius, specifics_skidsteer_drive_wheel_radius);
	if (!std::isnan(specifics_skidsteer_drive_wheel_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getSkidsteer()->setDriveWheelRadius(specifics_skidsteer_drive_wheel_radius);
	// INERTIAL
	cfg.declare_param<double>("inertial.maximum_forward_speed", inertial_maximum_forward_speed, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_forward_speed", inertial_maximum_forward_speed, inertial_maximum_forward_speed);
	if (!std::isnan(inertial_maximum_forward_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardSpeed(inertial_maximum_forward_speed);
	cfg.declare_param<double>("inertial.maximum_reverse_speed", inertial_maximum_reverse_speed, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_reverse_speed", inertial_maximum_reverse_speed, inertial_maximum_reverse_speed);
	if (!std::isnan(inertial_maximum_reverse_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseSpeed(inertial_maximum_reverse_speed);
	cfg.declare_param<double>("inertial.maximum_rotational_speed", inertial_maximum_rotational_speed, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians per second]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_rotational_speed", inertial_maximum_rotational_speed, inertial_maximum_rotational_speed);
	if (!std::isnan(inertial_maximum_rotational_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumRotationalSpeed(inertial_maximum_rotational_speed);
	cfg.declare_param<double>("inertial.maximum_forward_acceleration", inertial_maximum_forward_acceleration, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second squared]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_forward_acceleration", inertial_maximum_forward_acceleration, inertial_maximum_forward_acceleration);
	if (!std::isnan(inertial_maximum_forward_acceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardAcceleration(inertial_maximum_forward_acceleration);
	cfg.declare_param<double>("inertial.maximum_reverse_acceleration", inertial_maximum_reverse_acceleration, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second squared]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_reverse_acceleration", inertial_maximum_reverse_acceleration, inertial_maximum_reverse_acceleration);
	if (!std::isnan(inertial_maximum_reverse_acceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseAcceleration(inertial_maximum_reverse_acceleration);
	cfg.declare_param<double>("inertial.maximum_forward_deceleration", inertial_maximum_forward_deceleration, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second squared]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_forward_deceleration", inertial_maximum_forward_deceleration, inertial_maximum_forward_deceleration);
	if (!std::isnan(inertial_maximum_forward_deceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardDeceleration(inertial_maximum_forward_deceleration);
	cfg.declare_param<double>("inertial.maximum_reverse_deceleration", inertial_maximum_reverse_deceleration, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meters per second squared]",
		"Default: NAN");
	cfg.param<double>("inertial.maximum_reverse_deceleration", inertial_maximum_reverse_deceleration, inertial_maximum_reverse_deceleration);
	if (!std::isnan(inertial_maximum_reverse_deceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseDeceleration(inertial_maximum_reverse_deceleration);
	// SPEC
	cfg.declare_param<std::string>("spec.mobility_platform_name", spec_mobility_platform_name, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"A human-readable string for the vehicle.",
		"Default: unset");
	cfg.param<std::string>("spec.mobility_platform_name", spec_mobility_platform_name, spec_mobility_platform_name);
	report.getBody()->getPlatformData()->getPlatformSpec()->setMobilityPlatformName(spec_mobility_platform_name);
	cfg.declare_param<double>("spec.front", spec_front, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.front", spec_front, spec_front);
	if (!std::isnan(spec_front))
		report.getBody()->getPlatformData()->getPlatformSpec()->setFront(spec_front);
	cfg.declare_param<double>("spec.back", spec_back, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.back", spec_back, spec_back);
	if (!std::isnan(spec_back))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBack(spec_back);
	cfg.declare_param<double>("spec.right", spec_right, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.right", spec_right, spec_right);
	if (!std::isnan(spec_right))
		report.getBody()->getPlatformData()->getPlatformSpec()->setRight(spec_right);
	cfg.declare_param<double>("spec.left", spec_left, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.left", spec_left, spec_left);
	if (!std::isnan(spec_left))
		report.getBody()->getPlatformData()->getPlatformSpec()->setLeft(spec_left);
	cfg.declare_param<double>("spec.bottom", spec_bottom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.bottom", spec_bottom, spec_bottom);
	if (!std::isnan(spec_bottom))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBottom(spec_bottom);
	cfg.declare_param<double>("spec.top", spec_top, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter]",
		"Default: NAN");
	cfg.param<double>("spec.top", spec_top, spec_top);
	if (!std::isnan(spec_top))
		report.getBody()->getPlatformData()->getPlatformSpec()->setTop(spec_top);
	cfg.declare_param<double>("spec.xcg", spec_xcg, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Measured from the vehicle coordinate frame.",
		"Default: NAN");
	cfg.param<double>("spec.xcg", spec_xcg, spec_xcg);
	if (!std::isnan(spec_xcg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setXcg(spec_xcg);
	cfg.declare_param<double>("spec.ycg", spec_ycg, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Measured from the vehicle coordinate frame.",
		"Default: NAN");
	cfg.param<double>("spec.ycg", spec_ycg, spec_ycg);
	if (!std::isnan(spec_ycg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setYcg(spec_ycg);
	cfg.declare_param<double>("spec.zcg", spec_zcg, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Measured from the vehicle coordinate frame.",
		"Default: NAN");
	cfg.param<double>("spec.zcg", spec_zcg, spec_zcg);
	if (!std::isnan(spec_zcg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setZcg(spec_zcg);
	cfg.declare_param<double>("spec.wheel_base", spec_wheel_base, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[meter] Distance between the center of the frontmost wheel and the center of the rearmost wheel.",
		"Default: NAN");
	cfg.param<double>("spec.wheel_base", spec_wheel_base, spec_wheel_base);
	if (!std::isnan(spec_wheel_base))
		report.getBody()->getPlatformData()->getPlatformSpec()->setWheelBase(spec_wheel_base);
	cfg.declare_param<double>("spec.static_pitch_over", spec_static_pitch_over, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians]",
		"Default: NAN");
	cfg.param<double>("spec.static_pitch_over", spec_static_pitch_over, spec_static_pitch_over);
	if (!std::isnan(spec_static_pitch_over))
		report.getBody()->getPlatformData()->getPlatformSpec()->setStaticPitchOver(spec_static_pitch_over);
	cfg.declare_param<double>("spec.static_roll_over", spec_static_roll_over, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians]",
		"Default: NAN");
	cfg.param<double>("spec.static_roll_over", spec_static_roll_over, spec_static_roll_over);
	if (!std::isnan(spec_static_roll_over))
		report.getBody()->getPlatformData()->getPlatformSpec()->setStaticRollOver(spec_static_roll_over);
	cfg.declare_param<double>("spec.vehicle_weight", spec_vehicle_weight, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[kilogram]",
		"Default: NAN");
	cfg.param<double>("spec.vehicle_weight", spec_vehicle_weight, spec_vehicle_weight);
	if (!std::isnan(spec_vehicle_weight))
		report.getBody()->getPlatformData()->getPlatformSpec()->setVehicleWeight(spec_vehicle_weight);
	cfg.declare_param<double>("spec.approach_angle", spec_approach_angle, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.",
		"Default: NAN");
	cfg.param<double>("spec.approach_angle", spec_approach_angle, spec_approach_angle);
	if (!std::isnan(spec_approach_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setApproachAngle(spec_approach_angle);
	cfg.declare_param<double>("spec.departure_angle", spec_departure_angle, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.",
		"Default: NAN");
	cfg.param<double>("spec.departure_angle", spec_departure_angle, spec_departure_angle);
	if (!std::isnan(spec_departure_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setDepartureAngle(spec_departure_angle);
	cfg.declare_param<double>("spec.break_over_angle", spec_break_over_angle, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"[radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.",
		"Default: NAN");
	cfg.param<double>("spec.break_over_angle", spec_break_over_angle, spec_break_over_angle);
	if (!std::isnan(spec_break_over_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBreakOverAngle(spec_break_over_angle);
}



void PlatformSpecificationsService_ReceiveFSM::sendReportPlatformSpecificationsAction(QueryPlatformSpecifications msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger,  "send ReportPlatformSpecifications to %s", sender.str().c_str());
	sendJausMessage(report, transportData.getAddress());
}





}
