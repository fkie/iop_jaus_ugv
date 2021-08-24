

#include <fkie_iop_component/iop_config.h>
#include "urn_jaus_jss_ugv_PlatformSpecifications/PlatformSpecificationsService_ReceiveFSM.h"
#include <cmath>



using namespace JTS;

namespace urn_jaus_jss_ugv_PlatformSpecifications
{



PlatformSpecificationsService_ReceiveFSM::PlatformSpecificationsService_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all 
	 * class variables are available if an EntryAction of the InitialState of the 
	 * statemachine needs them. 
	 */
	context = new PlatformSpecificationsService_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;

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


	iop::Config cfg("~PlatformSpecificationsService");
	// ACKERMANN
	cfg.param("specifics/ackermann/steering_angle_range", specifics_ackermann_steering_angle_range, specifics_ackermann_steering_angle_range, false, true, "radians");
	if (!std::isnan(specifics_ackermann_steering_angle_range))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setSteeringAngleRange(specifics_ackermann_steering_angle_range);
	cfg.param("specifics/ackermann/min_turn_radius", specifics_ackermann_min_turn_radius, specifics_ackermann_min_turn_radius, false, true, "[meter]");
	if (!std::isnan(specifics_ackermann_min_turn_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setMinTurnRadius(specifics_ackermann_min_turn_radius);

	cfg.param("specifics/ackermann/wheel_separation", specifics_ackermann_wheel_separation, specifics_ackermann_wheel_separation, false, true, "[meter]");
	if (!std::isnan(specifics_ackermann_wheel_separation))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setWheelSeparation(specifics_ackermann_wheel_separation);
	cfg.param("specifics/ackermann/drive_wheel_radius", specifics_ackermann_drive_wheel_radius, specifics_ackermann_drive_wheel_radius, false, true, "[meter]");
	if (!std::isnan(specifics_ackermann_drive_wheel_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getAckermann()->setDriveWheelRadius(specifics_ackermann_drive_wheel_radius);
	// SKIDSTEER
	cfg.param("specifics/skidsteer/track_separation", specifics_skidsteer_track_separation, specifics_skidsteer_track_separation, false, true, "[meter]");
	if (!std::isnan(specifics_skidsteer_track_separation))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getSkidsteer()->setTrackSeparation(specifics_skidsteer_track_separation);
	cfg.param("specifics/skidsteer/drive_wheel_radius", specifics_skidsteer_drive_wheel_radius, specifics_skidsteer_drive_wheel_radius, false, true, "[meter]");
	if (!std::isnan(specifics_skidsteer_drive_wheel_radius))
		report.getBody()->getPlatformData()->getPlatformSpecifics()->getSkidsteer()->setDriveWheelRadius(specifics_skidsteer_drive_wheel_radius);
	// INERTIAL
	cfg.param("inertial/maximum_forward_speed", inertial_maximum_forward_speed, inertial_maximum_forward_speed, false, true, "[meters per second]");
	if (!std::isnan(inertial_maximum_forward_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardSpeed(inertial_maximum_forward_speed);
	cfg.param("inertial/maximum_reverse_speed", inertial_maximum_reverse_speed, inertial_maximum_reverse_speed, false, true, "[meters per second]");
	if (!std::isnan(inertial_maximum_reverse_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseSpeed(inertial_maximum_reverse_speed);
	cfg.param("inertial/maximum_rotational_speed", inertial_maximum_rotational_speed, inertial_maximum_rotational_speed, false, true, "[radians per second]");
	if (!std::isnan(inertial_maximum_rotational_speed))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumRotationalSpeed(inertial_maximum_rotational_speed);
	cfg.param("inertial/maximum_forward_acceleration", inertial_maximum_forward_acceleration, inertial_maximum_forward_acceleration, false, true, "[meters per second squared]");
	if (!std::isnan(inertial_maximum_forward_acceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardAcceleration(inertial_maximum_forward_acceleration);
	cfg.param("inertial/maximum_reverse_acceleration", inertial_maximum_reverse_acceleration, inertial_maximum_reverse_acceleration, false, true, "[meters per second squared]");
	if (!std::isnan(inertial_maximum_reverse_acceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseAcceleration(inertial_maximum_reverse_acceleration);
	cfg.param("inertial/maximum_forward_deceleration", inertial_maximum_forward_deceleration, inertial_maximum_forward_deceleration, false, true, "[meters per second squared]");
	if (!std::isnan(inertial_maximum_forward_deceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumForwardDeceleration(inertial_maximum_forward_deceleration);
	cfg.param("inertial/maximum_reverse_deceleration", inertial_maximum_reverse_deceleration, inertial_maximum_reverse_deceleration, false, true, "[meters per second squared]");
	if (!std::isnan(inertial_maximum_reverse_deceleration))
		report.getBody()->getPlatformData()->getPlatformInertial()->setMaximumReverseDeceleration(inertial_maximum_reverse_deceleration);
	// SPEC
	cfg.param("spec/mobility_platform_name", spec_mobility_platform_name, spec_mobility_platform_name, false, true);
	report.getBody()->getPlatformData()->getPlatformSpec()->setMobilityPlatformName(spec_mobility_platform_name);
	cfg.param("spec/front", spec_front, spec_front, false, true, "[meter]");
	if (!std::isnan(spec_front))
		report.getBody()->getPlatformData()->getPlatformSpec()->setFront(spec_front);
	cfg.param("spec/back", spec_back, spec_back, false, true, "[meter]");
	if (!std::isnan(spec_back))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBack(spec_back);
	cfg.param("spec/right", spec_right, spec_right, false, true, "[meter]");
	if (!std::isnan(spec_right))
		report.getBody()->getPlatformData()->getPlatformSpec()->setRight(spec_right);
	cfg.param("spec/left", spec_left, spec_left, false, true, "[meter]");
	if (!std::isnan(spec_left))
		report.getBody()->getPlatformData()->getPlatformSpec()->setLeft(spec_left);
	cfg.param("spec/bottom", spec_bottom, spec_bottom, false, true, "[meter]");
	if (!std::isnan(spec_bottom))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBottom(spec_bottom);
	cfg.param("spec/top", spec_top, spec_top, false, true, "[meter]");
	if (!std::isnan(spec_top))
		report.getBody()->getPlatformData()->getPlatformSpec()->setTop(spec_top);
	cfg.param("spec/xcg", spec_xcg, spec_xcg, false, true, "[meter]");
	if (!std::isnan(spec_xcg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setXcg(spec_xcg);
	cfg.param("spec/ycg", spec_ycg, spec_ycg, false, true, "[meter]");
	if (!std::isnan(spec_ycg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setYcg(spec_ycg);
	cfg.param("spec/zcg", spec_zcg, spec_zcg, false, true, "[meter]");
	if (!std::isnan(spec_zcg))
		report.getBody()->getPlatformData()->getPlatformSpec()->setZcg(spec_zcg);
	cfg.param("spec/wheel_base", spec_wheel_base, spec_wheel_base, false, true, "[meter]");
	if (!std::isnan(spec_wheel_base))
		report.getBody()->getPlatformData()->getPlatformSpec()->setWheelBase(spec_wheel_base);
	cfg.param("spec/static_pitch_over", spec_static_pitch_over, spec_static_pitch_over, false, true, "[radians]");
	if (!std::isnan(spec_static_pitch_over))
		report.getBody()->getPlatformData()->getPlatformSpec()->setStaticPitchOver(spec_static_pitch_over);
	cfg.param("spec/static_roll_over", spec_static_roll_over, spec_static_roll_over, false, true, "[radians]");
	if (!std::isnan(spec_static_roll_over))
		report.getBody()->getPlatformData()->getPlatformSpec()->setStaticRollOver(spec_static_roll_over);
	cfg.param("spec/vehicle_weight", spec_vehicle_weight, spec_vehicle_weight, false, true, "[kilogram]");
	if (!std::isnan(spec_vehicle_weight))
		report.getBody()->getPlatformData()->getPlatformSpec()->setVehicleWeight(spec_vehicle_weight);
	cfg.param("spec/approach_angle", spec_approach_angle, spec_approach_angle, false, true, "[radians]");
	if (!std::isnan(spec_approach_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setApproachAngle(spec_approach_angle);
	cfg.param("spec/departure_angle", spec_departure_angle, spec_departure_angle, false, true, "[radians]");
	if (!std::isnan(spec_departure_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setDepartureAngle(spec_departure_angle);
	cfg.param("spec/break_over_angle", spec_break_over_angle, spec_break_over_angle, false, true, "[radians]");
	if (!std::isnan(spec_break_over_angle))
		report.getBody()->getPlatformData()->getPlatformSpec()->setBreakOverAngle(spec_break_over_angle);
}

void PlatformSpecificationsService_ReceiveFSM::sendReportPlatformSpecificationsAction(QueryPlatformSpecifications msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PlatformSpecificationsService",  "send ReportPlatformSpecifications to %s", sender.str().c_str());
	sendJausMessage(report, transportData.getAddress());
}





};
