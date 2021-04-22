

#ifndef PLATFORMSPECIFICATIONSSERVICE_RECEIVEFSM_H
#define PLATFORMSPECIFICATIONSSERVICE_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_PlatformSpecifications/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_PlatformSpecifications/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "PlatformSpecificationsService_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>


namespace urn_jaus_jss_ugv_PlatformSpecifications
{

class DllExport PlatformSpecificationsService_ReceiveFSM : public JTS::StateMachine
{
public:
	PlatformSpecificationsService_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PlatformSpecificationsService_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();


	/// Action Methods
	virtual void sendReportPlatformSpecificationsAction(QueryPlatformSpecifications msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods


	PlatformSpecificationsService_ReceiveFSMContext *context;
	
protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	ReportPlatformSpecifications report;
      	double specifics_ackermann_steering_angle_range;  // [radians] The maximum achievable steering angles, as measured by a virtual wheel at the center of the front axle. Negative values cause the vehicle to turn left; positive values cause the vehicle to turn right.
        double specifics_ackermann_min_turn_radius;  // [meter] The radius of the smallest turn the vehicle is able to make.
        double specifics_ackermann_wheel_separation;  // [meter] Distance between the centers of the left and right wheels.
        double specifics_ackermann_drive_wheel_radius;  // [meter] Radius of the drive wheel (distance from center of the drive shaft to the outer edge of the drive wheel) in meters
        double specifics_skidsteer_track_separation;  // [meter] Distance between the two tracks.
        double specifics_skidsteer_drive_wheel_radius;  // [meter] Radius of the drive wheel (distance from center of the drive shaft to the outer edge of the drive wheel) in meters.
      	double inertial_maximum_forward_speed;  // [meters per second]
        double inertial_maximum_reverse_speed;  // [meters per second]
        double inertial_maximum_rotational_speed;  // [radians per second]
        double inertial_maximum_forward_acceleration;  // [meters per second squared]
        double inertial_maximum_reverse_acceleration;  // [meters per second squared]
        double inertial_maximum_forward_deceleration;  // [meters per second squared]
        double inertial_maximum_reverse_deceleration;  // [meters per second squared]
      	std::string spec_mobility_platform_name;  // A human-readable string for the vehicle.
        double spec_front;  // [meter]
        double spec_back;  // [meter]
        double spec_right;  // [meter]
        double spec_left;  // [meter]
        double spec_bottom;  // [meter]
        double spec_top;  // [meter]
        double spec_xcg;  // [meter] Measured from the vehicle coordinate frame.
        double spec_ycg;  // [meter] Measured from the vehicle coordinate frame.
        double spec_zcg;  // [meter] Measured from the vehicle coordinate frame.
        double spec_wheel_base;  // [meter] Distance between the center of the frontmost wheel and the center of the rearmost wheel.
        double spec_static_pitch_over;  // [radians]
        double spec_static_roll_over;  // [radians]
        double spec_vehicle_weight;  // [kilogram]
        double spec_approach_angle;  // [radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.
        double spec_departure_angle;  // [radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.
        double spec_break_over_angle;  // [radians] The angle formed by an imaginary line from the bottom of the tires to the center of the vehicle’s underside.

};

}

#endif // PLATFORMSPECIFICATIONSSERVICE_RECEIVEFSM_H
