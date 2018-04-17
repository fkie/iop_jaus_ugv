

#ifndef ILLUMINATIONSERVICE_RECEIVEFSM_H
#define ILLUMINATIONSERVICE_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_IlluminationService/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_IlluminationService/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"

#include <ros/ros.h>
#include <iop_illumination_fkie/IlluminatorList.h>
#include "IlluminationService_ReceiveFSM_sm.h"

namespace urn_jaus_jss_ugv_IlluminationService
{

class DllExport IlluminationService_ReceiveFSM : public JTS::StateMachine
{
public:
	IlluminationService_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~IlluminationService_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendReportIlluminationConfigurationAction(QueryIlluminationConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportIlluminationStateAction(QueryIlluminationState msg, Receive::Body::ReceiveRec transportData);
	virtual void setIlluminationStateAction(SetIlluminationState msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isSupported(SetIlluminationState msg);



	IlluminationService_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	ReportIlluminationState p_ilumination_state;
	iop::IlluminatorList p_illuminator_list;

	void p_state_callback(urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState report);
};

};

#endif // ILLUMINATIONSERVICE_RECEIVEFSM_H
