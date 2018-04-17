

#include "urn_jaus_jss_ugv_IlluminationService/IlluminationService_ReceiveFSM.h"
#include <iop_component_fkie/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_ugv_IlluminationService
{



IlluminationService_ReceiveFSM::IlluminationService_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new IlluminationService_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
}



IlluminationService_ReceiveFSM::~IlluminationService_ReceiveFSM()
{
	delete context;
}

void IlluminationService_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_IlluminationService_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_IlluminationService_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_IlluminationService_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_IlluminationService_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "IlluminationService_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "IlluminationService_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "IlluminationService_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "IlluminationService_ReceiveFSM");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryIlluminationState::ID);
	// iop::Config cfg("~Illumination");
	p_illuminator_list.init();
	p_illuminator_list.set_state_callback(&IlluminationService_ReceiveFSM::p_state_callback, this);
	p_ilumination_state = p_illuminator_list.get_state_report();
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryIlluminationState::ID, &p_ilumination_state);
}

void IlluminationService_ReceiveFSM::sendReportIlluminationConfigurationAction(QueryIlluminationConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	ReportIlluminationConfiguration ilumination_cfg = p_illuminator_list.get_configuration_report();
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("Illumination", "send ReportIlluminationConfiguration to %s", sender.str().c_str());
	sendJausMessage(ilumination_cfg, transportData.getAddress());
}

void IlluminationService_ReceiveFSM::sendReportIlluminationStateAction(QueryIlluminationState msg, Receive::Body::ReceiveRec transportData)
{
	ReportIlluminationState ilumination_state = p_illuminator_list.get_state_report();
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("Illumination", "send ReportIlluminationState to %s", sender.str().c_str());
	sendJausMessage(ilumination_state, transportData.getAddress());
}

void IlluminationService_ReceiveFSM::setIlluminationStateAction(SetIlluminationState msg)
{
	ROS_DEBUG_NAMED("Illumination", "SetIlluminationState");
	p_illuminator_list.set_state(*msg.getBody()->getIlluminationRec()->getIllumination());
}



bool IlluminationService_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool IlluminationService_ReceiveFSM::isSupported(SetIlluminationState msg)
{
	return true;  // TODO: should we check only for TRUE? It is done while setting.
}

void IlluminationService_ReceiveFSM::p_state_callback(urn_jaus_jss_ugv_IlluminationService::ReportIlluminationState report)
{
	p_ilumination_state = report;
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryIlluminationState::ID, &p_ilumination_state);
}

};
