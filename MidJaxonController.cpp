// -*- C++ -*-
/*!
 * @file  MidJaxonController.cpp * @brief  * $Date$ 
 *
 * $Id$ 
 */
#include "MidJaxonController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* midjaxoncontroller_spec[] =
  {
    "implementation_id", "MidJaxonController",
    "type_name",         "MidJaxonController",
    "description",       "",
    "version",           "1.0.0",
    "vendor",            "",
    "category",          "",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

MidJaxonController::MidJaxonController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_u_inIn("u_in", m_u_in),
    m_u_outOut("u_out", m_u_out)
{
}

MidJaxonController::~MidJaxonController()
{
}


RTC::ReturnCode_t MidJaxonController::onInitialize()
{
  addInPort("u_in", m_u_inIn);
  addOutPort("u_out", m_u_outOut);
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t MidJaxonController::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MidJaxonController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void MidJaxonControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(midjaxoncontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<MidJaxonController>,
                             RTC::Delete<MidJaxonController>);
  }
  
};



