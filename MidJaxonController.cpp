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
    "conf.default.debugLevel", "0",
    "conf.default.axesIds", "0,1,2",
    "conf.default.scales", "1.0,1.0,1.0",
    "conf.default.neutrals", "0.0,0.0,0.0",
    ""
  };
// </rtc-template>

MidJaxonController::MidJaxonController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qIn("q", m_q),
    m_axesIn("axes", m_axes),
    m_buttonsIn("buttons", m_buttons),
    m_qRefOut("qRef", m_qRef)
{
}

MidJaxonController::~MidJaxonController()
{
}


RTC::ReturnCode_t MidJaxonController::onInitialize()
{
  bindParameter("debugLevel", m_debugLevel, "0");
  addInPort("q", m_qIn);
  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);
  addOutPort("qRef", m_qRefOut);
  m_axes.data.length(7);
  for (unsigned int i=0; i<m_axes.data.length(); i++){
      m_axes.data[i] = 0.0;
  }
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

RTC::ReturnCode_t MidJaxonController::onExecute(RTC::UniqueId ec_id)
{
  if (m_axesIn.isNew()) m_axesIn.read();
  if (m_qIn.isNew()) {
    m_qIn.read(); 
    //m_angles.pan  = m_axes.data[0];
    //m_angles.tilt = m_axes.data[1];
    //if (m_debugLevel > 0) {
      std::cerr << m_axes.data[0] << " "
                << m_axes.data[1] << " "
                << m_axes.data[2] << " "
                << m_axes.data[3] << std::endl;
      //}
    m_qRefOut.write();
  }
  return RTC::RTC_OK;
}

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



