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
  m_buttons.data.length(10);
  for (unsigned int i=0; i<m_buttons.data.length(); i++){
      m_buttons.data[i] = false;
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

RTC::ReturnCode_t MidJaxonController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "MidJaxonController::onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MidJaxonController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "MidJaxonController::onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MidJaxonController::onExecute(RTC::UniqueId ec_id)
{
  if (m_axesIn.isNew()) m_axesIn.read();
  if (m_buttonsIn.isNew()) m_buttonsIn.read();
  if (m_qIn.isNew()) {
    m_qIn.read();
    m_qRef.data.length(m_q.data.length());
    for (size_t i = 0; i < m_q.data.length(); i++) {
      m_qRef.data[i] = m_q.data[i];
    }
    double lvel = m_axes.data[0] * -0.01;
    double rvel = m_axes.data[4] * -0.01;
    bool l_flip_up = m_buttons.data[4];
    bool l_flip_down = m_buttons.data[6];
    bool r_flip_up = m_buttons.data[5];
    bool r_flip_down = m_buttons.data[7];
    m_qRef.data[30] = m_qRef.data[31] = m_qRef.data[32] = lvel;
    m_qRef.data[33] = m_qRef.data[34] = m_qRef.data[35] = rvel;
    m_qRefOut.write();
    if (m_debugLevel > 0) {
      printf("lvel %4.1f rvel %4.1f\n", lvel, rvel);
    }
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



