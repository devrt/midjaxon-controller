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
    "conf.default.autobalance", "0",
    ""
  };
// </rtc-template>

MidJaxonController::MidJaxonController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qIn("q", m_q),
    m_qUpstreamIn("qUpstream", m_qUpstream),
    m_gsensorIn("gsensor", m_gsensor),
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
  bindParameter("autobalance", m_autobalance, "0");
  addInPort("q", m_qIn);
  addInPort("qUpstream", m_qUpstreamIn);
  addInPort("gsensor", m_gsensorIn);
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
  prev_flips.resize(4);
  for (unsigned int i=0; i<prev_flips.size(); i++){
    prev_flips[i] = 0.0;
  }
  offset_flips.resize(4);
  for (unsigned int i=0; i<offset_flips.size(); i++){
    offset_flips[i] = 0.0;
  }
  offset_yaw = 0.0;
  offset_pitch = 0.0;
  g_x = g_y = 0.0;
  state = COMPLETED;
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
  if (m_gsensorIn.isNew()) m_gsensorIn.read();
  if (m_qUpstreamIn.isNew()) m_qUpstreamIn.read();
  if (m_qIn.isNew()) {
    m_qIn.read();
    m_qRef.data.length(m_q.data.length());
    if (m_q.data.length() != m_qUpstream.data.length()) return RTC::RTC_OK;
    for (size_t i = 0; i < m_qUpstream.data.length(); i++) {
      m_qRef.data[i] = m_qUpstream.data[i];
    }
    if (m_axes.data.length() < 6) {
      m_qRefOut.write();
      return RTC::RTC_OK;
    }

    // control speed of crawlers
    double lvel = m_axes.data[1] * -0.02;
    double rvel = m_axes.data[4] * -0.02;
    m_qRef.data[30] = m_qRef.data[31] = m_qRef.data[32] = lvel;
    m_qRef.data[33] = m_qRef.data[34] = m_qRef.data[35] = rvel;

    // control angles of flippers
    bool l_flip_up = m_buttons.data[4];
    bool l_flip_down = m_buttons.data[6];
    bool r_flip_up = m_buttons.data[5];
    bool r_flip_down = m_buttons.data[7];
    bool flip_changed = false;
    for (unsigned int i=0; i < prev_flips.size(); i++) {
      if (m_qUpstream.data[26+i] != prev_flips[i]) flip_changed = true;
    }
    if (flip_changed) {
      for (unsigned int i=0; i < offset_flips.size(); i++) {
        offset_flips[i] = 0.0;
      }
    }
    double flip_step = 0.001;
    if (l_flip_up) {
      offset_flips[0] = offset_flips[2] = offset_flips[0] - flip_step;
    } else if (l_flip_down) {
      offset_flips[0] = offset_flips[2] = offset_flips[0] + flip_step;
    }
    if (r_flip_up) {
      offset_flips[1] = offset_flips[3] = offset_flips[1] + flip_step;
    } else if (r_flip_down) {
      offset_flips[1] = offset_flips[3] = offset_flips[1] - flip_step;
    }
    for (unsigned int i=0; i < prev_flips.size(); i++) {
      m_qRef.data[26+i] = m_qUpstream.data[26+i] + offset_flips[i];
    }
    for (unsigned int i=0; i < prev_flips.size(); i++) {
      prev_flips[i] = m_qUpstream.data[26+i];
    }

    // yaw of body
    double yaw = m_axes.data[5];
    double pitch = m_axes.data[6];
    double yaw_step = 0.001;
    double pitch_step = 0.001;
    offset_yaw -= yaw_step * yaw;
    offset_pitch += pitch_step * pitch;
    if (flip_changed) {
      offset_yaw = 0.0;
      offset_pitch = 0.0;
    }
    m_qRef.data[2] = m_qUpstream.data[2] + offset_yaw;
    m_qRef.data[4] =  m_qUpstream.data[4] + offset_pitch;
    
    // auto balancer
    g_x = g_x * 0.999 + m_gsensor.data.ax * 0.001;
    g_y = g_y * 0.999 + m_gsensor.data.ay * 0.001;
    if (m_autobalance) {
      if (flip_changed) {
        m_qRef.data[1] = m_qUpstream.data[0];
        m_qRef.data[0] = m_qUpstream.data[1];
      } else {
        m_qRef.data[1] = g_x * 0.1;
        m_qRef.data[0] = -g_y * 0.1;
      }
    }
    
    m_qRefOut.write();
    if (m_debugLevel > 0) {
      //printf("lvel %4.2f, rvel %4.2f, lflip %4.2f, rflip %4.2f\n",
      //       lvel, rvel, offset_flips[0], offset_flips[1]);
      printf("gsensor %4.2f, %4.2f\n",
             g_x, g_y);
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



