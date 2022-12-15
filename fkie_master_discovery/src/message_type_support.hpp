#ifndef MESSAGE_TYPE_SUPPORT_HPP_
#define MESSAGE_TYPE_SUPPORT_HPP_

#include "fastrtps/Domain.h"
#include "fastrtps/participant/ParticipantListener.h"

#include "rmw_dds_common/msg/participant_entities_info.hpp"
#include <fastrtps/TopicDataType.h>
#include "fastrtps/subscriber/SampleInfo.h"

#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"

#include "rmw_fastrtps_cpp/MessageTypeSupport.hpp"
#include "rmw_fastrtps_cpp/ServiceTypeSupport.hpp"

//#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rmw/error_handling.h"

using TypeSupport_cpp = rmw_fastrtps_cpp::TypeSupport;
// copy from rmw_fastrtps_cpp/type_support_common.hpp
inline std::string
_create_type_name(
    const message_type_support_callbacks_t *members)
{
  if (!members)
  {
    RMW_SET_ERROR_MSG("members handle is null");
    return "";
  }

  std::ostringstream ss;
  std::string message_namespace(members->message_namespace_);
  std::string message_name(members->message_name_);
  if (!message_namespace.empty())
  {
    ss << message_namespace << "::";
  }
  ss << "dds_::" << message_name << "_";
  return ss.str();
}

std::string register_type(eprosima::fastrtps::Participant *participant, const void **type_support_impl);

#endif // MESSAGE_TYPE_SUPPORT_HPP_