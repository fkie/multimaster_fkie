// ROS 2 Node Manager
// Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
//
// Author: Alexander Tiderko
//
// Copyright 2020 Fraunhofer FKIE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "message_type_support.hpp"

#ifdef rmw_dds_common_FOUND
#define RMW_FASTRTPS_CPP_TYPESUPPORT_CPP rosidl_typesupport_fastrtps_cpp::typesupport_identifier

using ParticipantEntitiesInfo = rmw_dds_common::msg::ParticipantEntitiesInfo;

// The code of register_type() is based on rmw_fastrtps/rmw_fastrtps_cpp/subscription.cpp#create_subscription()
// We need it to register ParticipantEntitiesInfo to get messages from "ros_discovery_info"
std::string register_type(eprosima::fastrtps::Participant *participant, const void **type_support_impl)
{
  std::string empty;
  const rosidl_message_type_support_t *type_supports = rosidl_typesupport_cpp::get_message_type_support_handle<ParticipantEntitiesInfo>();
  const rosidl_message_type_support_t *type_support = get_message_typesupport_handle(type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
  if (!type_support)
  {
    RMW_SET_ERROR_MSG("type support not from this implementation");
    return empty;
  }
  *type_support_impl = type_support->data;

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  std::string type_name = _create_type_name(callbacks);
  rmw_fastrtps_cpp::MessageTypeSupport *message_type = nullptr;
  bool type_ok = eprosima::fastrtps::Domain::getRegisteredType(
      participant, type_name.c_str(),
      reinterpret_cast<eprosima::fastrtps::TopicDataType **>(&message_type));
  if (!type_ok)
  {
    message_type = new (std::nothrow) rmw_fastrtps_cpp::MessageTypeSupport(callbacks);
    if (!message_type)
    {
      RMW_SET_ERROR_MSG("Failed to allocate MessageTypeSupport");
      return empty;
    }
    eprosima::fastrtps::Domain::registerType(participant, message_type);
  }
  return type_name;
}

// TypeSupport implementation copied (1to1) from rmw_fastrtps_cpp/type_support_common.cpp
// because the linker cannot find the reference to rmw_fastrtps_cpp::MessageTypeSupport::MessageTypeSupport
namespace rmw_fastrtps_cpp
{

  TypeSupport::TypeSupport()
  {
    m_isGetKeyDefined = false;
    max_size_bound_ = false;
    // is_plain_ = false;
  }

  void TypeSupport::set_members(const message_type_support_callbacks_t *members)
  {
    members_ = members;
    bool is_plain_ = false;

#ifdef ROSIDL_TYPESUPPORT_FASTRTPS_HAS_PLAIN_TYPES
    char bounds_info;
    auto data_size = static_cast<uint32_t>(members->max_serialized_size(bounds_info));
    max_size_bound_ = 0 != (bounds_info & ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE);
    is_plain_ = bounds_info == ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE;
#else
    is_plain_ = true;
    auto data_size = static_cast<uint32_t>(members->max_serialized_size(is_plain_));
    max_size_bound_ = is_plain_;
#endif

    // A plain message of size 0 is an empty message
    if (is_plain_ && (data_size == 0))
    {
      has_data_ = false;
      ++data_size; // Dummy byte
    }
    else
    {
      has_data_ = true;
    }

    // Total size is encapsulation size + data size
    m_typeSize = 4 + data_size;
    // Account for RTPS submessage alignment
    m_typeSize = (m_typeSize + 3) & ~3;
  }

  size_t TypeSupport::getEstimatedSerializedSize(const void *ros_message, const void *impl) const
  {
    // if (is_plain_) {
    //   return m_typeSize;
    // }

    assert(ros_message);
    assert(impl);

    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);

    // Encapsulation size + message size
    return 4 + callbacks->get_serialized_size(ros_message);
  }

  bool TypeSupport::serializeROSmessage(
      const void *ros_message, eprosima::fastcdr::Cdr &ser, const void *impl) const
  {
    assert(ros_message);
    assert(impl);

    // Serialize encapsulation
    ser.serialize_encapsulation();

    // If type is not empty, serialize message
    if (has_data_)
    {
      auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
      return callbacks->cdr_serialize(ros_message, ser);
    }

    // Otherwise, add a dummy byte
    ser << (uint8_t)0;
    return true;
  }

  bool TypeSupport::deserializeROSmessage(
      eprosima::fastcdr::Cdr &deser, void *ros_message, const void *impl) const
  {
    assert(ros_message);
    assert(impl);

    try
    {
      // Deserialize encapsulation.
      deser.read_encapsulation();

      // If type is not empty, deserialize message
      if (has_data_)
      {
        auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
        return callbacks->cdr_deserialize(deser, ros_message);
      }

      // Otherwise, consume dummy byte
      uint8_t dump = 0;
      deser >> dump;
      (void)dump;
    }
    catch (const eprosima::fastcdr::exception::Exception &)
    {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Fast CDR exception deserializing message of type %s.",
          getName());
      return false;
    }

    return true;
  }

  MessageTypeSupport::MessageTypeSupport(const message_type_support_callbacks_t *members)
  {
    assert(members);

    std::string name = _create_type_name(members);
    this->setName(name.c_str());

    set_members(members);
  }

  ServiceTypeSupport::ServiceTypeSupport()
  {
  }

  RequestTypeSupport::RequestTypeSupport(const service_type_support_callbacks_t *members)
  {
    assert(members);

    auto msg = static_cast<const message_type_support_callbacks_t *>(
        members->request_members_->data);
    std::string name = _create_type_name(msg); // + "Request_";
    this->setName(name.c_str());

    set_members(msg);
  }

  ResponseTypeSupport::ResponseTypeSupport(const service_type_support_callbacks_t *members)
  {
    assert(members);

    auto msg = static_cast<const message_type_support_callbacks_t *>(
        members->response_members_->data);
    std::string name = _create_type_name(msg); // + "Response_";
    this->setName(name.c_str());

    set_members(msg);
  }

} // namespace rmw_fastrtps_cpp

#endif // rmw_dds_common_FOUND
