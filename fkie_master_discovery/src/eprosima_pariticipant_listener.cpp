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
//
// This code is based on rmw_fastrtps/rmw_fastrtps_shared_cpp/include/rmw_fastrtps_shared_cpp/custom_participant_info.hpp

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <limits.h>

#include "rmw_fastrtps_shared_cpp/guid_utils.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"

#include "fastrtps/Domain.h"
#include "fastrtps/participant/ParticipantListener.h"
#include "fastrtps/subscriber/SubscriberListener.h"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/thread_safety_annotations.hpp"
#include "rmw/impl/cpp/key_value.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "fkie_multimaster_msgs/msg/discovered_state.hpp"
#include "fkie_multimaster_msgs/msg/gid.hpp"
#include "fkie_multimaster_msgs/msg/participant_entities_info.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

using namespace std::chrono_literals;
using namespace fkie_multimaster_msgs::msg;

using paricipant_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, ParticipantEntitiesInfo>;

/**
 * Get a value from a environment variable
 */
std::string getEnvironmentVariable(std::string const &key)
{
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
}


void convert_gid_to_msg(const eprosima::fastrtps::rtps::GUID_t &gid, fkie_multimaster_msgs::msg::Gid &msg_gid)
{
    rmw_fastrtps_shared_cpp::copy_from_fastrtps_guid_to_byte_array(gid, const_cast<uint8_t *>(msg_gid.data.begin()));
}


void convert_msg_to_gid(const fkie_multimaster_msgs::msg::Gid &msg_gid, eprosima::fastrtps::rtps::GUID_t &gid)
{
    rmw_fastrtps_shared_cpp::copy_from_byte_array_to_fastrtps_guid(const_cast<uint8_t *>(msg_gid.data.begin()), &gid);
}


class CustomParticipantListener : public rclcpp::Node,
                                  public eprosima::fastrtps::ParticipantListener,
                                  public eprosima::fastrtps::SubscriberListener
{
public:
    CustomParticipantListener(const std::string &node_name, const std::string &namespace_) : Node(node_name, namespace_),
                                                                                             eprosima::fastrtps::ParticipantListener(),
                                                                                             eprosima::fastrtps::SubscriberListener()
    {
        rclcpp::QoS qos_settings(100);
        // qos_settings.reliable();
        // qos_settings.transient_local();
        qos_settings.keep_last(1);
        on_shutdown = false;
        publisher_ = this->create_publisher<fkie_multimaster_msgs::msg::DiscoveredState>("~/rosstate", qos_settings);
        daemon_topic_ = std::string("rt") + this->publisher_->get_topic_name();
        participant_ = nullptr;
        eprosima::fastrtps::ParticipantAttributes participant_attr;
        std::string full_name = this->get_node_base_interface()->get_fully_qualified_name();
        RCLCPP_INFO(get_logger(), "Node name: %s", node_name.c_str());
        RCLCPP_INFO(get_logger(), "create eProsima participant: %s", full_name.c_str());
        participant_attr.rtps.setName(full_name.erase(0, 1).c_str());
        participant_ = eprosima::fastrtps::Domain::createParticipant(participant_attr, this);
    }

    void shutdown()
    {
        this->on_shutdown = true;
        eprosima::fastrtps::Domain::stopAll();
    }

    // SubscriberListener implementation
    void onSubscriptionMatched(
        eprosima::fastrtps::Subscriber * /*sub*/,
        eprosima::fastrtps::rtps::MatchingInfo &info) final
    {
        // std::lock_guard<std::mutex> lock(internalMutex_);
        if (eprosima::fastrtps::rtps::MATCHED_MATCHING == info.status)
        {
            // std::cout << "mathed subscription " << info.remoteEndpointGuid << std::endl;
            // publishers_.insert(info.remoteEndpointGuid);
        }
        else if (eprosima::fastrtps::rtps::REMOVED_MATCHING == info.status)
        {
            // publishers_.erase(info.remoteEndpointGuid);
            // std::cout << "removed subscription " << info.remoteEndpointGuid << std::endl;
        }
    }

    void onSubscriberDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&/*info*/) override
    {
        (void)participant;

        // if (eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        //     {
        //         isnew = true;
        //     }
        //     else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        //     {
        //     }
        //     process_discovery_info(info.info, isnew, fkie_multimaster_msgs::msg::TopicEntity::INFO_READER);
        // }
        publish_notification();

    }

    void onPublisherDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo &&/*info*/) override
    {
        (void)participant;

        // if (eprosima::fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        //     {
        //         isnew = true;
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher discovered");
        //     }
        //     else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        //     {
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher removed");
        //     }
        //     process_discovery_info(info.info, isnew, fkie_multimaster_msgs::msg::TopicEntity::INFO_WRITER);
        // }
        publish_notification();

    }

    template <class T>
    std::string to_string(T &data)
    {
        std::stringstream stream;
        stream << data;
        return stream.str();
    }

    void publish_notification()
    {
        std::lock_guard<std::mutex> guard(mutex_);
        fkie_multimaster_msgs::msg::DiscoveredState rosmsg;
        for (auto itdp = discoveredParticipants_.begin(); itdp != discoveredParticipants_.end(); itdp++)
        {
            rosmsg.participants.push_back(itdp->second);
        }
        RCLCPP_DEBUG(get_logger(), "rosgraph: publish state with %lu participants", rosmsg.participants.size());
        publisher_->publish(rosmsg);
    }

    void onParticipantDiscovery(
        eprosima::fastrtps::Participant * /*participant*/,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info)
    {
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery: participant info %s", info.info.m_participantName.c_str());
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:   status %d", info.status);
        auto participant_guid = to_string(info.info.m_guid);
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT:
        {
            RCLCPP_INFO(get_logger(), "onParticipantDiscovery: new participant %s", participant_guid.c_str());
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            if (itp == discoveredParticipants_.end())
            {
                ParticipantEntitiesInfo pei;
                fkie_multimaster_msgs::msg::Gid gid;
                convert_gid_to_msg(info.info.m_guid, gid);
                pei.guid = gid;
                discoveredParticipants_[info.info.m_guid] = pei;
            }
            auto pi = discoveredParticipants_[info.info.m_guid];
            auto map = rmw::impl::cpp::parse_key_value(info.info.m_userData);
            // get as defined since foxy
            auto name_found = map.find("name");
            if (name_found != map.end())
            {
                pi.enclave = std::string(name_found->second.begin(), name_found->second.end());
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   name '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            }            // get as defined since foxy
            auto enclave_found = map.find("enclave");
            if (enclave_found != map.end())
            {
                pi.enclave = std::string(enclave_found->second.begin(), enclave_found->second.end());
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   enclave '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            }
            for (auto i = info.info.default_locators.unicast.begin(); i != info.info.default_locators.unicast.end(); ++i)
            {
                RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:  add unicast locator: %s", to_string(*i).c_str());
                pi.unicast_locators.push_back(to_string(*i));
            }
            discoveredParticipants_[info.info.m_guid] = pi;
            break;
        }
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT:
        // fall through
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT:
        {
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            // only consider known GUIDs
            if (itp != discoveredParticipants_.end())
            {
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   remove participant %s:", to_string(info.info.m_guid).c_str());
                discoveredParticipants_.erase(itp);
            }
            break;
        }
        default:
            return;
        }
        publish_notification();
    }

    /** Returns current time in seconds. */
    uint64_t now()
    {
        auto time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    }

private:
    mutable std::mutex mutex_;
    bool on_shutdown;
    eprosima::fastrtps::Participant *participant_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    rclcpp::Publisher<fkie_multimaster_msgs::msg::DiscoveredState>::SharedPtr publisher_;
    std::string daemon_topic_;
    double stamp_;
    paricipant_map_t discoveredParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    char hostname_chars[HOST_NAME_MAX];
    gethostname(hostname_chars, HOST_NAME_MAX);
    // std::string ros_distro = getEnvironmentVariable("ROS_DISTRO");
    std::string node_name = "_discovery_" + std::string(hostname_chars);
    auto listener = std::make_shared<CustomParticipantListener>(node_name, "/node_manager");
    RCLCPP_INFO(listener->get_logger(), "started");
    rclcpp::spin(listener);
    RCLCPP_INFO(listener->get_logger(), "shutdown...");
    listener->shutdown();
    rclcpp::shutdown();
    printf("bye!\n");
    return 0;
}
