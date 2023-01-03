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

#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#include "rmw_dds_common/msg/detail/participant_entities_info__struct.hpp"
#include "rmw_fastrtps_shared_cpp/guid_utils.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"
#include "message_type_support.hpp"

#include "fastrtps/Domain.h"
#include "fastrtps/participant/ParticipantListener.h"
#include "fastrtps/subscriber/Subscriber.h"
#include "fastrtps/subscriber/SubscriberListener.h"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/thread_safety_annotations.hpp"
#include "rmw/impl/cpp/key_value.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "fkie_multimaster_msgs/msg/discovered_state.hpp"
#include "fkie_multimaster_msgs/msg/endpoint.hpp"
#include "fkie_multimaster_msgs/msg/gid.hpp"
#include "fkie_multimaster_msgs/msg/node_entities_info.hpp"
#include "fkie_multimaster_msgs/msg/participant_entities_info.hpp"
#include "fkie_multimaster_msgs/msg/topic_entity.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

using namespace std::chrono_literals;
using namespace fkie_multimaster_msgs::msg;

using paricipant_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, ParticipantEntitiesInfo>;
using topic_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, TopicEntity>;

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
        on_shutdown = false;
        publisher_ = this->create_publisher<fkie_multimaster_msgs::msg::DiscoveredState>("~/rosstate", qos_settings);
        subscriber_daemon_ = this->create_subscription<fkie_multimaster_msgs::msg::Endpoint>("daemons", qos_settings, std::bind(&CustomParticipantListener::_daemon_detected, this, std::placeholders::_1));
        daemon_topic_ = std::string("rt") + this->publisher_->get_topic_name();
        participant_ = nullptr;
        subscriber_ = nullptr;
        infoUpdated_ = false;
        infoUpdatedTs_ = 0;
        infoUpdatedSkipped_ = 0;
        eprosima::fastrtps::ParticipantAttributes participant_attr;
        std::string full_name = this->get_node_base_interface()->get_fully_qualified_name();
        RCLCPP_INFO(get_logger(), "Node Name: %s", node_name.c_str());
        RCLCPP_INFO(get_logger(), "create eProsima participant: %s", full_name.c_str());
        participant_attr.rtps.setName(full_name.erase(0, 1).c_str());
        participant_ = eprosima::fastrtps::Domain::createParticipant(participant_attr, this);
        eprosima::fastrtps::SubscriberAttributes subscriber_attr; // Configuration structure
        subscriber_attr.topic.historyQos.kind = eprosima::fastrtps::KEEP_ALL_HISTORY_QOS;
        subscriber_attr.qos.m_reliability.kind = eprosima::fastrtps::RELIABLE_RELIABILITY_QOS;
        subscriber_attr.qos.m_durability.kind = eprosima::fastrtps::TRANSIENT_LOCAL_DURABILITY_QOS;
        subscriber_attr.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        subscriber_attr.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        subscriber_attr.topic.topicDataType = register_type(participant_, &message_type_impl_);
        // subscriber_attr.topic.topicDataType = "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";
        subscriber_attr.topic.topicName = "ros_discovery_info";
        RCLCPP_INFO(get_logger(), "create subscription on 'ros_discovery_info' as %s", subscriber_attr.topic.topicDataType.c_str());
        subscriber_ = eprosima::fastrtps::Domain::createSubscriber(participant_, subscriber_attr, this);
        RCLCPP_INFO(get_logger(), "done");
        timer_ = this->create_wall_timer(1000ms, std::bind(&CustomParticipantListener::_callback_timer_state, this));
    }

    void shutdown()
    {
        this->on_shutdown = true;
        this->timer_->cancel();
        eprosima::fastrtps::Domain::stopAll();
    }

    void _callback_timer_state()
    {
        bool doUpdate = false;
        if (infoUpdated_)
        {
            doUpdate = true;
            if (now() - infoUpdatedTs_ < 500000000)
            {
                doUpdate = false;
                infoUpdatedSkipped_++;
            }
            if (infoUpdatedSkipped_ > 5)
            {
                doUpdate = true;
            }
        }
        if (doUpdate)
        {
            std::lock_guard<std::mutex> guard(mutex_);
            fkie_multimaster_msgs::msg::DiscoveredState rosmsg;
            rosmsg.full_state = fullUpdateRequest_;
            if (fullUpdateRequest_)
            {
                RCLCPP_INFO(get_logger(), "rosgraph: full update");
                for (auto itdp = discoveredParticipants_.begin(); itdp != discoveredParticipants_.end(); itdp++)
                {
                    RCLCPP_INFO(get_logger(), "rosgraph: participant; guid=%s, enclave=%s with %lu nodes and %lu topics", to_string(itdp->first).c_str(), itdp->second.enclave.c_str(), itdp->second.node_entities.size(), itdp->second.topic_entities.size());
                    rosmsg.participants.push_back(itdp->second);
                }
                fullUpdateRequest_ = false;
            } else
            {
                RCLCPP_INFO(get_logger(), "rosgraph: update");
                for (auto &removed_participant : removedParticipants_)
                {
                    RCLCPP_INFO(get_logger(), "rosgraph: participant removed; guid=%s", to_string(removed_participant).c_str());
                    fkie_multimaster_msgs::msg::Gid gid;
                    convert_gid_to_msg(removed_participant, gid);
                    rosmsg.removed_participants.push_back(gid);
                }

                for (auto &updated_participant : updatedParticipants_)
                {
                    auto itdp = discoveredParticipants_.find(updated_participant);
                    if (itdp != discoveredParticipants_.end()) {
                        RCLCPP_INFO(get_logger(), "rosgraph: participant; guid=%s, enclave=%s with %lu nodes and %lu topics", to_string(itdp->first).c_str(), itdp->second.enclave.c_str(), itdp->second.node_entities.size(), itdp->second.topic_entities.size());
                        rosmsg.participants.push_back(itdp->second);
                    } else {
                        RCLCPP_WARN(get_logger(), "rosgraph: participant; guid=%s not found while update", to_string(updated_participant).c_str());
                    }
                }
            }
            removedParticipants_.clear();
            updatedParticipants_.clear();
            RCLCPP_INFO(get_logger(), "rosgraph:     publish");
            publisher_->publish(rosmsg);
            RCLCPP_INFO(get_logger(), "rosgraph:     published");
            infoUpdated_ = false;
            infoUpdatedSkipped_ = 0;
        }
    }

    void _daemon_detected(const fkie_multimaster_msgs::msg::Endpoint::SharedPtr msg)
    {
        if (!msg->on_shutdown)
        {
            infoUpdated_ = true;
            infoUpdatedTs_ = now();
            fullUpdateRequest_ = true;
        }
    }

    builtin_interfaces::msg::Duration from_rmw_time(rmw_time_t duration)
    {
        builtin_interfaces::msg::Duration res;
        res.sec = duration.sec;
        res.nanosec = duration.nsec;
        return res;
    }

    /** called on 'ros_discovery_info'
     * We update ROS nodes associated with a participant.
     */
    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub)
    {
        if (this->on_shutdown)
            return;
        RCLCPP_INFO(get_logger(), "New ParticipantEntitiesInfo msg on 'ros_discovery_info', count unread: %lu", sub->get_unread_count());
        std::lock_guard<std::mutex> guard(mutex_);
        rmw_dds_common::msg::ParticipantEntitiesInfo msg;
        eprosima::fastcdr::FastBuffer buffer;
        rmw_fastrtps_shared_cpp::SerializedData data;
        data.is_cdr_buffer = false;
        data.data = &msg;
        data.impl = message_type_impl_; // not used when is_cdr_buffer is true
        eprosima::fastrtps::SampleInfo_t info;
        if (sub->readNextData(&data, &info))
        {
            rmw_gid_t rmw_gid;
            rmw_dds_common::convert_msg_to_gid(&msg.gid, &rmw_gid);
            eprosima::fastrtps::rtps::GUID_t guid;
            rmw_fastrtps_shared_cpp::copy_from_byte_array_to_fastrtps_guid(rmw_gid.data, &guid);
            if (info.sampleKind != eprosima::fastrtps::rtps::ALIVE)
            {
                RCLCPP_INFO(get_logger(), "new participant discovered is not alive, ignore; guid=%s", to_string(guid).c_str());
                return;
            }
            auto itp = discoveredParticipants_.find(guid);
            if (itp == discoveredParticipants_.end())
            {
                ParticipantEntitiesInfo pei;
                RCLCPP_INFO(get_logger(), "new participant discovered; guid=%s", to_string(guid).c_str());
                fkie_multimaster_msgs::msg::Gid gid;
                convert_gid_to_msg(guid, gid);
                pei.guid = gid;
                discoveredParticipants_[guid] = pei;
            }
            auto &participant = discoveredParticipants_[guid];
            updatedParticipants_.insert(guid);
            std::vector<NodeEntitiesInfo> old_nodes;
            for (auto it = participant.node_entities.begin(); it != participant.node_entities.end(); it++)
            {
                old_nodes.push_back(*it);
            }
            // create a new list with all nodes
            participant.node_entities.clear();
            participant.topic_entities.clear();
                RCLCPP_INFO(get_logger(), "  add discovered nodes for guid=%s", to_string(guid).c_str());
            for (auto &node : msg.node_entities_info_seq)
            {
                NodeEntitiesInfo &ni = _get_node(participant, node.node_name, node.node_namespace);
                // update publisher/subscriber
                ni.publisher.clear();
                for (auto &writer_gid : node.writer_gid_seq)
                {
                    fkie_multimaster_msgs::msg::Gid gid;
                    std::memcpy(&gid.data, &writer_gid, sizeof(gid.data));
                    ni.publisher.push_back(gid);
                    eprosima::fastrtps::rtps::GUID_t writer_guid;
                    convert_msg_to_gid(gid, writer_guid);
                    auto itt = discoveredTopics_.find(writer_guid);
                    if (itt != discoveredTopics_.end())
                    {
                        participant.topic_entities.push_back(itt->second);
                    } else
                    {
                        RCLCPP_WARN(get_logger(), "  no topic for guid=%s found! assigned as publisher to node=%s, namespace=%s", to_string(writer_guid).c_str(), node.node_name.c_str(), node.node_namespace.c_str());
                    }
                    // participant.topic_entities.push_back(discoveredTopics_[writer_guid]);
                }
                ni.subscriber.clear();
                for (auto &reader_gid : node.reader_gid_seq)
                {
                    fkie_multimaster_msgs::msg::Gid gid;
                    std::memcpy(&gid.data, &reader_gid, sizeof(gid.data));
                    ni.subscriber.push_back(gid);
                    eprosima::fastrtps::rtps::GUID_t reader_guid;
                    convert_msg_to_gid(gid, reader_guid);
                    auto itt = discoveredTopics_.find(reader_guid);
                    if (itt != discoveredTopics_.end())
                    {
                        participant.topic_entities.push_back(itt->second);
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "  no topic for guid=%s found! assigned as subscriber to node=%s, namespace=%s", to_string(reader_guid).c_str(), node.node_name.c_str(), node.node_namespace.c_str());
                    }
                }
                infoUpdated_ = true;
                infoUpdatedTs_ = now();
                // remove the node from old_nodes
                for (auto it = old_nodes.begin(); it != old_nodes.end(); it++)
                {
                    if (it->ns.compare(node.node_namespace) == 0 && it->name.compare(node.node_name) == 0)
                    {
                        old_nodes.erase(it);
                        break;
                    }
                }
            }
            // copy all remaining nodes to removed nodes
            // if (old_nodes.size() > 0)
            // {
            //     removedParticipants_.insert(guid);
            // }
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Can't read ParticipantEntitiesInfo from 'ros_discovery_info'");
        }
    }

    NodeEntitiesInfo &_get_node(ParticipantEntitiesInfo &pi, std::string name, std::string ns)
    {
        for (auto &pi_node : pi.node_entities)
        {
            if (name.compare(pi_node.name) == 0 && ns.compare(pi_node.ns) == 0)
            {
                return pi_node;
            }
        }
        RCLCPP_INFO(get_logger(), "    new node discovered; ns=%s, name=%s, count already known nodes: %lu", ns.c_str(), name.c_str(), pi.node_entities.size());
        NodeEntitiesInfo ni;
        ni.name = name;
        ni.ns = ns;
        pi.node_entities.push_back(ni);
        return pi.node_entities.back();
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
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) override
    {
        (void)participant;

        if (eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER != info.status)
        {
            bool isnew = false;
            if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
            {
                isnew = true;
            }
            else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
            {
            }
            process_discovery_info(info.info, isnew, fkie_multimaster_msgs::msg::TopicEntity::INFO_READER);
        }
    }

    void onPublisherDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) override
    {
        (void)participant;

        if (eprosima::fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER != info.status)
        {
            bool isnew = false;
            if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
            {
                isnew = true;
            }
            else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
            {
            }
            process_discovery_info(info.info, isnew, fkie_multimaster_msgs::msg::TopicEntity::INFO_WRITER);
        }
    }

    template <class T>
    std::string to_string(T &data)
    {
        std::stringstream stream;
        stream << data;
        return stream.str();
    }

    template <class T>
    void process_discovery_info(T &proxyData, bool isnew, int8_t topicInfo)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        auto participant_guid = iHandle2GUID(proxyData.RTPSParticipantKey());
        if (isnew)
        {
            fkie_multimaster_msgs::msg::TopicEntity ti;
            auto itt = discoveredTopics_.find(proxyData.guid());
            if (itt != discoveredTopics_.end())
            {
                ti = itt->second;
            }
            ti.name = proxyData.topicName();
            ti.ttype = proxyData.typeName();
            ti.info = topicInfo;
            convert_gid_to_msg(proxyData.guid(), ti.guid);
            rmw_qos_profile_t qos = rmw_qos_profile_unknown;
            rtps_qos_to_rmw_qos(proxyData.m_qos, &qos);
            ti.qos.durability = qos.durability;
            ti.qos.history = qos.history;
            ti.qos.depth = qos.depth;
            ti.qos.liveliness = qos.liveliness;
            ti.qos.reliability = qos.reliability;
            ti.qos.deadline = from_rmw_time(qos.deadline);
            ti.qos.lease_duration = from_rmw_time(qos.liveliness_lease_duration);
            ti.qos.lifespan = from_rmw_time(qos.lifespan);
            discoveredTopics_[proxyData.guid()] = ti;
            RCLCPP_INFO(get_logger(), "process_discovery_info: new topic %s, participan_guid: %s", ti.name.c_str(), to_string(participant_guid).c_str());
            if (ti.name == daemon_topic_)
            {
                RCLCPP_INFO(get_logger(), "process_discovery_info: ->>DAEMON found %s, participan_guid: %s", ti.name.c_str(), to_string(participant_guid).c_str());
                infoUpdated_ = true;
                infoUpdatedTs_ = now();
                fullUpdateRequest_ = true;
            }

            // If the 'ros_discovery_info' message was received before the topic was discovered, we need to add the topic information afterwards.
            for (auto itdp = discoveredParticipants_.begin(); itdp != discoveredParticipants_.end(); itdp++)
            {
                bool found = false;
                // find the participant with the topic id (in node's publisher or subscriber lists)
                for (auto it_ne = itdp->second.node_entities.begin(); it_ne != itdp->second.node_entities.end(); it_ne++)
                {
                    for (auto it_pub = it_ne->publisher.begin(); it_pub != it_ne->publisher.end(); it_pub++)
                    {
                        eprosima::fastrtps::rtps::GUID_t gid;
                        convert_msg_to_gid(*it_pub, gid);
                        if (proxyData.guid() == gid)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        for (auto it_sub = it_ne->subscriber.begin(); it_sub != it_ne->subscriber.end(); it_sub++)
                        {
                            eprosima::fastrtps::rtps::GUID_t gid;
                            convert_msg_to_gid(*it_sub, gid);
                            if (proxyData.guid() == gid)
                            {
                                found = true;
                                break;
                            }
                        }
                    }
                }
                if (found) {
                    // we do not check for already added topic info, since we discoverd it now
                    RCLCPP_INFO(get_logger(), "process_discovery_info:   add as topic_entity to existing participant");
                    itdp->second.topic_entities.push_back(ti);
                    infoUpdated_ = true;
                    infoUpdatedTs_ = now();
                }
            }
        } else
        {
            auto itp = discoveredTopics_.find(proxyData.guid());
            if (itp != discoveredTopics_.end())
            {
                discoveredTopics_.erase(itp);
            }
            RCLCPP_INFO(get_logger(), "remove topic %s", proxyData.topicName().c_str());
        }
    }

    void onParticipantDiscovery(
        eprosima::fastrtps::Participant * /*participant*/,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info)
    {
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery: participant info %s", info.info.m_participantName.c_str());
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:   status %d", info.status);
        std::lock_guard<std::mutex> guard(mutex_);
        auto participant_guid = to_string(info.info.m_guid);
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT:
        {
            RCLCPP_INFO(get_logger(), "onParticipantDiscovery: new participant %s", participant_guid.c_str());
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
            auto enclave_found = map.find("enclave");
            if (enclave_found != map.end())
            {
                pi.enclave = std::string(enclave_found->second.begin(), enclave_found->second.end());
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   enclave '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            }
            else
            {
                // compatibility to eloquent version
                NodeEntitiesInfo ni;
                auto name_found = map.find("name");
                auto ns_found = map.find("namespace");
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:  no enclave found for %s, pname: %s", participant_guid.c_str(), info.info.m_participantName.c_str());
                if (name_found != map.end())
                {
                    ni.name = std::string(name_found->second.begin(), name_found->second.end());
                    if (ns_found != map.end())
                    {
                        ni.ns = std::string(ns_found->second.begin(), ns_found->second.end());
                    }
                    pi.node_entities.push_back(ni);
                }
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
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            // only consider known GUIDs
            if (itp != discoveredParticipants_.end())
            {
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   remove participant %s with nodes[%lu]:", to_string(info.info.m_guid).c_str(), itp->second.node_entities.size());
                for (auto &ni : itp->second.node_entities)
                {
                    RCLCPP_INFO(get_logger(), "onParticipantDiscovery:     - ns=%s, name:=%s", ni.ns.c_str(), ni.name.c_str());
                }
                discoveredParticipants_.erase(itp);
                // remove all topics of this participant ID
                for (auto itt = discoveredTopics_.begin(); itt != discoveredTopics_.end();)
                {
                    if (info.info.m_guid == itt->first)
                    {
                        RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   remove topic belongs to %s", itt->second.name.c_str());
                        auto vrt = std::find(removedTopics_.begin(), removedTopics_.end(), itt->first);
                        if (vrt == removedTopics_.end())
                        {
                            removedTopics_.insert(itt->first);
                        }
                        itt = discoveredTopics_.erase(itt);
                    }
                    else
                    {
                        ++itt;
                    }
                }
                removedParticipants_.insert(info.info.m_guid);
                infoUpdated_ = true;
                infoUpdatedTs_ = now();
            }
            break;
        }
        default:
            return;
        }
    }

    /** Returns current time in seconds. */
    uint64_t now()
    {
        auto time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    }

private:
    const void *message_type_impl_;
    mutable std::mutex mutex_;
    bool infoUpdated_;
    bool fullUpdateRequest_;
    bool on_shutdown;
    uint64_t infoUpdatedTs_;
    uint infoUpdatedSkipped_;
    rclcpp::TimerBase::SharedPtr timer_;
    eprosima::fastrtps::Participant *participant_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    eprosima::fastrtps::Subscriber *subscriber_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    rclcpp::Publisher<fkie_multimaster_msgs::msg::DiscoveredState>::SharedPtr publisher_;
    rclcpp::Subscription<fkie_multimaster_msgs::msg::Endpoint>::SharedPtr subscriber_daemon_;
    std::string daemon_topic_;
    double stamp_;
    paricipant_map_t discoveredParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    topic_map_t discoveredTopics_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

    std::set<eprosima::fastrtps::rtps::GUID_t> removedTopics_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    std::set<eprosima::fastrtps::rtps::GUID_t> removedParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    std::set<eprosima::fastrtps::rtps::GUID_t> updatedParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    char hostname_chars[HOST_NAME_MAX];
    gethostname(hostname_chars, HOST_NAME_MAX);
    // std::string ros_distro = getEnvironmentVariable("ROS_DISTRO");
    std::string node_name = "_discovery_" + std::string(hostname_chars);
    auto listener = std::make_shared<CustomParticipantListener>(node_name, "/node_manager");
    rclcpp::spin(listener);
    RCLCPP_INFO(listener->get_logger(), "shutdown...");
    listener->shutdown();
    rclcpp::shutdown();
    printf("bye!\n");
    return 0;
}
