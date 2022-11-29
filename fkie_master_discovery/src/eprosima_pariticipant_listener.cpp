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

#ifdef rmw_dds_common_FOUND
#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#include "rmw_dds_common/msg/detail/participant_entities_info__struct.hpp"
#include "rmw_fastrtps_shared_cpp/guid_utils.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"
#endif

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
#include "fkie_multimaster_msgs/msg/guid.hpp"
#include "fkie_multimaster_msgs/msg/node_entities_info.hpp"
#include "fkie_multimaster_msgs/msg/participant_entities_info.hpp"
#include "fkie_multimaster_msgs/msg/topic_entity.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

using namespace std::chrono_literals;

struct NodeInfo
{
    std::string name;
    std::string ns;
    std::vector<fkie_multimaster_msgs::msg::Guid> publisher;
    std::vector<fkie_multimaster_msgs::msg::Guid> subscriber;
};

struct ParticipantInfo
{
    std::string enclave;
    std::vector<std::string> unicast_locators;
    std::vector<NodeInfo> nodes;
};

struct TopicInfo
{
    std::string name;
    std::string ttype;
    int8_t info;
    eprosima::fastrtps::rtps::GUID_t participant_guid;
#ifdef rmw_dds_common_FOUND
    rmw_qos_profile_t qos = rmw_qos_profile_unknown;
#endif
};

using paricipant_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, ParticipantInfo>;
using topic_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, TopicInfo>;


/**
 * Get a value from a environment variable
 */
std::string getEnvironmentVariable(std::string const &key)
{
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
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
        //qos_settings.reliable();
        //qos_settings.transient_local();
        on_shutdown = false;
        subscriber_count = 0;
        publisher_ = this->create_publisher<fkie_multimaster_msgs::msg::DiscoveredState>("~/rosstate", qos_settings);
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
#ifdef rmw_dds_common_FOUND
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
#endif
        timer_ = this->create_wall_timer(1000ms, std::bind(&CustomParticipantListener::_callback_timer_state, this));
    }

    void shutdown()
    {
        this->on_shutdown = true;
        timer_->cancel();
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
        auto current_subscriber_count = this->count_subscribers("~/rosstate");
        if (current_subscriber_count != subscriber_count)
        {
            if (current_subscriber_count > subscriber_count && current_subscriber_count > 0) {
                infoUpdated_ = true;
            }
            subscriber_count = current_subscriber_count;
        }
        if (doUpdate)
        {
            // publish all participants
            uint64_t newStamp = now();
            std::lock_guard<std::mutex> guard(mutex_);
            fkie_multimaster_msgs::msg::DiscoveredState rosmsg;
            rosmsg.stamp = newStamp;
            for (auto itdp = discoveredParticipants_.begin(); itdp != discoveredParticipants_.end(); itdp++)
            {
                fkie_multimaster_msgs::msg::ParticipantEntitiesInfo rospt;
                rospt.guid = guid_to_msg(itdp->first);
                rospt.enclave = itdp->second.enclave;
                for (auto ul = itdp->second.unicast_locators.begin(); ul != itdp->second.unicast_locators.end(); ul++)
                {
                    rospt.unicast_locators.push_back(*ul);
                }
                for (auto un = itdp->second.nodes.begin(); un != itdp->second.nodes.end(); un++)
                {
                    fkie_multimaster_msgs::msg::NodeEntitiesInfo rosnd;
                    rosnd.name = un->name;
                    rosnd.ns = un->ns;
                    // copy GUIDs of all publisher associated with this node
                    for (auto unp = un->publisher.begin(); unp != un->publisher.end(); unp++)
                    {
                        rosnd.publisher.push_back(*unp);
                    }
                    // copy GUIDs of all subscriber associated with this node
                    for (auto uns = un->subscriber.begin(); uns != un->subscriber.end(); uns++)
                    {
                        rosnd.subscriber.push_back(*uns);
                    }
                    RCLCPP_INFO(get_logger(), "add node to participant; guid=%s, node=%s", to_string(itdp->first).c_str(), rosnd.name.c_str());
                    rospt.node_entities.push_back(rosnd);
                }
                for (auto itdt = discoveredTopics_.begin(); itdt != discoveredTopics_.end(); itdt++)
                {
                    if (itdt->second.participant_guid == itdp->first)
                    {
                        fkie_multimaster_msgs::msg::TopicEntity rt;
                        rt.guid = guid_to_msg(itdt->first);
                        rt.name = itdt->second.name;
                        rt.ttype = itdt->second.ttype;
                        rt.info = itdt->second.info;
#ifdef rmw_dds_common_FOUND
                        rt.qos.durability = itdt->second.qos.durability;
                        rt.qos.history = itdt->second.qos.history;
                        rt.qos.depth = itdt->second.qos.depth;
                        rt.qos.liveliness = itdt->second.qos.liveliness;
                        rt.qos.reliability = itdt->second.qos.reliability;
                        rt.qos.deadline = from_rmw_time(itdt->second.qos.deadline);
                        rt.qos.lease_duration = from_rmw_time(itdt->second.qos.liveliness_lease_duration);
                        rt.qos.lifespan = from_rmw_time(itdt->second.qos.lifespan);
#endif
                        rospt.topic_entities.push_back(rt);
                    }
                }
                rosmsg.participants.push_back(rospt);
            }
            // add removed nodes
            rosmsg.since = stamp_;
            for (auto itrp = removedParticipants_.begin(); itrp != removedParticipants_.end(); itrp++)
            {
                fkie_multimaster_msgs::msg::ParticipantEntitiesInfo rospt;
                rospt.guid = guid_to_msg(itrp->first);
                for (auto itrpn = itrp->second.nodes.begin(); itrpn != itrp->second.nodes.end(); itrpn++)
                {
                    fkie_multimaster_msgs::msg::NodeEntitiesInfo rosnd;
                    rosnd.name = itrpn->name;
                    rosnd.ns = itrpn->ns;
                }
                rosmsg.removed_nodes.push_back(rospt);
            }
            removedParticipants_.clear();
            // add removed topics
            for (auto itrt = removedTopics_.begin(); itrt != removedTopics_.end(); itrt++)
            {
                rosmsg.removed_topics.push_back(guid_to_msg(*itrt));
            }
            removedTopics_.clear();
            RCLCPP_INFO(get_logger(), "pub ros graph");
            stamp_ = newStamp;
            publisher_->publish(rosmsg);
            infoUpdated_ = false;
            infoUpdatedSkipped_ = 0;
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
        if (this->on_shutdown) return;
        RCLCPP_DEBUG(get_logger(), "New ParticipantEntitiesInfo msg on 'ros_discovery_info', count unread: %lu", sub->get_unread_count());
#ifdef rmw_dds_common_FOUND
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
                ParticipantInfo pi;
                RCLCPP_INFO(get_logger(), "new participant discovered; guid=%s", to_string(guid).c_str());
                discoveredParticipants_[guid] = pi;
            }
            auto &participant = discoveredParticipants_[guid];
            std::vector<NodeInfo> old_nodes;
            for (auto it = participant.nodes.begin(); it != participant.nodes.end(); it++)
            {
                old_nodes.push_back(*it);
            }
            // create a new list with all nodes
            participant.nodes.clear();
            RCLCPP_INFO(get_logger(), "add discovered nodes for guid=%s", to_string(guid).c_str());
            for (auto &node : msg.node_entities_info_seq)
            {
                NodeInfo &ni = _get_node(participant, node.node_name, node.node_namespace);
                // update publisher/subscriber
                // ni.publisher.clear();
                for (auto &writer_gid : node.writer_gid_seq)
                {
                    fkie_multimaster_msgs::msg::Guid guid;
                    std::memcpy(&guid.data, &writer_gid, sizeof(guid.data));
                    ni.publisher.push_back(guid);
                }
                // ni.subscriber.clear();
                for (auto &reader_gid : node.reader_gid_seq)
                {
                    fkie_multimaster_msgs::msg::Guid guid;
                    std::memcpy(&guid.data, &reader_gid, sizeof(guid.data));
                    ni.subscriber.push_back(guid);
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
            if (old_nodes.size() > 0)
            {
                ParticipantInfo rpi;
                for (auto it = old_nodes.begin(); it != old_nodes.end(); it++)
                {
                    NodeInfo ni;
                    ni.name = it->name;
                    ni.ns = it->ns;
                    rpi.nodes.push_back(ni);
                }
                removedParticipants_[guid] = rpi;
            }
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Can't read ParticipantEntitiesInfo from 'ros_discovery_info'");
        }
#endif
    }

    NodeInfo &_get_node(ParticipantInfo &pi, std::string name, std::string ns)
    {
        for (auto &pi_node : pi.nodes)
        {
            if (name.compare(pi_node.name) == 0 && ns.compare(pi_node.ns) == 0)
            {
                return pi_node;
            }
        }
        RCLCPP_INFO(get_logger(), "new node discovered; ns=%s, name=%s, count already known nodes: %lu", ns.c_str(), name.c_str(), pi.nodes.size());
        NodeInfo ni;
        ni.name = name;
        ni.ns = ns;
        pi.nodes.push_back(ni);
        return pi.nodes.back();
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
            TopicInfo ti;
            auto itt = discoveredTopics_.find(proxyData.guid());
            if (itt != discoveredTopics_.end())
            {
                ti = itt->second;
            }
            ti.name = proxyData.topicName();
            ti.ttype = proxyData.typeName();
            ti.info = topicInfo;
            ti.participant_guid = participant_guid;
#ifdef rmw_dds_common_FOUND
            rtps_qos_to_rmw_qos(proxyData.m_qos, &ti.qos);
#endif
            discoveredTopics_[proxyData.guid()] = ti;
            RCLCPP_DEBUG(get_logger(), "new topic %s", ti.name.c_str());
        }
        else
        {
            auto itp = discoveredTopics_.find(proxyData.guid());
            if (itp != discoveredTopics_.end())
            {
                discoveredTopics_.erase(itp);
            }
            auto vrt = std::find(removedTopics_.begin(), removedTopics_.end(), proxyData.guid());
            if (vrt == removedTopics_.end())
            {
                removedTopics_.push_back(proxyData.guid());
            }
            RCLCPP_DEBUG(get_logger(), "remove topic %s", proxyData.topicName().c_str());
        }
        infoUpdated_ = true;
        infoUpdatedTs_ = now();
    }

    void onParticipantDiscovery(
        eprosima::fastrtps::Participant * /*participant*/,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info)
    {
        RCLCPP_DEBUG(get_logger(), "participant info %s", info.info.m_participantName.c_str());
        RCLCPP_DEBUG(get_logger(), "  status %d", info.status);
        std::lock_guard<std::mutex> guard(mutex_);
        auto participant_guid = to_string(info.info.m_guid);
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT:
        {
            RCLCPP_INFO(get_logger(), "new participant %s", participant_guid.c_str());
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            if (itp == discoveredParticipants_.end())
            {
                ParticipantInfo pi;
                discoveredParticipants_[info.info.m_guid] = pi;
            }
            auto pi = discoveredParticipants_[info.info.m_guid];
            auto map = rmw::impl::cpp::parse_key_value(info.info.m_userData);
            // get as defined since foxy
            auto enclave_found = map.find("enclave");
            if (enclave_found != map.end())
            {
                pi.enclave = std::string(enclave_found->second.begin(), enclave_found->second.end());
                RCLCPP_INFO(get_logger(), "  enclave '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            }
            else
            {
                // compatibility to eloquent version
                NodeInfo ni;
                auto name_found = map.find("name");
                auto ns_found = map.find("namespace");
                RCLCPP_INFO(get_logger(), " no enclave found for %s, pname: %s", participant_guid.c_str(), info.info.m_participantName.c_str());
                if (name_found != map.end())
                {
                    ni.name = std::string(name_found->second.begin(), name_found->second.end());
                    if (ns_found != map.end())
                    {
                        ni.ns = std::string(ns_found->second.begin(), ns_found->second.end());
                    }
                    pi.nodes.push_back(ni);
                }
            }
            for (auto i = info.info.default_locators.unicast.begin(); i != info.info.default_locators.unicast.end(); ++i)
            {
                RCLCPP_DEBUG(get_logger(), "add unicast locator: %s", to_string(*i).c_str());
                pi.unicast_locators.push_back(to_string(*i));
            }
            discoveredParticipants_[info.info.m_guid] = pi;
            infoUpdated_ = true;
            infoUpdatedTs_ = now();
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
                RCLCPP_INFO(get_logger(), "  remove participant %s with nodes[%lu]:", to_string(info.info.m_guid).c_str(), itp->second.nodes.size());
                for (auto &ni : itp->second.nodes)
                {
                    RCLCPP_INFO(get_logger(), "    - ns=%s, name:=%s", ni.ns.c_str(), ni.name.c_str());
                }
                discoveredParticipants_.erase(itp);
                // remove all topics of this participant ID
                for (auto itt = discoveredTopics_.begin(); itt != discoveredTopics_.end();)
                {
                    if (info.info.m_guid == itt->first)
                    {
                        RCLCPP_INFO(get_logger(), "  remove topic belongs to %s", itt->second.name.c_str());
                        auto vrt = std::find(removedTopics_.begin(), removedTopics_.end(), itt->first);
                        if (vrt == removedTopics_.end())
                        {
                            removedTopics_.push_back(itt->first);
                        }
                        itt = discoveredTopics_.erase(itt);
                    }
                    else
                    {
                        ++itt;
                    }
                }
                ParticipantInfo pi;
                removedParticipants_[info.info.m_guid] = pi;
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

    fkie_multimaster_msgs::msg::Guid guid_to_msg(const eprosima::fastrtps::rtps::GUID_t &guid)
    {
        fkie_multimaster_msgs::msg::Guid result;
        rmw_gid_t rmw_gid = {};
        rmw_fastrtps_shared_cpp::copy_from_fastrtps_guid_to_byte_array(guid, rmw_gid.data);
        std::memcpy(&result.data, rmw_gid.data, sizeof(rmw_gid.data));
        return result;
    }

private:
#ifdef rmw_dds_common_FOUND
    const void *message_type_impl_;
#endif
    mutable std::mutex mutex_;
    size_t subscriber_count;
    bool infoUpdated_;
    bool on_shutdown;
    uint64_t infoUpdatedTs_;
    uint infoUpdatedSkipped_;
    rclcpp::TimerBase::SharedPtr timer_;
    eprosima::fastrtps::Participant *participant_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    eprosima::fastrtps::Subscriber *subscriber_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    rclcpp::Publisher<fkie_multimaster_msgs::msg::DiscoveredState>::SharedPtr publisher_;
    double stamp_;
    paricipant_map_t discoveredParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    topic_map_t discoveredTopics_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    paricipant_map_t removedParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    std::vector<eprosima::fastrtps::rtps::GUID_t> removedTopics_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    char hostname_chars[HOST_NAME_MAX];
    gethostname(hostname_chars, HOST_NAME_MAX);
    std::string ros_distro = getEnvironmentVariable("ROS_DISTRO");
    std::string node_name = "_discovery_" + ros_distro + "_" + std::string(hostname_chars);
    auto listener = std::make_shared<CustomParticipantListener>(node_name, "/node_manager");
    rclcpp::spin(listener);
    RCLCPP_INFO(listener->get_logger(), "shutdown...");
    listener->shutdown();
    rclcpp::shutdown();
    printf("bye!\n");
    return 0;
}
