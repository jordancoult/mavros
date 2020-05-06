/**
 * @brief Morph Status Plugin
 * @file morph.cpp
 * @author Jordan Coult <jordan@robodub.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2020 Jordan Coult.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <mavros_msgs/MorphStatus.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Morph Status plugin
 *
 * Publishes morph data
 * @see mode_cb()
 */
class MorphPlugin : public plugin::PluginBase {
public:
    MorphPlugin() : PluginBase(),
         nh("~morph")
    { }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        morph_mode_sub = nh.subscribe("status", 10, &MorphPlugin::status_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber morph_mode_sub;


    void status_cb(const mavros_msgs::MorphStatus::ConstPtr &req)//std_msgs::UInt8::ConstPtr &req)
    {   
        // recieved by mavros and sent over mavlink to fcu

        std::cout << "(Helloworld message from morph_status.cpp in mavros)";
        mavlink::common::msg::MORPH_STATUS status {};  // this was added later. Do i have to change any other files to make this work? yes. add MORPH_STATUS msg       
        
        status.time_usec = req->header.stamp.toNSec() / 1000;                   //!< [microsecs]
        status.mode = req->mode;
        std::copy(req->angles.cbegin(), req->angles.cend(), status.angles.begin());
        std::copy(req->raw_cg.cbegin(), req->raw_cg.cend(), status.raw_cg.begin());
        std::copy(req->filt_cg.cbegin(), req->filt_cg.cend(), status.filt_cg.begin());
        std::copy(req->ct.cbegin(), req->ct.cend(), status.ct.begin());
        status.shutter_open = uint8_t(req->shutter_open);

        UAS_FCU(m_uas)->send_message_ignore_drop(status);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MorphPlugin, mavros::plugin::PluginBase)
