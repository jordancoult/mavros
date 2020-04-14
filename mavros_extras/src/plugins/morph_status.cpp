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
#include <std_msgs/UInt8.h>
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
        morph_mode_sub = nh.subscribe("mode", 10, &MorphPlugin::mode_cb, this);
        // to transmit more data, I should make a mavros msg definition i think. See onboard_computer_status as an example

        //morph_mode_sub = nh.subscribe("raw_cg", 10, &MorphPlugin::raw_cg_cb, this);
        //morph_mode_sub = nh.subscribe("filtered_cg", 10, &MorphPlugin:filtered_cg_cb, this);
        //morph_mode_sub = nh.subscribe("ct", 10, &MorphPlugin::ct_cb, this);
        //morph_mode_sub = nh.subscribe("shutter", 10, &MorphPlugin::shutter_cb, this);
        //morph_mode_sub = nh.subscribe("angles", 10, &MorphPlugin::angles_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber morph_mode_sub;

    /*
    void mode_cb(const std_msgs::UInt8::ConstPtr &req)//std_msgs::UInt8::ConstPtr &req)
    {
        std::cout << "(Helloworld message from morphstatus.cpp) Got Int : " << req->data <<  std::endl;

        UAS_FCU(m_uas)->send_message_ignore_drop(req->data);
    }
    */

    void mode_cb(const mavros_msgs::MorphStatus::ConstPtr &req)//std_msgs::UInt8::ConstPtr &req)
    {
        uint8_t mode = req->mode;
        std::cout << "(Helloworld message from morph_status.cpp) Got Int : " << mode <<  std::endl;
        mavlink::common::msg::MORPH_STATUS status {};  // this was added later. Do i have to change any other files to make this work? yes. add MORPH_STATUS msg

        status.mode = mode;//data;
        // "data" is defeined in the type's header file (UInt8)

        UAS_FCU(m_uas)->send_message_ignore_drop(status);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MorphPlugin, mavros::plugin::PluginBase)
