/**
 * @brief Morphing Status Plugin
 * @file morphing.cpp
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

namespace mavros {
namespace extra_plugins{
/**
 * @brief Morphing Status plugin
 *
 * Publishes morphing data
 * @see status_cb()
 */
class MorphingPlugin : public plugin::PluginBase {
public:
    MorphingPlugin() : PluginBase(),
         nh("~morph")
    { }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        morph_mode_sub = nh.subscribe("mode", 10, &MorphingPlugin::mode_cb, this);
        // to transmit more data, I should make a mavros msg definition i think. See onboard_computer_status as an example

        //morph_mode_sub = nh.subscribe("raw_cg", 10, &MorphingPlugin::raw_cg_cb, this);
        //morph_mode_sub = nh.subscribe("filtered_cg", 10, &MorphingPlugin:filtered_cg_cb, this);
        //morph_mode_sub = nh.subscribe("ct", 10, &MorphingPlugin::ct_cb, this);
        //morph_mode_sub = nh.subscribe("shutter", 10, &MorphingPlugin::shutter_cb, this);
        //morph_mode_sub = nh.subscribe("angles", 10, &MorphingPlugin::angles_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber morph_mode_sub;

    void mode_cb(const std_msgs::UInt8::ConstPtr &req)
    {
        // possible error: UInt8 is a saved type variable in c? Doesn't turn blue for Char
        // (checking source, UInt8 is the correct name)
        std::cout << "(Helloworld message from morphing.cpp) Got Int : " << req->data <<  std::endl;
        mavlink::common::msg::MORPH_STATUS status {};  // this was added later. Do i have to change any other files to make this work?

        status.mode = req->data;
        // "data" is defeined in the type's header file (UInt8)
        // possible error: i removed a couple semicolons because they weren't there for other files

        UAS_FCU(m_uas)->send_message_ignore_drop(status, req->data);
    }
};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MorphingPlugin, mavros::plugin::PluginBase)
