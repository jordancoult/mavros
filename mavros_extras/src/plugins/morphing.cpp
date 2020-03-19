 #include <mavros/mavros_plugin.h>
 #include <pluginlib/class_list_macros.h>
 #include <iostream>
 #include <std_msgs/UInt8.h>

 namespace mavros {
 namespace extra_plugins{

 class MorphingPlugin : public plugin::PluginBase {
 public:
     MorphingPlugin() : PluginBase(),
         nh("~morph")

    { };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         morph_mode_sub = nh.subscribe("mode", 10, &MorphingPlugin::mode_cb, this);
         //morph_mode_sub = nh.subscribe("raw_cg", 10, &MorphingPlugin::raw_cg_cb, this);
         //morph_mode_sub = nh.subscribe("filtered_cg", 10, &MorphingPlugin:filtered_cg_cb, this);
         //morph_mode_sub = nh.subscribe("ct", 10, &MorphingPlugin::ct_cb, this);
         //morph_mode_sub = nh.subscribe("shutter", 10, &MorphingPlugin::shutter_cb, this);
         //morph_mode_sub = nh.subscribe("angles", 10, &MorphingPlugin::angles_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber morph_mode_sub;

    void mode_cb(const std_msgs::Int32::ConstPtr &req)
     {
         std::cout << "Got Int : " << req->data <<  std::endl;
         UAS_FCU(m_uas)->send_message_ignore_drop(req->data);
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MorphingPlugin, mavros::plugin::PluginBase)
