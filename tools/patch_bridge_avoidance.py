from pathlib import Path
p=(Path(__file__).resolve().parents[1] / 'src/pixhawk_bridge/src/pixhawk_node.cpp')
s=p.read_text()
assert 'avoidance_advisory_tx' not in s
s=s.replace('#include <pixhawk_interfaces/msg/mission_item.hpp>',
  '#include <pixhawk_interfaces/msg/mission_item.hpp>\n#include <pixhawk_interfaces/msg/avoidance_advisory.hpp>\n#include "../../../avoidance_core/AvoidanceWire.hpp"')
s=s.replace('    raw_enabled_=declare_parameter<bool>("publish_raw_mavlink",false);',
'''    raw_enabled_=declare_parameter<bool>("publish_raw_mavlink",false);
    avoidance_tx_=declare_parameter<bool>("enable_avoidance_advisory_tx",false);
    avoidance_fw_confirmed_=declare_parameter<bool>("avoidance_firmware_confirmed",false);
    if(avoidance_tx_ && (!enable_motion_ || !avoidance_fw_confirmed_))
      throw std::runtime_error("Avoidance TX needs enable_motion_commands and explicit firmware confirmation");
    const auto now_session=static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    avoidance_session_=now_session?now_session:1;''')
s=s.replace('    setup_services();',
'''    avoidance_sub_=create_subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>(
      "avoidance/advisory",rclcpp::QoS(5),
      [this](const pixhawk_interfaces::msg::AvoidanceAdvisory::ConstSharedPtr m) {
        const bool finite=std::isfinite(m->vx_limit_mps) &&
          std::isfinite(m->yaw_correction_radps)&&
          std::isfinite(m->front_clearance_m)&&
          std::isfinite(m->left_clearance_m)&&
          std::isfinite(m->right_clearance_m);
        if(!finite||m->vx_limit_mps<0||m->vx_limit_mps>5||
           std::abs(m->yaw_correction_radps)>3||
           m->front_clearance_m<0||m->left_clearance_m<0||m->right_clearance_m<0)return;
        std::lock_guard<std::mutex> lk(avoidance_mutex_);
        avoidance_last_=*m;
        avoidance_last_rx_=std::chrono::steady_clock::now();
        avoidance_received_=true;
      });
    setup_services();''')
s=s.replace('bool enable_motion_=false,allow_param_write_=false,allow_mode_=false,allow_mission_=false,raw_enabled_=false,auto_sensor_streams_=true;',
'''bool enable_motion_=false,allow_param_write_=false,allow_mode_=false,allow_mission_=false,raw_enabled_=false,auto_sensor_streams_=true;
  bool avoidance_tx_{false},avoidance_fw_confirmed_{false};
  uint32_t avoidance_session_{0};
  uint16_t avoidance_seq_{0};
  bool avoidance_received_{false};
  std::mutex avoidance_mutex_;
  std::chrono::steady_clock::time_point avoidance_last_rx_{};
  pixhawk_interfaces::msg::AvoidanceAdvisory avoidance_last_;
  rclcpp::Subscription<pixhawk_interfaces::msg::AvoidanceAdvisory>::SharedPtr avoidance_sub_;''')
anchor='    if(now-last_sp_>=100ms){'
assert anchor in s
s=s.replace(anchor,'''    if (avoidance_tx_ && avoidance_fw_confirmed_ && enable_motion_ &&
        connected() && airframe_valid_ && now-last_avoidance_send_>=100ms) {
      last_avoidance_send_=now;
      pixhawk_interfaces::msg::AvoidanceAdvisory input;
      bool fresh=false;
      {
        std::lock_guard<std::mutex> lk(avoidance_mutex_);
        fresh=avoidance_received_ && (now-avoidance_last_rx_<250ms);
        if(fresh)input=avoidance_last_;
      }
      if(fresh) {
        avoidance::AdvisoryWire payload{};
        payload.flags=avoidance::VALID |
          (input.obstacle?avoidance::OBSTACLE:0) |
          ((input.stop||!input.valid)?avoidance::STOP:0);
        payload.sequence=++avoidance_seq_;
        payload.session=avoidance_session_;
        payload.vx_limit_mps=input.valid?input.vx_limit_mps:0.f;
        payload.yaw_correction_radps=input.valid?input.yaw_correction_radps:0.f;
        payload.front_clearance_m=input.front_clearance_m;
        payload.left_clearance_m=input.left_clearance_m;
        payload.right_clearance_m=input.right_clearance_m;
        avoidance::seal(payload);
        mavlink_message_t packet{};
        mavlink_msg_tunnel_pack(245,MAV_COMP_ID_ONBOARD_COMPUTER,&packet,
          target_sys_,target_comp_,avoidance::kTunnelPayloadType,sizeof(payload),
          reinterpret_cast<const uint8_t*>(&payload));
        send(packet);
      }
    }
'''+anchor)
s=s.replace('std::chrono::steady_clock::time_point last_send_heartbeat_{},last_status_{},last_sp_{},start_=std::chrono::steady_clock::now();',
'''std::chrono::steady_clock::time_point last_send_heartbeat_{},last_status_{},last_sp_{},
    last_avoidance_send_{},start_=std::chrono::steady_clock::now();''')
p.write_text(s)
p=(Path(__file__).resolve().parents[1] / 'src/pixhawk_bridge/launch/pixhawk.launch.py')
s=p.read_text()
s=s.replace("                'enable_motion_commands': False,",
"""                'enable_motion_commands': False,
                'enable_avoidance_advisory_tx': False,
                'avoidance_firmware_confirmed': False,""")
p.write_text(s)
p=(Path(__file__).resolve().parents[1] / 'src/pixhawk_bridge/config/default.yaml')
s=p.read_text()
s=s.replace('    enable_motion_commands: false',
'''    enable_motion_commands: false
    enable_avoidance_advisory_tx: false
    avoidance_firmware_confirmed: false''')
p.write_text(s)
print('Added opt-in, firmware-confirmation-gated MAVLink TUNNEL advisory TX. Defaults remain read-only.')
