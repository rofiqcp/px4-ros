from pathlib import Path
p=(Path(__file__).resolve().parents[1] / 'src/pixhawk_bridge/src/pixhawk_node.cpp')
s=p.read_text()
assert 'rover_feedback_pub_' not in s
s=s.replace('    mission_pub_=create_publisher<std_msgs::msg::String>("mission/events",10);',
'''    mission_pub_=create_publisher<std_msgs::msg::String>("mission/events",10);
    rover_feedback_pub_=create_publisher<std_msgs::msg::String>("rover/feedback",10);''')
s=s.replace('  std::vector<rclcpp::ServiceBase::SharedPtr> services_;',
'''  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rover_feedback_pub_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;''')
anchor='      case MAVLINK_MSG_ID_MISSION_COUNT:{'
assert anchor in s
s=s.replace(anchor,'''      case MAVLINK_MSG_ID_MISSION_CURRENT:{
        mavlink_mission_current_t v{};mavlink_msg_mission_current_decode(&m,&v);
        std_msgs::msg::String evt;
        evt.data="{\\\"type\\\":\\\"MISSION_CURRENT\\\",\\\"seq\\\":"+
          std::to_string(v.seq)+",\\\"total\\\":"+std::to_string(v.total)+"}";
        mission_pub_->publish(evt);break;
      }
      case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:{
        mavlink_mission_item_reached_t v{};mavlink_msg_mission_item_reached_decode(&m,&v);
        std_msgs::msg::String evt;evt.data=
          "{\\\"type\\\":\\\"MISSION_ITEM_REACHED\\\",\\\"seq\\\":"+std::to_string(v.seq)+"}";
        mission_pub_->publish(evt);break;
      }
      case MAVLINK_MSG_ID_RC_CHANNELS:{
        mavlink_rc_channels_t v{};mavlink_msg_rc_channels_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream o;
        o<<"{\\\"type\\\":\\\"RC_CHANNELS\\\",\\\"ch1\\\":"<<v.chan1_raw
         <<",\\\"ch2\\\":"<<v.chan2_raw<<",\\\"ch3\\\":"<<v.chan3_raw
         <<",\\\"ch4\\\":"<<v.chan4_raw<<",\\\"rssi\\\":"<<static_cast<int>(v.rssi)<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
      case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:{
        mavlink_servo_output_raw_t v{};mavlink_msg_servo_output_raw_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream o;
        // Servo PWM is command/actuator output, NOT a measured wheel steering angle.
        o<<"{\\\"type\\\":\\\"SERVO_OUTPUT_RAW\\\",\\\"port\\\":"<<static_cast<int>(v.port)
         <<",\\\"servo1_pwm\\\":"<<v.servo1_raw
         <<",\\\"servo2_pwm\\\":"<<v.servo2_raw
         <<",\\\"servo3_pwm\\\":"<<v.servo3_raw
         <<",\\\"servo4_pwm\\\":"<<v.servo4_raw<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
      case MAVLINK_MSG_ID_VFR_HUD:{
        mavlink_vfr_hud_t v{};mavlink_msg_vfr_hud_decode(&m,&v);
        std_msgs::msg::String msg;std::ostringstream o;
        o<<"{\\\"type\\\":\\\"VFR_HUD\\\",\\\"ground_speed_mps\\\":"<<json_num(v.groundspeed)
         <<",\\\"heading_deg\\\":"<<v.heading<<"}";
        msg.data=o.str();rover_feedback_pub_->publish(msg);break;
      }
'''+anchor)
needle='        attitude_pub_->publish(out);break;'
assert needle in s
s=s.replace(needle,'''        attitude_pub_->publish(out);
        std_msgs::msg::String feedback;
        feedback.data="{\\\"type\\\":\\\"ATTITUDE_QUATERNION\\\",\\\"yaw_rate_flu_radps\\\":"+
          json_num(-v.yawspeed)+"}";
        rover_feedback_pub_->publish(feedback);
        break;''',1)
p.write_text(s)
print('MAVLink mission/RC/servo/groundspeed/yawrate read-only feedback added.')
