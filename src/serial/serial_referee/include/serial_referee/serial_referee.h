#ifndef SERIAL_REFEREE_H
#define SERIAL_REFEREE_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "serial_referee_interface/msg/message_game_status.hpp"
#include "serial_referee_interface/msg/message_game_hurt.hpp"
#include "serial_referee_interface/msg/message_game_command.hpp"
#include "serial_referee_interface/msg/message_game_hp.hpp"
#include "serial_referee_interface/msg/message_game_sentry_cmd.hpp"
#include "serial_referee_interface/msg/message_game_projectile_allowance.hpp"
#include "serial_referee_interface/msg/message_game_sentry_info.hpp"
#include <iostream>
#include <string>
#include "serial_referee/uart.h"
#include "serial_referee/check.h"
#include <stdexcept>
#include "serial_referee_interface/msg/referee_info.hpp"

namespace Referee {

// Declare all data structures and types
struct ext_game_sentry_cmd {
    uint32_t sentry_cmd;
} __attribute__((packed));

struct ext_game_status_t {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_time_stamp;
} __attribute__((packed));

struct ext_game_robot_HP_t {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_5_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_5_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
} __attribute__((packed));

struct ext_robot_hurt_t {
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} __attribute__((packed));

struct ext_robot_command_t {
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t command_keyboard;
    uint16_t target_robot_id;
} __attribute__((packed));

struct ext_sentry_info_t {
    uint32_t sentry_cmd;
} __attribute__((packed));


struct ext_projectile_allowance_t {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} __attribute__((packed));

class SerialRefereeNode : public rclcpp::Node {
public:
    explicit SerialRefereeNode(const rclcpp::NodeOptions& options);

private:
    void get_projectile_allowance(uint8_t *msg);
    void get_status(uint8_t *msg);
    void get_HP(uint8_t *msg);
    void get_hurt(uint8_t *msg);
    void get_command(uint8_t *msg);
    void get_sentry_info(uint8_t *msg);
    void processInput();
    void readLoop();
    bool openAndConfigureUART();
    void setupPublishers();

    std::vector<uint8_t> uart_buff;
    std::string serial_name;
    int uart_com_;
    rclcpp::Publisher<serial_referee_interface::msg::RefereeInfo>::SharedPtr referee_info_pub_;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameStatus>::SharedPtr game_status_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameHp>::SharedPtr game_HP_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameCommand>::SharedPtr game_command_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameHurt>::SharedPtr game_hurt_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameSentryCmd>::SharedPtr game_sentry_cmd_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameSentryInfo>::SharedPtr game_sentry_info_publisher;
    rclcpp::Publisher<serial_referee_interface::msg::MessageGameProjectileAllowance>::SharedPtr game_projectile_allowance_publisher;
};

} // namespace Referee

#endif // SERIAL_REFEREE_H
