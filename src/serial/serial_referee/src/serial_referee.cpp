#include "serial_referee/serial_referee.h"

namespace Referee {

SerialRefereeNode::SerialRefereeNode(const rclcpp::NodeOptions& options)
: Node("rm_serial_referee", options) {
    this->declare_parameter<std::string>("referee_device_name", "/dev/ttyACM1"); // Make sure to declare before getting
    this->get_parameter("referee_device_name", serial_name);
    RCLCPP_INFO(this->get_logger(), "Serial device name: %s", serial_name.c_str());
    setupPublishers();
    
    try {
        if (!openAndConfigureUART()) {
            throw std::runtime_error("Failed to open and configure UART on " + serial_name);
        }
        readLoop();
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s", serial_name.c_str(), ex.what());
        throw ex;
    }
}

void SerialRefereeNode::setupPublishers() {
    game_status_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameStatus>("/referee/status", 10);
    game_HP_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameHp>("/referee/hp", 10);
    game_command_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameCommand>("/referee/command", 10);
    game_hurt_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameHurt>("/referee/hurt", 10);
    game_sentry_cmd_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameSentryCmd>("/referee/sentry_cmd", 10);
    game_sentry_info_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameSentryInfo>("/referee/sentry_info", 10);
    game_projectile_allowance_publisher = this->create_publisher<serial_referee_interface::msg::MessageGameProjectileAllowance>("/referee/projectile_allowance", 10);

}

bool SerialRefereeNode::openAndConfigureUART() {
     try {
        uart_com_ = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_com_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART '%s': %s", serial_name.c_str(), std::strerror(errno));
            return false;
        }

        struct termios Opt;
        tcgetattr(uart_com_, &Opt);
        cfsetispeed(&Opt, B115200);
        cfsetospeed(&Opt, B115200);
        Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Input
        Opt.c_oflag &= ~OPOST;                         // Output
        Opt.c_cc[VMIN] = 0;
        Opt.c_cc[VTIME] = 1; // 0.1 seconds

        if (tcsetattr(uart_com_, TCSANOW, &Opt) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set UART parameters!");
            close(uart_com_);
            return false;
        }
        return true;
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "An error occurred while opening UART: %s", e.what());
        return false;
    }
}

void SerialRefereeNode::readLoop() {
    // 主循环，用于从 UART 读取和处理数据
    uint8_t frame_hex[1];
    while (rclcpp::ok()) {
        frame_hex[0] = {0x00}; // 注意索引应为0，之前的代码有误

        ssize_t n = read(uart_com_, &frame_hex, 1);
        if (n > 0) {
            uart_buff.push_back(frame_hex[0]);
            processInput();
        } else if (n == 0) {
            // 数据读取超时或没有数据，可根据需要处理
            continue;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Read error or disconnect, attempting to reopen UART.");
            // 关闭当前串口，然后尝试重新打开，直到成功为止
            close(uart_com_);
            while (!openAndConfigureUART() && rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Retrying to open UART...");
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 1秒后重试
            }
        }

    }
}

void SerialRefereeNode::processInput() {
    size_t eraseSize = 0;
    if (uart_buff.size() < sizeof(Head) + sizeof(Tail) + 1)
    {
        return ;
    }
    bool okHeadFound = false;

    for (size_t i = 0; i < uart_buff.size()- sizeof(Head) - sizeof(Tail); i++) {

        uint8_t *p = (uint8_t *)uart_buff.data() + i;
        int size = uart_buff.size() - i;
        Head head;
        memcpy(&head, p, sizeof(Head));

        if (checker_head(head)) {
            if (!okHeadFound) {
                eraseSize = i - 1;
            }
            continue;
        }
        int length = getLength(head);
        if (size < length) {
            okHeadFound = 1;
            continue;
        }

        Tail tail;
        memcpy(&tail, p + length - sizeof(Tail), sizeof(Tail));
        if (checker_all(head, tail, p)) {
            continue;
        }
        eraseSize = i + head.length;
        p = (uint8_t *)uart_buff.data() + i + 7;
        switch (head.cmd_id) {
           case 0x0001:
                if (head.length == 11)
                {
                    get_status(p);
                }
                break;
            case 0x0003:
                if (head.length == 32)
                {
                    get_HP(p);
                }
                break;
            case 0x0206:
                if (head.length == 1)
                {
                    get_hurt(p);
                }
                break;
            case 0x0303:
                if (head.length == 15)
                {
                    get_command(p);
                }
                break;
            case 0x0208:
                if (head.length == 6)
                {
                    get_projectile_allowance(p);
                }
                break;
            case 0x020D:
                    get_projectile_allowance(p);
                break;
        }
        eraseSize = i + size;
    }
    if (eraseSize <= uart_buff.size()) {
        uart_buff.erase(uart_buff.begin(), uart_buff.begin() + eraseSize);
    } else {
        uart_buff.clear();
    }
}


void SerialRefereeNode::get_projectile_allowance(uint8_t *msg) {
    ext_projectile_allowance_t ext_projectile_allowance = *(ext_projectile_allowance_t *)msg;
    auto projectile_allowance_msg = std::make_shared<serial_referee_interface::msg::MessageGameProjectileAllowance>();
    projectile_allowance_msg->projectile_allowance_17mm = ext_projectile_allowance.projectile_allowance_17mm;
    projectile_allowance_msg->projectile_allowance_42mm = ext_projectile_allowance.projectile_allowance_42mm;
    projectile_allowance_msg->remaining_gold_coin = ext_projectile_allowance.remaining_gold_coin;
    game_projectile_allowance_publisher->publish(*projectile_allowance_msg);
}


void SerialRefereeNode::get_status(uint8_t *msg) {
    ext_game_status_t ext_status = *(ext_game_status_t *)msg;
    auto status_msg = std::make_shared<serial_referee_interface::msg::MessageGameStatus>();
    status_msg->game_type = ext_status.game_type;
    status_msg->game_progress = ext_status.game_progress;
    status_msg->stage_remain_time = ext_status.stage_remain_time;
    status_msg->sync_time_stamp = ext_status.sync_time_stamp;
    game_status_publisher->publish(*status_msg);
}


void SerialRefereeNode::get_HP(uint8_t *msg) {
    ext_game_robot_HP_t ext_HP = *(ext_game_robot_HP_t *)msg;
    auto HP_msg = std::make_shared<serial_referee_interface::msg::MessageGameHp>();
    HP_msg->red_1_robot_hp = ext_HP.red_1_robot_hp;
    HP_msg->red_2_robot_hp = ext_HP.red_2_robot_hp;
    HP_msg->red_3_robot_hp = ext_HP.red_3_robot_hp;
    HP_msg->red_4_robot_hp = ext_HP.red_4_robot_hp;
    HP_msg->red_5_robot_hp = ext_HP.red_5_robot_hp;
    HP_msg->red_7_robot_hp = ext_HP.red_7_robot_hp;
    HP_msg->red_outpost_hp = ext_HP.red_outpost_hp;
    HP_msg->red_base_hp = ext_HP.red_base_hp;
    HP_msg->blue_1_robot_hp = ext_HP.blue_1_robot_hp;
    HP_msg->blue_2_robot_hp = ext_HP.blue_2_robot_hp;
    HP_msg->blue_3_robot_hp = ext_HP.blue_3_robot_hp;
    HP_msg->blue_4_robot_hp = ext_HP.blue_4_robot_hp;
    HP_msg->blue_5_robot_hp = ext_HP.blue_5_robot_hp;
    HP_msg->blue_7_robot_hp = ext_HP.blue_7_robot_hp;
    HP_msg->blue_outpost_hp = ext_HP.blue_outpost_hp;
    HP_msg->blue_base_hp = ext_HP.blue_base_hp;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "HP_msg red_outpost_HP :%d, blue_outpost_HP :%d", HP_msg->red_7_robot_hp, HP_msg->blue_7_robot_hp);

    game_HP_publisher->publish(*HP_msg);
}

void SerialRefereeNode::get_hurt(uint8_t *msg) {
    ext_robot_hurt_t ext_hurt = *(ext_robot_hurt_t *)msg;
    auto hurt_msg = std::make_shared<serial_referee_interface::msg::MessageGameHurt>();
    hurt_msg->armor_id = ext_hurt.armor_id;
    hurt_msg->hurt_type = ext_hurt.hurt_type;
    game_hurt_publisher->publish(*hurt_msg);
}

void SerialRefereeNode::get_command(uint8_t *msg) {
  ext_robot_command_t ext_command = *(ext_robot_command_t *)msg;
    auto command_msg = std::make_shared<serial_referee_interface::msg::MessageGameCommand>();
    command_msg->target_position_x = ext_command.target_position_x;
    command_msg->target_position_y = ext_command.target_position_y;
    command_msg->target_position_z = ext_command.target_position_z;
    command_msg->command_keyboard = ext_command.command_keyboard;
    command_msg->target_robot_id = ext_command.target_robot_id;
    game_command_publisher->publish(*command_msg);
}

void SerialRefereeNode::get_sentry_info(uint8_t *msg) {
    ext_sentry_info_t ext_sentry_info = *(ext_sentry_info_t *)msg;
    
    auto command_msg = std::make_shared<serial_referee_interface::msg::MessageGameSentryInfo>();
    
    command_msg->success_ammo = (ext_sentry_info.sentry_cmd & 0x7FF);  // 提取0-10位
    command_msg->success_remote_ammo = (ext_sentry_info.sentry_cmd >> 11) & 0xF;  // 提取11-14位
    command_msg->success_remote_hp = (ext_sentry_info.sentry_cmd >> 15) & 0xF;  // 提取15-18位

    game_sentry_info_publisher->publish(*command_msg);
}


}  // namespace Referee

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Referee::SerialRefereeNode)
