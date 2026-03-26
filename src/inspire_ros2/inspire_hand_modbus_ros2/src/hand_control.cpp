#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include "service_interfaces/srv/getangleact.hpp"
#include "service_interfaces/srv/getangleset.hpp"
#include "service_interfaces/srv/getposact.hpp"
#include "service_interfaces/srv/getposset.hpp"
#include "service_interfaces/srv/getspeedset.hpp"
#include "service_interfaces/srv/getforceact.hpp"
#include "service_interfaces/srv/getforceset.hpp"
#include "service_interfaces/srv/getcurrentact.hpp"
#include "service_interfaces/srv/gettemp.hpp"
#include "service_interfaces/srv/geterror.hpp"
#include "service_interfaces/srv/getstatus.hpp"

#include "service_interfaces/srv/setangle.hpp"
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setspeed.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/setforceclb.hpp"
#include "service_interfaces/srv/setclearerror.hpp"
#include "service_interfaces/srv/setcurrentlimit.hpp"
#include "service_interfaces/srv/setdefaultforce.hpp"
#include "service_interfaces/srv/setdefaultspeed.hpp"
#include "service_interfaces/srv/setid.hpp"
#include "service_interfaces/srv/setreduratio.hpp"
#include "service_interfaces/srv/setresetpara.hpp"
#include "service_interfaces/srv/setsaveflash.hpp"
#include "service_interfaces/srv/setgestureno.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Hand_control : public rclcpp::Node 
{
public:
    Hand_control()
    : Node("hand_modbus_control_node"), ctx_(nullptr)
    {
        // 从 config.yaml 读取 Modbus TCP 配置，读取失败则使用默认值
        std::string ip_address = "192.168.11.210";
        int port = 6000;

        try {
            const std::string share_dir = ament_index_cpp::get_package_share_directory("inspire_hand_modbus_ros2");
            const std::string config_path = share_dir + "/config/config.yaml";

            std::ifstream cfg(config_path);
            if (!cfg.is_open()) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to open config file: %s, using default Modbus IP/port",
                            config_path.c_str());
            } else {
                std::string line;
                while (std::getline(cfg, line)) {
                    // 去掉注释
                    const auto hash_pos = line.find('#');
                    if (hash_pos != std::string::npos) {
                        line = line.substr(0, hash_pos);
                    }

                    // 查找 ip
                    const auto ip_pos = line.find("ip:");
                    if (ip_pos != std::string::npos) {
                        std::string value = line.substr(ip_pos + 3);
                        // 去掉两端空白和引号
                        auto first = value.find_first_not_of(" \t\"");
                        auto last = value.find_last_not_of(" \t\"");
                        if (first != std::string::npos && last != std::string::npos && last >= first) {
                            ip_address = value.substr(first, last - first + 1);
                        }
                        continue;
                    }

                    // 查找 port
                    const auto port_pos = line.find("port:");
                    if (port_pos != std::string::npos) {
                        std::string value = line.substr(port_pos + 5);
                        auto first = value.find_first_not_of(" \t");
                        auto last = value.find_last_not_of(" \t");
                        if (first != std::string::npos && last != std::string::npos && last >= first) {
                            try {
                                port = std::stoi(value.substr(first, last - first + 1));
                            } catch (const std::exception &e) {
                                RCLCPP_WARN(this->get_logger(),
                                            "Failed to parse port from config.yaml: %s, using default %d",
                                            e.what(), port);
                            }
                        }
                        continue;
                    }
                }
                RCLCPP_INFO(this->get_logger(),
                            "Loaded Modbus config from %s, ip=%s, port=%d",
                            config_path.c_str(), ip_address.c_str(), port);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                        "Exception while loading config.yaml: %s, using default Modbus IP/port",
                        e.what());
        }

        // Modbus TCP 设置
        ctx_ = modbus_new_tcp(ip_address.c_str(), port);
        
        // 尝试连接到Modbus
        if (modbus_connect(ctx_) == -1) {
            RCLCPP_FATAL(this->get_logger(), "Unable to connect to Modbus TCP server: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to Modbus TCP server at %s:%d", ip_address.c_str(), port);

        // 创建服务
        Getangleact_Server = this->create_service<service_interfaces::srv::Getangleact>("Getangleact",
                                    std::bind(&Hand_control::getangleact_callback, this, _1, _2));

        Getangleset_Server = this->create_service<service_interfaces::srv::Getangleset>("Getangleset",
                                    std::bind(&Hand_control::getangleset_callback, this, _1, _2));

        Getposact_Server = this->create_service<service_interfaces::srv::Getposact>("Getposact",
                                    std::bind(&Hand_control::getposact_callback, this, _1, _2));

        Getposset_Server = this->create_service<service_interfaces::srv::Getposset>("Getposset",
                                    std::bind(&Hand_control::getposset_callback, this, _1, _2));

        Getspeedset_Server = this->create_service<service_interfaces::srv::Getspeedset>("Getspeedset",
                                    std::bind(&Hand_control::getspeedset_callback, this, _1, _2));

        Getforceact_Server = this->create_service<service_interfaces::srv::Getforceact>("Getforceact",
                                    std::bind(&Hand_control::getforceact_callback, this, _1, _2));

        Getforceset_Server = this->create_service<service_interfaces::srv::Getforceset>("Getforceset",
                                    std::bind(&Hand_control::getforceset_callback, this, _1, _2));

        Getcurrentact_Server = this->create_service<service_interfaces::srv::Getcurrentact>("Getcurrentact",
                                    std::bind(&Hand_control::getcurrentact_callback, this, _1, _2));
                                    
        Gettemp_Server = this->create_service<service_interfaces::srv::Gettemp>("Gettemp",
                                    std::bind(&Hand_control::gettemp_callback, this, _1, _2));
        
        Geterror_Server = this->create_service<service_interfaces::srv::Geterror>("Geterror",
            			    std::bind(&Hand_control::geterror_callback, this,_1,_2));
        
	Getstatus_Server = this->create_service<service_interfaces::srv::Getstatus>("Getstatus",
            			    std::bind(&Hand_control::getstatus_callback, this,_1,_2));                                                   
            	
            	
            			    
        Setangle_Server = this->create_service<service_interfaces::srv::Setangle>("Setangle",
                                    std::bind(&Hand_control::setangle_callback,this,_1,_2)); 
                                    
        Setpos_Server = this->create_service<service_interfaces::srv::Setpos>("Setpos",
                                    std::bind(&Hand_control::setpos_callback,this,_1,_2));

        Setspeed_Server = this->create_service<service_interfaces::srv::Setspeed>("Setspeed",
                                    std::bind(&Hand_control::setspeed_callback,this,_1,_2));

        Setforce_Server = this->create_service<service_interfaces::srv::Setforce>("Setforce",
                                    std::bind(&Hand_control::setforce_callback,this,_1,_2));
                                    
        Setforceclb_Server = this->create_service<service_interfaces::srv::Setforceclb>("Setforceclb",
                                    std::bind(&Hand_control::setforceclb_callback,this,_1,_2));
        
        Setcurrentlimit_Server = this->create_service<service_interfaces::srv::Setcurrentlimit>("Setcurrentlimit",
                                    std::bind(&Hand_control::setcurrentlimit_callback,this,_1,_2));
                                    
        Setdefaultforce_Server = this->create_service<service_interfaces::srv::Setdefaultforce>("Setdefaultforce",
                                    std::bind(&Hand_control::setdefaultforce_callback,this,_1,_2));                                    
                                    
        Setdefaultspeed_Server = this->create_service<service_interfaces::srv::Setdefaultspeed>("Setdefaultspeed",
                                    std::bind(&Hand_control::setdefaultspeed_callback,this,_1,_2));                                    
                                    
        Setid_Server = this->create_service<service_interfaces::srv::Setid>("Setid",
                                    std::bind(&Hand_control::setid_callback,this,_1,_2));                                    
                                    
        Setreduratio_Server = this->create_service<service_interfaces::srv::Setreduratio>("Setreduratio",
                                    std::bind(&Hand_control::setreduratio_callback,this,_1,_2));                                                                                                       
                      
        Setclearerror_Server = this->create_service<service_interfaces::srv::Setclearerror>("Setclearerror",
                                    std::bind(&Hand_control::setclearerror_callback,this,_1,_2));                            
                                    
        Setsaveflash_Server = this->create_service<service_interfaces::srv::Setsaveflash>("Setsaveflash",
                                    std::bind(&Hand_control::setsaveflash_callback,this,_1,_2));                                                     
                                    
        Setresetpara_Server = this->create_service<service_interfaces::srv::Setresetpara>("Setresetpara",
                                    std::bind(&Hand_control::setresetpara_callback,this,_1,_2));                                                        
                                    
        Setgestureno_Server = this->create_service<service_interfaces::srv::Setgestureno>("Setgestureno",
                                    std::bind(&Hand_control::setgestureno_callback,this,_1,_2));                              
                                                                  			    
    }

    ~Hand_control() {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
    }

private:
    modbus_t *ctx_;
    rclcpp::Service<service_interfaces::srv::Getangleact>::SharedPtr Getangleact_Server;
    rclcpp::Service<service_interfaces::srv::Getangleset>::SharedPtr Getangleset_Server;
    rclcpp::Service<service_interfaces::srv::Getposact>::SharedPtr Getposact_Server;
    rclcpp::Service<service_interfaces::srv::Getposset>::SharedPtr Getposset_Server;
    rclcpp::Service<service_interfaces::srv::Getspeedset>::SharedPtr Getspeedset_Server;
    rclcpp::Service<service_interfaces::srv::Getforceact>::SharedPtr Getforceact_Server;
    rclcpp::Service<service_interfaces::srv::Getforceset>::SharedPtr Getforceset_Server;
    rclcpp::Service<service_interfaces::srv::Getcurrentact>::SharedPtr Getcurrentact_Server;
    rclcpp::Service<service_interfaces::srv::Gettemp>::SharedPtr Gettemp_Server;
    rclcpp::Service<service_interfaces::srv::Geterror>::SharedPtr Geterror_Server;
    rclcpp::Service<service_interfaces::srv::Getstatus>::SharedPtr Getstatus_Server;

    
   
    rclcpp::Service<service_interfaces::srv::Setangle>::SharedPtr Setangle_Server;
    rclcpp::Service<service_interfaces::srv::Setpos>::SharedPtr Setpos_Server;
    rclcpp::Service<service_interfaces::srv::Setspeed>::SharedPtr Setspeed_Server;
    rclcpp::Service<service_interfaces::srv::Setforce>::SharedPtr Setforce_Server;
    rclcpp::Service<service_interfaces::srv::Setforceclb>::SharedPtr Setforceclb_Server;
    rclcpp::Service<service_interfaces::srv::Setclearerror>::SharedPtr Setclearerror_Server;
    rclcpp::Service<service_interfaces::srv::Setsaveflash>::SharedPtr Setsaveflash_Server;
    rclcpp::Service<service_interfaces::srv::Setresetpara>::SharedPtr Setresetpara_Server;    
    rclcpp::Service<service_interfaces::srv::Setcurrentlimit>::SharedPtr Setcurrentlimit_Server;    
    rclcpp::Service<service_interfaces::srv::Setdefaultforce>::SharedPtr Setdefaultforce_Server;
    rclcpp::Service<service_interfaces::srv::Setdefaultspeed>::SharedPtr Setdefaultspeed_Server;    
    rclcpp::Service<service_interfaces::srv::Setid>::SharedPtr Setid_Server;    
    rclcpp::Service<service_interfaces::srv::Setreduratio>::SharedPtr Setreduratio_Server;
    rclcpp::Service<service_interfaces::srv::Setgestureno>::SharedPtr Setgestureno_Server;
        
    void getangleact_callback(const service_interfaces::srv::Getangleact::Request::SharedPtr request,
                              const service_interfaces::srv::Getangleact::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Angle Actual values request received");

    // 定义寄存器地址和读取的角度值数组
    int register_addresses[6] = {1546, 1548, 1550, 1552, 1554, 1556};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的角度实际值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->curangle[i] = -1; // 读取失败
        } else {
            response->curangle[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->curangle[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read ANGLE_ACT(%d) value: %d", i, response->curangle[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read ANGLE_ACT(%d) value", i);
        }
    }
}


    void getangleset_callback(const service_interfaces::srv::Getangleset::Request::SharedPtr request,
                              const service_interfaces::srv::Getangleset::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Angle Set values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1486, 1488, 1490, 1492, 1494, 1496};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的上电初始角度
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->setangle[i] = -1; // 读取失败
        } else {
            response->setangle[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->setangle[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read ANGLE_SET(%d) value: %d", i, response->setangle[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read ANGLE_SET(%d) value", i);
        }
    }
}

    void getposact_callback(const service_interfaces::srv::Getposact::Request::SharedPtr request,
                            const service_interfaces::srv::Getposact::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Position Actual values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1534, 1536, 1538, 1540, 1542, 1544};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的驱动器实际位置值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->curpos[i] = -1; // 读取失败
        } else {
            response->curpos[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->curpos[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read POS_ACT(%d) value: %d", i, response->curpos[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read POS_ACT(%d) value", i);
        }
    }
}

    void getposset_callback(const service_interfaces::srv::Getposset::Request::SharedPtr request,
                            const service_interfaces::srv::Getposset::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Position Set values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1474, 1476, 1478, 1480, 1482, 1484};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的驱动器位置设置值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->setpos[i] = -1; // 读取失败
        } else {
            response->setpos[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->setpos[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read POS_SET(%d) value: %d", i, response->setpos[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read POS_SET(%d) value", i);
        }
    }
}

                               
    void getspeedset_callback(const service_interfaces::srv::Getspeedset::Request::SharedPtr request,
                          const service_interfaces::srv::Getspeedset::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Speed Set values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1522, 1524, 1526, 1528, 1530, 1532};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的驱动器速度设置值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->curspeedset[i] = -1; // 读取失败
        } else {
            response->curspeedset[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->curspeedset[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read SPEED_SET(%d) value: %d", i, response->curspeedset[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read SPEED_SET(%d) value", i);
        }
    }
}

    void getforceact_callback(const service_interfaces::srv::Getforceact::Request::SharedPtr request,
                          const service_interfaces::srv::Getforceact::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Force Actual values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1582, 1584, 1586, 1588, 1590, 1592};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的实际受力值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->curforce[i] = -1; // 读取失败
        } else {
            response->curforce[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        RCLCPP_INFO(this->get_logger(), "Read FORCE_ACT(%d) value: %d", i, response->curforce[i]);
    }
}

    void getforceset_callback(const service_interfaces::srv::Getforceset::Request::SharedPtr request,
                          const service_interfaces::srv::Getforceset::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Force Set values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1498, 1500, 1502, 1504, 1506, 1508};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的力控设置值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->setforce[i] = -1; // 读取失败
        } else {
            response->setforce[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->setforce[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read FORCE_SET(%d) value: %d", i, response->setforce[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read FORCE_SET(%d) value", i);
        }
    }
}

    void getcurrentact_callback(const service_interfaces::srv::Getcurrentact::Request::SharedPtr request,
                            const service_interfaces::srv::Getcurrentact::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Current values request received");

    // 定义寄存器地址
    int register_addresses[6] = {1594, 1596, 1598, 1600, 1602, 1604};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个电缸的电流值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->current[i] = -1; // 读取失败
        } else {
            response->current[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->current[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read CURRENT(%d) value: %d mA", i, response->current[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read CURRENT(%d) value", i);
        }
    }
}

                            
   void gettemp_callback(const service_interfaces::srv::Gettemp::Request::SharedPtr request,
                      const service_interfaces::srv::Gettemp::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get temperature request received");

    uint16_t tab_reg[6]; // 用于存储读取的寄存器值

    // 读取寄存器 (地址从 1618 开始)
    int rc = modbus_read_registers(ctx_, 1618, 6, tab_reg);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read temperature registers: %s", modbus_strerror(errno));
        response->success = false; // Error
        return; // 返回，不再执行后面的代码
    }

    // 解析温度值并存储到响应中
    for (int i = 0; i < 3; i++) {
        // 每个寄存器包含两个字节
        response->tempvalue[i * 2] = static_cast<int16_t>(tab_reg[i] & 0xFF);       // 低字节
        response->tempvalue[i * 2 + 1] = static_cast<int16_t>((tab_reg[i] >> 8) & 0xFF); // 高字节
        
        // 输出温度信息
        RCLCPP_INFO(this->get_logger(), "TEMP(%d): %d", i * 2, response->tempvalue[i * 2]);      // 输出低字节
        RCLCPP_INFO(this->get_logger(), "TEMP(%d): %d", i * 2 + 1, response->tempvalue[i * 2 + 1]); // 输出高字节
    }

    response->success = true; // 成功
}
    
    void geterror_callback(const service_interfaces::srv::Geterror::Request::SharedPtr request,
                           const service_interfaces::srv::Geterror::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Hand: Get error request received. Status: %s", request->status.c_str());

        uint16_t tab_reg[6]; // 用于存储读取的寄存器值

        // 根据 status 进行不同的处理
        if (request->status == "get_error") {
            // 读取寄存器 (地址从 1606 开始)
            int rc = modbus_read_registers(ctx_, 1606, 6, tab_reg);
            if (rc == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read error registers: %s", modbus_strerror(errno));
                response->success = false; // 读取失败
                return;
            }

            // 解析故障信息并存储到响应中
            for (int i = 0; i < 3; i++) {
                response->errorvalue[i] = static_cast<int16_t>(tab_reg[i]);
                RCLCPP_INFO(this->get_logger(), "ERROR(%d): %d", i, response->errorvalue[i]);
            }

            response->success = true; // 成功
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported status: %s", request->status.c_str());
            response->success = false; // 不支持的状态
        }
    }
    
    void getstatus_callback(const service_interfaces::srv::Getstatus::Request::SharedPtr request,
                        const service_interfaces::srv::Getstatus::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Get Status values request received");

    // 定义寄存器地址和读取的状态值数组
    int register_addresses[6] = {1612, 1614, 1616, 1618, 1620, 1622};
    uint16_t tab_reg[1]; // 用于存储读取的寄存器值

    // 循环读取各个手指的状态值
    for (int i = 0; i < 6; ++i) {
        int rc = modbus_read_registers(ctx_, register_addresses[i], 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read register at %d: %s", register_addresses[i], modbus_strerror(errno));
            response->statusvalue[i] = -1; // 读取失败
        } else {
            response->statusvalue[i] = static_cast<int16_t>(tab_reg[0]);
        }

        // 记录读取的值
        if (response->statusvalue[i] != -1) {
            RCLCPP_INFO(this->get_logger(), "Read STATUS(%d) value: %d", i, response->statusvalue[i]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read STATUS(%d) value", i);
        }
    }
}
    

void setangle_callback(const service_interfaces::srv::Setangle::Request::SharedPtr request,
                       const service_interfaces::srv::Setangle::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "hand: set angle");

    // 检查请求中的角度参数是否合法
    if ((request->angle0 >= -1 && request->angle0 <= 1000) &&
        (request->angle1 >= -1 && request->angle1 <= 1000) &&
        (request->angle2 >= -1 && request->angle2 <= 1000) &&
        (request->angle3 >= -1 && request->angle3 <= 1000) &&
        (request->angle4 >= -1 && request->angle4 <= 1000) &&
        (request->angle5 >= -1 && request->angle5 <= 1000))
    {
        // 将角度值写入 Modbus 寄存器
        uint16_t angles[6] = {
            static_cast<uint16_t>(request->angle0), 
            static_cast<uint16_t>(request->angle1), 
            static_cast<uint16_t>(request->angle2), 
            static_cast<uint16_t>(request->angle3), 
            static_cast<uint16_t>(request->angle4), 
            static_cast<uint16_t>(request->angle5)
        };

        // 写入所有角度
        modbus_write_registers(ctx_, 1486, 6, angles);
        
        // 读取设备的返回帧
        uint16_t response_data[6]; 
        int read_status = modbus_read_registers(ctx_, 1486, 6, response_data);
        
        if (read_status != -1) {
            // 判断写入的数据与读取的数据是否相同
            bool is_equal = true;
            for (int i = 0; i < 6; ++i) {
                if (angles[i] != response_data[i]) {
                    is_equal = false;
                    break;
                }
            }

            // 设置 angle_accepted 的值
            response->angle_accepted = is_equal;

            if (is_equal) {
                RCLCPP_INFO(this->get_logger(), "Write and read values match.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Write and read values do not match.");
            }

            // 打印返回帧内容
            RCLCPP_INFO(this->get_logger(), "Device response: ");
            for (int i = 0; i < read_status; ++i) {
                RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, response_data[i]);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read response data: %s", modbus_strerror(errno));
            response->angle_accepted = false; 
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: angle error! Angle values must be in the range of -1 to 1000.");
        response->angle_accepted = false;
    }
}
    
void setpos_callback(const service_interfaces::srv::Setpos::Request::SharedPtr request,
                     const service_interfaces::srv::Setpos::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "hand: set pos");

    // 检查请求中的位置参数是否合法
    if ((request->pos0 >= 0 && request->pos0 <= 2000) &&
        (request->pos1 >= 0 && request->pos1 <= 2000) &&
        (request->pos2 >= 0 && request->pos2 <= 2000) &&
        (request->pos3 >= 0 && request->pos3 <= 2000) &&
        (request->pos4 >= 0 && request->pos4 <= 2000) &&
        (request->pos5 >= 0 && request->pos5 <= 2000))
    {
        // 将位置值写入 Modbus 寄存器
        uint16_t positions[6] = {
            static_cast<uint16_t>(request->pos0), 
            static_cast<uint16_t>(request->pos1), 
            static_cast<uint16_t>(request->pos2), 
            static_cast<uint16_t>(request->pos3), 
            static_cast<uint16_t>(request->pos4), 
            static_cast<uint16_t>(request->pos5)
        };

        // 写入所有位置
        modbus_write_registers(ctx_, 1474, 6, positions);

        // 读取设备的返回帧
        uint16_t response_data[6]; 
        int read_status = modbus_read_registers(ctx_, 1474, 6, response_data);
        
        if (read_status != -1) {
            // 判断写入的数据与读取的数据是否相同
            bool is_equal = true;
            for (int i = 0; i < 6; ++i) {
                if (positions[i] != response_data[i]) {
                    is_equal = false;
                    break;
                }
            }

            // 设置 pos_accepted 的值
            response->pos_accepted = is_equal;

            if (is_equal) {
                RCLCPP_INFO(this->get_logger(), "Write and read values match.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Write and read values do not match.");
            }

            // 打印返回帧内容
            RCLCPP_INFO(this->get_logger(), "Device response: ");
            for (int i = 0; i < read_status; ++i) {
                RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, response_data[i]);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read response data: %s", modbus_strerror(errno));
            response->pos_accepted = false; 
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: pos error! Position values must be in the range of 0 to 2000.");
        response->pos_accepted = false;
    }
}

void setspeed_callback(const service_interfaces::srv::Setspeed::Request::SharedPtr request,
                       const service_interfaces::srv::Setspeed::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "hand: set speed");

    // 检查请求中的速度参数是否合法
    if ((request->speed0 >= 0 && request->speed0 <= 1000) &&
        (request->speed1 >= 0 && request->speed1 <= 1000) &&
        (request->speed2 >= 0 && request->speed2 <= 1000) &&
        (request->speed3 >= 0 && request->speed3 <= 1000) &&
        (request->speed4 >= 0 && request->speed4 <= 1000) &&
        (request->speed5 >= 0 && request->speed5 <= 1000))
    {
        // 将速度值写入 Modbus 寄存器
        uint16_t speeds[6] = {
            static_cast<uint16_t>(request->speed0), 
            static_cast<uint16_t>(request->speed1), 
            static_cast<uint16_t>(request->speed2), 
            static_cast<uint16_t>(request->speed3), 
            static_cast<uint16_t>(request->speed4), 
            static_cast<uint16_t>(request->speed5)
        };

        modbus_write_registers(ctx_, 1522, 6, speeds);

        // 读取设备的返回帧
        uint16_t response_data[6]; 
        int read_status = modbus_read_registers(ctx_, 1522, 6, response_data);
        
        if (read_status != -1) {
            // 判断写入的数据与读取的数据是否相同
            bool is_equal = true;
            for (int i = 0; i < 6; ++i) {
                if (speeds[i] != response_data[i]) {
                    is_equal = false;
                    break;
                }
            }

            // 设置 speed_accepted 的值
            response->speed_accepted = is_equal;

            if (is_equal) {
                RCLCPP_INFO(this->get_logger(), "Write and read values match.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Write and read values do not match.");
            }

            // 打印返回帧内容
            RCLCPP_INFO(this->get_logger(), "Device response: ");
            for (int i = 0; i < read_status; ++i) {
                RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, response_data[i]);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read response data: %s", modbus_strerror(errno));
            response->speed_accepted = false; // 读取失败时将 speed_accepted 设为 false
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: speed error! Speed values must be in the range of 0 to 1000.");
        response->speed_accepted = false;
    }
}

void setforce_callback(const service_interfaces::srv::Setforce::Request::SharedPtr request,
                       const service_interfaces::srv::Setforce::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "hand: set force");

    // 检查请求中的力控设置值是否合法
    if ((request->force0 >= 0 && request->force0 <= 3000) &&
        (request->force1 >= 0 && request->force1 <= 3000) &&
        (request->force2 >= 0 && request->force2 <= 3000) &&
        (request->force3 >= 0 && request->force3 <= 3000) &&
        (request->force4 >= 0 && request->force4 <= 3000) &&
        (request->force5 >= 0 && request->force5 <= 3000))
    {
        // 将力控设置值写入 Modbus 寄存器
        uint16_t forces[6] = {
            static_cast<uint16_t>(request->force0), 
            static_cast<uint16_t>(request->force1), 
            static_cast<uint16_t>(request->force2), 
            static_cast<uint16_t>(request->force3), 
            static_cast<uint16_t>(request->force4), 
            static_cast<uint16_t>(request->force5)
        };

        modbus_write_registers(ctx_, 1498, 6, forces);

        // 读取设备的返回帧
        uint16_t response_data[6]; 
        int read_status = modbus_read_registers(ctx_, 1498, 6, response_data);
        
        if (read_status != -1) {
            // 判断写入的数据与读取的数据是否相同
            bool is_equal = true;
            for (int i = 0; i < 6; ++i) {
                if (forces[i] != response_data[i]) {
                    is_equal = false;
                    break;
                }
            }

            // 设置 force_accepted 的值
            response->force_accepted = is_equal;

            if (is_equal) {
                RCLCPP_INFO(this->get_logger(), "Write and read values match.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Write and read values do not match.");
            }

            // 打印返回帧内容
            RCLCPP_INFO(this->get_logger(), "Device response: ");
            for (int i = 0; i < read_status; ++i) {
                RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, response_data[i]);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read response data: %s", modbus_strerror(errno));
            response->force_accepted = false; 
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: force error! Force values must be in the range of 0 to 3000.");
        response->force_accepted = false;
    }
}
  
    void setforceclb_callback(const service_interfaces::srv::Setforceclb::Request::SharedPtr request,
                           const service_interfaces::srv::Setforceclb::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Force Calibration request received");

    uint16_t calibration_value = 1000; // 要写入的校准值

    // 写入寄存器 1486 到 1496
    for (int register_address = 1486; register_address <= 1496; register_address += 2) {
        uint16_t tab_reg[1]; // 用于存储写入的数据
        tab_reg[0] = calibration_value; // 设置要写入的值

        int rc = modbus_write_registers(ctx_, register_address, 1, tab_reg);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register %d: %s", register_address, modbus_strerror(errno));
            response->setforce_clb_accepted = false; // 写入失败
            return; // 返回，不再继续执行
        }
    }

    // 延时 10 毫秒
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms

    // 向寄存器 1009 写入 1，进行力控校准
    uint16_t force_calibration_value = 1;
    uint16_t tab_reg_force[1]; // 用于存储写入的数据
    tab_reg_force[0] = force_calibration_value; // 设置要写入的值

    int rc = modbus_write_registers(ctx_, 1009, 1, tab_reg_force);
    
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register 1009: %s", modbus_strerror(errno));
        response->setforce_clb_accepted = false; // 写入失败
    } else {
        response->setforce_clb_accepted = true; // 写入成功
    }
}
    
    void setcurrentlimit_callback(const service_interfaces::srv::Setcurrentlimit::Request::SharedPtr request,
                                   const service_interfaces::srv::Setcurrentlimit::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Current Limit request received");

    // 检查请求中的电流保护值是否合法
    if ((request->current0 >= 0 && request->current0 <= 1500) &&
        (request->current1 >= 0 && request->current1 <= 1500) &&
        (request->current2 >= 0 && request->current2 <= 1500) &&
        (request->current3 >= 0 && request->current3 <= 1500) &&
        (request->current4 >= 0 && request->current4 <= 1500) &&
        (request->current5 >= 0 && request->current5 <= 1500))
    {
        // 将电流保护值写入 Modbus 寄存器
        uint16_t currents[6] = {
            static_cast<uint16_t>(request->current0), 
            static_cast<uint16_t>(request->current1), 
            static_cast<uint16_t>(request->current2), 
            static_cast<uint16_t>(request->current3), 
            static_cast<uint16_t>(request->current4), 
            static_cast<uint16_t>(request->current5)
        };

        response->current_limit_accepted = (modbus_write_registers(ctx_, 1020, 6, currents) == 0); // 写入所有电流保护值

        // 读取某个寄存器的值（小拇指电流保护值）
        uint16_t read_value[1];
        if (modbus_read_registers(ctx_, 1020, 1, read_value) != -1) {
            RCLCPP_INFO(this->get_logger(), "Read CURRENT_LIMIT(0) value: %d", read_value[0]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read CURRENT_LIMIT(0) value: %s", modbus_strerror(errno));
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: current limit error! Current values must be in the range of 0 to 1500.");
        response->current_limit_accepted = false;
    }
}
    
    void setdefaultforce_callback(const service_interfaces::srv::Setdefaultforce::Request::SharedPtr request,
                                  const service_interfaces::srv::Setdefaultforce::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Default Force request received");

    // 检查请求中的初始力控参数是否合法
    if ((request->force0 >= 0 && request->force0 <= 3000) &&
        (request->force1 >= 0 && request->force1 <= 3000) &&
        (request->force2 >= 0 && request->force2 <= 3000) &&
        (request->force3 >= 0 && request->force3 <= 3000) &&
        (request->force4 >= 0 && request->force4 <= 3000) &&
        (request->force5 >= 0 && request->force5 <= 3000))
    {
        // 将上电初始力控值写入 Modbus 寄存器
        uint16_t forces[6] = {
            static_cast<uint16_t>(request->force0), 
            static_cast<uint16_t>(request->force1), 
            static_cast<uint16_t>(request->force2), 
            static_cast<uint16_t>(request->force3), 
            static_cast<uint16_t>(request->force4), 
            static_cast<uint16_t>(request->force5)
        };

        response->default_force_accepted = (modbus_write_registers(ctx_, 1044, 6, forces) == 0); // 写入所有上电初始力控值

        // 读取某个寄存器的值（小拇指上电初始力控）
        uint16_t read_value[1];
        if (modbus_read_registers(ctx_, 1044, 1, read_value) != -1) {
            RCLCPP_INFO(this->get_logger(), "Read default force0 value: %d", read_value[0]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read default force0 value: %s", modbus_strerror(errno));
        }

        // 写入寄存器 1005 以保存设置
        uint16_t save_value = 1; // 代表保存设置
        if (modbus_write_registers(ctx_, 1005, 1, &save_value) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register 1005 to save settings");
            response->default_force_accepted = false; // 保存失败
        } else {
            RCLCPP_INFO(this->get_logger(), "Settings saved successfully to Modbus register 1005");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: default force error! Default force values must be in the range of 0 to 3000.");
        response->default_force_accepted = false;
    }
}

    void setdefaultspeed_callback(const service_interfaces::srv::Setdefaultspeed::Request::SharedPtr request,
                               const service_interfaces::srv::Setdefaultspeed::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Default Speed request received");

    // 检查请求中的初始速度参数是否合法
    if ((request->speed0 >= 0 && request->speed0 <= 1000) &&
        (request->speed1 >= 0 && request->speed1 <= 1000) &&
        (request->speed2 >= 0 && request->speed2 <= 1000) &&
        (request->speed3 >= 0 && request->speed3 <= 1000) &&
        (request->speed4 >= 0 && request->speed4 <= 1000) &&
        (request->speed5 >= 0 && request->speed5 <= 1000))
    {
        // 将上电初始速度值写入 Modbus 寄存器
        uint16_t speeds[6] = {
            static_cast<uint16_t>(request->speed0), 
            static_cast<uint16_t>(request->speed1), 
            static_cast<uint16_t>(request->speed2), 
            static_cast<uint16_t>(request->speed3), 
            static_cast<uint16_t>(request->speed4), 
            static_cast<uint16_t>(request->speed5)
        };

        response->default_speed_accepted = (modbus_write_registers(ctx_, 1032, 6, speeds) == 0); // 写入所有上电初始速度值

        // 读取某个寄存器的值（小拇指上电初始速度）
        uint16_t read_value[1];
        if (modbus_read_registers(ctx_, 1032, 1, read_value) != -1) {
            RCLCPP_INFO(this->get_logger(), "Read default speed0 value: %d", read_value[0]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read default speed0 value: %s", modbus_strerror(errno));
        }

        // 写入寄存器 1005 以保存设置
        uint16_t save_value = 1; // 代表保存设置
        if (modbus_write_registers(ctx_, 1005, 1, &save_value) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register 1005 to save settings");
            response->default_speed_accepted = false; // 保存失败
        } else {
            RCLCPP_INFO(this->get_logger(), "Settings saved successfully to Modbus register 1005");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Hand: default speed error! Default speed values must be in the range of 0 to 1000.");
        response->default_speed_accepted = false;
    }
}

void setid_callback(const service_interfaces::srv::Setid::Request::SharedPtr request,
                    const service_interfaces::srv::Setid::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set ID request received");

    // 检查请求中的 ID 是否在合法范围内
    if (request->id >= 1 && request->id <= 254) {
        // 将 ID 写入 Modbus 寄存器
        uint16_t id_value = static_cast<uint16_t>(request->id);
        modbus_write_registers(ctx_, 1000, 1, &id_value); // 写入 ID

        // 读取设备的返回帧
        uint16_t read_value;
        int read_status = modbus_read_registers(ctx_, 1000, 1, &read_value); // 读取 ID

        if (read_status != -1) {
            RCLCPP_INFO(this->get_logger(), "Read ID from Modbus register: %d", read_value);
            // 判断写入的数据与读取的数据是否相同
            if (read_value == id_value) {
                RCLCPP_INFO(this->get_logger(), "ID verification successful.");
                response->idgrab = true; // 操作成功，设置 idgrab 为 true
            } else {
                RCLCPP_WARN(this->get_logger(), "ID verification failed! Expected: %d, Read: %d", id_value, read_value);
                response->idgrab = false; // 操作失败，设置 idgrab 为 false
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read ID from Modbus register: %s", modbus_strerror(errno));
            response->idgrab = false; // 读取失败，设置 idgrab 为 false
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Hand: ID error! ID must be in the range of 1 to 254.");
        response->idgrab = false; // 操作失败，设置 idgrab 为 false
    }

    // 返回成功
    return; 
}

    void setreduratio_callback(const service_interfaces::srv::Setreduratio::Request::SharedPtr request,
                            const service_interfaces::srv::Setreduratio::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Reduction Ratio request received");

    // 检查请求中的 redu_ratio 是否在合法范围内
    if (request->redu_ratio >= 0 && request->redu_ratio <= 4) {
        // 将 redu_ratio 写入 Modbus 寄存器
        uint16_t redu_ratio_value = static_cast<uint16_t>(request->redu_ratio);
        response->redu_ratiograb = (modbus_write_registers(ctx_, 1002, 1, &redu_ratio_value) == 0); // 写入 redu_ratio

        if (response->redu_ratiograb) {
            response->success = true; // 操作成功

            // 读取寄存器以验证写入的 redu_ratio
            uint16_t read_value;
            if (modbus_read_registers(ctx_, 1002, 1, &read_value) != -1) {
                RCLCPP_INFO(this->get_logger(), "Read REDU_RATIO from Modbus register: %d", read_value);
                if (read_value == request->redu_ratio) {
                    RCLCPP_INFO(this->get_logger(), "Reduction Ratio verification successful.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Reduction Ratio verification failed! Expected: %d, Read: %d", request->redu_ratio, read_value);
                    response->success = false; // 操作失败
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to read REDU_RATIO from Modbus register: %s", modbus_strerror(errno));
                response->success = false; // 读取失败
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to write REDU_RATIO to Modbus register.");
            response->success = false; // 写入失败
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Hand: REDU_RATIO error! REDU_RATIO must be in the range of 0 to 4.");
        response->success = false; // 操作失败
    }

    // 返回成功
    return;
}
  
    void setclearerror_callback(const service_interfaces::srv::Setclearerror::Request::SharedPtr request,
                                const service_interfaces::srv::Setclearerror::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set CLEAR ERROR request received");

    uint16_t tab_reg[1]; // 用于存储读取的寄存器值
    int rc = modbus_read_registers(ctx_, 1004, 1, tab_reg);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register: %s", modbus_strerror(errno));
        response->setclear_error_accepted = false; // 写入失败
    } else {
        response->setclear_error_accepted = true; // 写入成功
    }
  }
  
    void setsaveflash_callback(const service_interfaces::srv::Setsaveflash::Request::SharedPtr request,
                           const service_interfaces::srv::Setsaveflash::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Hand: Set SAVE FLASH request received");

    uint16_t value = 1; // 写入1，代表保存到闪存
    // 写入 Modbus 寄存器 1005
    if (modbus_write_registers(ctx_, 1005, 1, &value) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register 1005: %s", modbus_strerror(errno));
        response->setsave_flash_accepted = false; // 写入失败
    } else {
        RCLCPP_INFO(this->get_logger(), "Settings saved successfully to Modbus register 1005");
        response->setsave_flash_accepted = true; // 写入成功
    }
}

    void setresetpara_callback(const service_interfaces::srv::Setresetpara::Request::SharedPtr request,
                           const service_interfaces::srv::Setresetpara::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Hand: Set RESET PARAMETER request received");

    uint16_t value = 1; // 写入1，代表重置参数

    // 写入 Modbus 寄存器 1006
    if (modbus_write_registers(ctx_, 1006, 1, &value) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to Modbus register 1006: %s", modbus_strerror(errno));
        response->setreset_para_accepted = false; // 写入失败
    } else {
        RCLCPP_INFO(this->get_logger(), "Parameter reset successfully in Modbus register 1006");
        response->setreset_para_accepted = true; // 写入成功
    }
}

void setgestureno_callback(const service_interfaces::srv::Setgestureno::Request::SharedPtr request,
                           const service_interfaces::srv::Setgestureno::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Hand: Set Gesture Number request received");

    // 检查手势编号是否在有效范围内
    if (request->gesture_no >= 0 && request->gesture_no <= 13) {
        int gesture_register_address = 2320;  // 手势编号寄存器地址
        int action_register_address = 2322;    // 执行动作序列号寄存器地址

        // 将手势编号写入 Modbus 寄存器
        if (modbus_write_register(ctx_, gesture_register_address, request->gesture_no) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gesture number: %d", request->gesture_no);
            response->gesture_nograb = false; // 写入失败
            return;
        }

        // 执行动作序列号
        uint16_t action_sequence_number = 1; 
        if (modbus_write_register(ctx_, action_register_address, action_sequence_number) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute action sequence for gesture number: %d", request->gesture_no);
            response->gesture_nograb = false; // 写入失败
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully set gesture number to: %d", request->gesture_no);
        response->gesture_nograb = true; // 写入成功
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Hand: Gesture number error! Gesture number must be in the range of 0 to 13.");
        response->gesture_nograb = false; // 无效的手势编号
    }
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Hand_control>();
    
    // 把节点的执行器变成多线程执行器, 避免死锁
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

