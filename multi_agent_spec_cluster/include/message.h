#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>

#include "ros/ros.h"
#include "multi_agent_messages/Communication.h"

namespace robot
{
    enum class RobotType
    {
        scout, cluster
    };

    struct Messages
    {
        static multi_agent_messages::Communication getConfirmMessage(
            std::string robot_namespace)
        {
            multi_agent_messages::Communication msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "World";

            msg.sender = robot_namespace;
            msg.receiver = "/scout";
            msg.message_type = "confirm";
            msg.message = "confirm";
            return msg;
        }
    };

    class MessageHandler
    {
        private:
        std::string robot_ns_;
        RobotType   robot_type_;
        
        public:

        MessageHandler(std::string robot_namespace, RobotType robot_type)
        {
            this->robot_ns_ = robot_namespace;
            this->robot_type_ = robot_type;
        }

        void addMessage(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            if(robot_type_ == RobotType::cluster)
            {
                if(msg->receiver.compare("/all_cluster_robots") == 0 || 
                    msg->receiver.compare(robot_ns_) == 0)
                    {
                        handleClusterMessage(msg);
                    }
            }
            else if(robot_type_ == RobotType::scout)
            {
                if(msg->receiver.compare(robot_ns_) == 0)
                {
                    handleScoutMessage(msg);
                }
            }
        
        }

        void run()
        {

        }

        private:

        void handleScoutMessage(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            ROS_INFO(
               "%s:Received message\nfrom: %s\nto: %s\ntype: %s\nmessage: %s",
               robot_ns_.c_str(), msg->sender.c_str(), msg->receiver.c_str(), 
               msg->message_type.c_str(), msg->message.c_str()
            );
        }

        void handleClusterMessage(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            ROS_INFO(
               "%s: Received message\nfrom: %s\nto: %s\ntype: %s\nmessage: %s",
               robot_ns_.c_str(), msg->sender.c_str(), msg->receiver.c_str(), 
               msg->message_type.c_str(), msg->message.c_str()
            );

            if(msg->message_type.compare("ready") == 0)
            {
                
            }
        }
    };
}

#endif