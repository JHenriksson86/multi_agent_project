#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>
#include <sstream>

#include "ros/ros.h"
#include "multi_agent_messages/Communication.h"

namespace robot
{

    struct Messages
    {
        static multi_agent_messages::Communication getMessageHeader(
             std::string sender)
        {
            multi_agent_messages::Communication msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "World";
            msg.sender = sender;
            msg.receiver = "";
            msg.message_type = "";
            msg.message = "";
            return msg;
        }

        static multi_agent_messages::Communication getReadyMessage(
            std::string sender)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = "/all_cluster_robots";
            msg.message_type = "ready";
            return msg;
        }

        static multi_agent_messages::Communication getStartMessage(
            std::string sender)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = "/all_cluster_robots";
            msg.message_type = "start";
            return msg;
        }

        static multi_agent_messages::Communication getConfirmMessage(
            std::string sender, std::string receiver)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = receiver;
            msg.message_type = "confirm";
            return msg;
        }

        static multi_agent_messages::Communication getAuctionMessage(
            std::string sender, double x, double y)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = "/all_cluster_robots";
            msg.message_type = "auction";
            msg.message = std::to_string(x) + "," + std::to_string(y);
            return msg;
        }

        static multi_agent_messages::Communication getResultMessage(
            std::string sender, std::string winner)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = "/all_cluster_robots";
            msg.message_type = "result";
            msg.message = winner;
            return msg;
        }

        static multi_agent_messages::Communication getBidMessage(
            std::string sender, std::string receiver, double bid)
        {
            multi_agent_messages::Communication msg = getMessageHeader(sender);

            msg.receiver = receiver;
            msg.message_type = "bid";
            msg.message = std::to_string(bid);
            return msg;
        }

        static std::string messageToString(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            std::string str = "Sender: " + msg->sender + "\n";
            str += "Receiver: " + msg->receiver + "\n";
            str += "Message type: " + msg->message_type + "\n";
            str += "Message: " + msg->message + "\n";
            return str;
        }
    };

    class MessageHandler
    {
        protected:
        
        std::string     robot_ns_;
        ros::Time       start_time_;
        bool            timer_started_;
        int             message_counter_;
        
        public:

        MessageHandler(std::string robot_namespace)
        {
            this->robot_ns_ = robot_namespace;
            this->timer_started_ = false;
            this->message_counter_ = 0;
        }

        virtual void addMessage(const multi_agent_messages::Communication::ConstPtr &msg) = 0;

        virtual multi_agent_messages::Communication getMessage() = 0;

        virtual bool run() = 0; 

        protected:

        void startTimer()
        {
            start_time_ = ros::Time::now();
            timer_started_ = true;
        }

        double getTimerSeconds()
        {
            return ros::Time::now().toSec() - start_time_.toSec();
        }

        void stopTimer(){ timer_started_ = false; }
    };

    class ScoutMessageHandler : public MessageHandler
    {
        private:

        enum class ScoutMessageState
        {
            ready, start, running, auction, bid, result
        };

        ScoutMessageState state_;
        int number_of_cluster_robots_;
        double best_bid_;
        std::string best_bidder_;

        public:

        ScoutMessageHandler(std::string robot_namespace, int number_of_cluster_robots)
        : MessageHandler(robot_namespace)
        {
            this->state_ = ScoutMessageState::ready;
            this->number_of_cluster_robots_ = number_of_cluster_robots;
            this->best_bid_ = -1.0;
            this->best_bidder_ = "";
        }

        void addMessage(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            if(msg->receiver.compare(robot_ns_) == 0)
            {
                ROS_INFO("%s:\n%s", robot_ns_.c_str(), Messages::messageToString(msg).c_str());
                switch(state_)
                {
                    case ScoutMessageState::ready:
                        break;
                    case ScoutMessageState::start:
                        if(msg->message_type.compare("confirm") == 0)
                        {
                            message_counter_++;
                        }
                        break;
                    case ScoutMessageState::running:
                        break;
                    case ScoutMessageState::auction:
                        if(msg->message_type.compare("auction") == 0)
                        {
                            double bid = std::stod(msg->message);
                            if(best_bid_ < 0.0 || bid < best_bid_)
                            {
                                best_bid_ = bid;
                                best_bidder_ = msg->sender;
                            }
                            message_counter_++;
                        }
                }
                
            }
        }

        multi_agent_messages::Communication getMessage()
        {
            switch(state_)
            {
                case ScoutMessageState::start:
                    return Messages::getReadyMessage(robot_ns_);
                    break;
                case ScoutMessageState::running:
                    return Messages::getStartMessage(robot_ns_);
                    break;
                case ScoutMessageState::auction:
                    return Messages::getResultMessage(robot_ns_, best_bidder_);
                    break;
            }
        }

        multi_agent_messages::Communication startAuction(double x, double y)
        {
            gotoAuction();
            return Messages::getAuctionMessage(robot_ns_, x, y);
        }

        bool run()
        {
            double time = 0.0;
            switch(state_)
            {
                case ScoutMessageState::ready:
                    if(!timer_started_)
                    {
                        startTimer();
                    }
                    time = getTimerSeconds();
                    if(time > 5.0)
                    {
                        gotoStart();
                        return true;
                    }
                    return false;
                    break;
                case ScoutMessageState::start:
                    if(!timer_started_)
                    {
                        startTimer();
                    }
                    time = getTimerSeconds();
                    if(message_counter_ == number_of_cluster_robots_)
                    {
                        gotoRunning();
                        return true;
                    }
                    if(time > 10.0)
                    {
                        gotoStart();
                        return true;
                    }
                    return false;
                    break;
                case ScoutMessageState::running:
                    return false;
                    break;
                case ScoutMessageState::auction:
                    if(!timer_started_)
                    {
                        startTimer();
                    }
                    time = getTimerSeconds();
                    if(message_counter_ == number_of_cluster_robots_ || time > 5.0)
                    {
                        gotoResult();
                        return true;
                    }
                    return false;
                    break;
                case ScoutMessageState::result:
                    if(!timer_started_)
                    {
                        startTimer();
                    }
                    time = getTimerSeconds();
                    if(message_counter_ == number_of_cluster_robots_)
                    {
                        gotoRunning();
                        return true;
                    }
                    if(time > 5.0)
                    {
                        gotoRunning();
                        return true;
                    }
                    return false;
                    break;
            
            }
        }

        private:

        void gotoStart()
        {
            stopTimer();
            state_ = ScoutMessageState::start;
            message_counter_ = 0;
        }

        void gotoRunning()
        {
            stopTimer();
            state_ = ScoutMessageState::running;
            message_counter_ = 0;
        }

        void gotoAuction()
        {
            state_ = ScoutMessageState::auction;
            message_counter_ = 0;
            best_bid_ = -1.0;
            best_bidder_ = "";
        }

        void gotoResult()
        {
            stopTimer();
            state_ = ScoutMessageState::result;
            message_counter_ = 0;
        }
    };

    class ClusterMessageHandler : public MessageHandler
    {
        private:

        enum class ClusterMessageState
        {
            ready, start, running, auction
        };

        ClusterMessageState state_;
        Navigation* navigation_;
        bool message_waiting_;
        double object_x_;
        double object_y_;

        public:

        ClusterMessageHandler(std::string robot_namespace, Navigation* navigation)
        : MessageHandler(robot_namespace)
        {
            this->state_ = ClusterMessageState::ready;
            this->message_waiting_ = false;
            this->navigation_ = navigation;
            this->object_x_ = 0.0;
            this->object_y_ = 0.0;
        }

        void addMessage(const multi_agent_messages::Communication::ConstPtr &msg)
        {
            if(msg->receiver.compare("/all_cluster_robots") == 0 || 
                msg->receiver.compare(robot_ns_) == 0)
            {
                ROS_INFO("%s:\n%s", robot_ns_.c_str(), Messages::messageToString(msg).c_str());
                switch(state_)
                {
                    case ClusterMessageState::ready:
                        if(msg->message_type.compare("ready") == 0)
                        {
                            message_waiting_ = true;
                        }
                        break;
                    case ClusterMessageState::start:
                        if(msg->message_type.compare("start") == 0)
                        {
                            message_waiting_ = true;
                        }
                        break;
                    case ClusterMessageState::running:
                        if(msg->message_type.compare("auction") == 0)
                        {
                            std::stringstream ss;
                            ss.str(msg->message);
                            std::string x = "", y = "";
                            std::getline(ss, x, ',');
                            std::getline(ss, y);
                            object_x_ = std::stod(x);
                            object_y_ = std::stod(y);

                            message_waiting_ = true;
                        }
                        break;
                    case ClusterMessageState::auction:
                        if(msg->message_type.compare("result") == 0)
                        {
                            if(msg->message.compare(robot_ns_) == 0)
                                ROS_INFO("%s: I'm winner!", robot_ns_.c_str());
                            message_waiting_ = true;
                        }
                        break;
                }
            }
        }

        multi_agent_messages::Communication getMessage()
        {
            double bid = 0.0;
            switch(state_)
            {
                case ClusterMessageState::ready:
                    state_ = ClusterMessageState::start;
                    message_waiting_ = false;
                    return Messages::getConfirmMessage(robot_ns_, "/scout");
                    break;
                case ClusterMessageState::start:
                    state_ = ClusterMessageState::running;
                    message_waiting_ = false;
                    return Messages::getConfirmMessage(robot_ns_, "/scout");
                    break;
                case ClusterMessageState::running:
                    state_ = ClusterMessageState::auction;
                    message_waiting_ = false;
                    bid = navigation_->getDistanceToCoordinate(object_x_, object_y_);
                    return Messages::getBidMessage(robot_ns_, "/scout", bid);
                    break;
                case ClusterMessageState::auction:
                    state_ = ClusterMessageState::running;
                    message_waiting_ = false;
                    return Messages::getConfirmMessage(robot_ns_, "/scout");
                    break;
            }
        }

        bool run()
        {
            return message_waiting_;
        }
    };
    
}

#endif