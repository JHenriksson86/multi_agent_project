#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>
#include <sstream>
#include <queue>
#include <cmath>

#include "ros/ros.h"
#include "multi_agent_messages/Communication.h"

namespace robot
{
    enum class MessageType
    {
        Auction,
        Start,
        Cluster,
        Unknown
    };

    struct Message
    {
        static multi_agent_messages::Communication emptyMessage()
        {
            multi_agent_messages::Communication msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "World";
            msg.sender = "";
            msg.receiver = "";
            msg.message_type = "";
            msg.message = "";
            return msg;
        }

        static multi_agent_messages::Communication message(
            const std::string& sender, 
            const std::string& receiver, 
            const MessageType& msg_type, 
            const std::string& message
        )
        {
            multi_agent_messages::Communication msg = emptyMessage();

            msg.sender = sender;
            msg.receiver = receiver;
            msg.message_type = messageTypeString(msg_type);
            msg.message = message;
            return msg;
        }

        static std::string messageTypeString(
            const MessageType& type
        )
        {
            switch(type)
            {
                case MessageType::Auction:
                    return "auction";
                case MessageType::Start:
                    return "start";
                case MessageType::Cluster:
                    return "cluster";
            }
            return "";
        }

        static MessageType getMessageType(
            const multi_agent_messages::Communication &msg
        )
        {
            if(msg.message_type.compare("auction") == 0)
                return MessageType::Auction;
            if(msg.message_type.compare("start") == 0)
                return MessageType::Start;
            if(msg.message_type.compare("cluster") == 0)
                return MessageType::Cluster;
            return MessageType::Unknown;
        }

        static std::string messageToString(
            const multi_agent_messages::Communication &msg
        )
        {
            std::string str = "Sender: " + msg.sender + "\n";
            str += "Receiver: " + msg.receiver + "\n";
            str += "Message type: " + msg.message_type + "\n";
            str += "Message: " + msg.message + "\n";
            return str;
        }
    };

    class MessageHandler
    {
        protected:

        enum class StartProcedureState
        {
            Idle, Ready, Start, Waiting
        };

        enum class AuctionState
        {
            Idle, Auction, Bidding, Result, Waiting
        };

        enum class ClusterState
        {
            Idle, Received, Waiting
        };

        StartProcedureState start_state_;
        AuctionState auction_state_;
        ClusterState cluster_state_;
        
        std::queue<multi_agent_messages::Communication> msg_in_;
        std::queue<multi_agent_messages::Communication> msg_out_;

        std::string     robot_ns_;
        ros::Time       start_time_;
        bool            timer_started_;
        int             confirm_counter_;
        
        public:

        MessageHandler(std::string robot_namespace)
        {
            this->robot_ns_ = robot_namespace;
            this->timer_started_ = false;
            this->confirm_counter_ = 0;
            this->start_state_ = StartProcedureState::Idle;
            this->auction_state_ = AuctionState::Idle;
            this->cluster_state_ = ClusterState::Idle;
        }

        virtual void addMessage(const multi_agent_messages::Communication::Ptr &msg) = 0;

        multi_agent_messages::Communication getMessage()
        {
            multi_agent_messages::Communication msg = msg_out_.front();
            msg_out_.pop();
            return msg;
        }

        bool messageOutEmpty()
        {
            return msg_out_.empty();
        }

        virtual bool run() = 0; 

        protected:

        void startTimer()
        {
            if(!timer_started_)
            {
                start_time_ = ros::Time::now();
                timer_started_ = true;
            }
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
        int number_of_cluster_robots_;
        double best_bid_;
        std::string best_bidder_;
        double object_x_;
        double object_y_;

        public:

        ScoutMessageHandler(std::string robot_namespace, int number_of_cluster_robots)
        : MessageHandler(robot_namespace)
        {
            this->number_of_cluster_robots_ = number_of_cluster_robots;
            this->best_bid_ = -1.0;
            this->best_bidder_ = "";
            this->object_x_ = 0.0;
            this->object_y_ = 0.0;
        }

        bool initStartProcedure()
        {
            if(start_state_ != StartProcedureState::Idle)
            {
                ROS_WARN("%s: Start procedure is already running.", robot_ns_.c_str());
                return false;
            }
            startProcedureReady();
            return true;
        }

        bool startProcedureFinished() const { return start_state_ == StartProcedureState::Idle; }

        bool initAuction(double x, double y)
        {
            if(auction_state_ != AuctionState::Idle)
            {
                ROS_WARN("%s: Cannot start new auction, auction is already running.", robot_ns_.c_str());
                return false;
            }
            auctionStateAuction(x, y);
            return true;
        }

        bool auctionFinished() const { return auction_state_ == AuctionState::Idle; }

        void addMessage(const multi_agent_messages::Communication::Ptr &msg)
        {
            if(msg->receiver.compare(robot_ns_) == 0)
            {
                ROS_DEBUG("%s:\n%s", robot_ns_.c_str(), Message::messageToString(*msg).c_str());
                msg_in_.push(*msg);
            }
        }

        bool run()
        {
            bool new_messages = !msg_in_.empty();
            if(new_messages)
            {
                readMessages();
            }

            runAuction();
            runStartProcedure();
        }

        private:

        void readMessages()
        {
            while (!msg_in_.empty())
            {
                multi_agent_messages::Communication msg = msg_in_.front();
                msg_in_.pop();
                switch(Message::getMessageType(msg))
                {
                    case MessageType::Auction:
                        ROS_INFO("%s: Auction message received.", robot_ns_.c_str());
                        readAuctionMessage(msg);
                        break;
                    case MessageType::Start:
                        ROS_INFO("%s: Start message received.", robot_ns_.c_str());
                        readStartMessage(msg);
                        break;
                    case MessageType::Cluster:
                        ROS_INFO("%s: Cluster message received.", robot_ns_.c_str());
                        readClusterMessage(msg);
                        break;
                    case MessageType::Unknown:
                        ROS_WARN("%s: Message type unknown", robot_ns_.c_str());
                        break;
                }
            }
        }

        void readAuctionMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            switch(auction_state_)
            {
                case AuctionState::Bidding:
                    if(action.compare("bid") == 0)
                    {
                        std::string bid = "";
                        std::getline(ss, bid);
                        double new_bid = std::stod(bid);
                        confirm_counter_++;
                        if(best_bid_ < 0 || best_bid_ > new_bid)
                        {
                            best_bid_ = new_bid;
                            best_bidder_ = msg.sender;
                        }
                    }
                    break;
                case AuctionState::Waiting:
                    if(action.compare("confirm") == 0)
                    {
                        confirm_counter_++;
                    }
                    break;
                default:
                    ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
                    break;
            }
        }

        void readStartMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            switch(start_state_)
            {
                case StartProcedureState::Waiting:
                    if(action.compare("confirm") == 0)
                    {
                        confirm_counter_++;
                    }
                    break;
                default:
                    ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
                    break;
            }
        }

        void readClusterMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            if(action.compare("finished") == 0)
            {
                std::string x = "", y = "";
                std::getline(ss, x, ' ');
                std::getline(ss, y);
                ROS_INFO("%s: Finished cluster message x = %s, y = %s.", robot_ns_.c_str(), x.c_str(), y.c_str());
                msg_out_.push(Message::message(robot_ns_, msg.sender, MessageType::Cluster, "confirm"));
            }
            else
            {
                ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
            }
        }

        void runAuction()
        {
            switch(auction_state_)
            {
                case AuctionState::Idle:
                    break;
                case AuctionState::Auction:
                    msg_out_.push(
                        Message::message(
                            robot_ns_, 
                            "/all_cluster_robots", 
                            MessageType::Auction, 
                            "auction " + std::to_string(object_x_) + " " + std::to_string(object_y_)
                        )
                    );
                    auctionStateBidding();
                    break;
                case AuctionState::Bidding:
                    startTimer();
                    if(confirm_counter_ == number_of_cluster_robots_ || getTimerSeconds() > 5.0)
                    {
                        auctionStateResult();
                    }
                    break;
                case AuctionState::Result:
                    msg_out_.push(
                        Message::message(
                            robot_ns_, 
                            "/all_cluster_robots", 
                            MessageType::Auction, 
                            "result " + best_bidder_
                        )
                    );
                    auctionStateWaiting();
                    break;
                case AuctionState::Waiting:
                    startTimer();
                    if(getTimerSeconds() > 5.0)
                    {
                        auctionStateResult();
                    }
                    if(confirm_counter_ == number_of_cluster_robots_)
                    {
                        auctionStateIdle();
                    }
                    break;
            }
        }

        void auctionStateIdle()
        {
            ROS_INFO("%s: AuctionState::Idle", robot_ns_.c_str());
            stopTimer();
            auction_state_ = AuctionState::Idle;
        }

        void auctionStateAuction(double x, double y)
        {
            ROS_INFO("%s: AuctionState::Auction, object x = %.2f y = %.2f.", robot_ns_.c_str(), x, y);
            this->object_x_ = x;
            this->object_y_ = y;
            auction_state_ = AuctionState::Auction;
        }

        void auctionStateBidding()
        {
            ROS_INFO("%s: AuctionState::Bidding", robot_ns_.c_str());
            confirm_counter_ = 0;
            best_bid_ = -1;
            best_bidder_ = "";
            auction_state_ = AuctionState::Bidding;
        }

        void auctionStateResult()
        {
            ROS_INFO("%s: AuctionState::Result", robot_ns_.c_str());
            stopTimer();
            auction_state_ = AuctionState::Result;
        }

        void auctionStateWaiting()
        {
            ROS_INFO("%s: AuctionState::Waiting", robot_ns_.c_str());
            confirm_counter_ = 0;
            auction_state_ = AuctionState::Waiting;
        }

        void runStartProcedure()
        {
            switch(start_state_)
            {
                case StartProcedureState::Idle:
                    break;
                case StartProcedureState::Ready:
                    msg_out_.push(
                        Message::message(
                            robot_ns_, 
                            "/all_cluster_robots", 
                            MessageType::Start, 
                            "ready"
                        )
                    );
                    startProcedureWaiting();
                    break;
                case StartProcedureState::Waiting:
                    startTimer();
                    if(getTimerSeconds() > 5.0)
                    {
                        stopTimer();
                        startProcedureReady();
                    }
                    if(confirm_counter_ == number_of_cluster_robots_)
                    {
                        stopTimer();
                        startProcedureStart();
                    }
                    break;
                case StartProcedureState::Start:
                    msg_out_.push(
                        Message::message(
                            robot_ns_, 
                            "/all_cluster_robots", 
                            MessageType::Start, 
                            "start"
                        )
                    );
                    startProcedureIdle();
                    break;
            }
        }

        void startProcedureIdle()
        {
            ROS_INFO("%s: StartProcedureState::Idle", robot_ns_.c_str());
            start_state_ = StartProcedureState::Idle;
        }

        void startProcedureReady()
        {
            ROS_INFO("%s: StartProcedureState::Ready", robot_ns_.c_str());
            confirm_counter_ = 0;
            start_state_ = StartProcedureState::Ready;
        }

        void startProcedureWaiting()
        {
            ROS_INFO("%s: StartProcedureState::Waiting", robot_ns_.c_str());
            confirm_counter_ = 0;
            start_state_ = StartProcedureState::Waiting;
        }

        void startProcedureStart()
        {
            ROS_INFO("%s: StartProcedureState::Start", robot_ns_.c_str());
            confirm_counter_ = 0;
            start_state_ = StartProcedureState::Start;
        }
    };

    class ClusterMessageHandler : public MessageHandler
    {
        private:
        double robot_x_;
        double robot_y_;
        int robot_queue_size_;

        double object_x_;
        double object_y_;

        public:

        ClusterMessageHandler(std::string robot_namespace)
        : MessageHandler(robot_namespace)
        {
            this->robot_x_ = 0.0;
            this->robot_y_ = 0.0;
            this->robot_queue_size_ = 0;

            this->object_x_ = 0.0;
            this->object_y_ = 0.0;
        }

        bool objectClustered(double x, double y)
        {
            if(cluster_state_ != ClusterState::Idle)
            {
                ROS_WARN("%s: Cannot send clustered message.", robot_ns_.c_str());
                return false;
            }
            msg_out_.push(
                Message::message(
                    robot_ns_, 
                    "/scout", 
                    MessageType::Cluster, 
                    "finished " + std::to_string(x) + " " + std::to_string(y)
                )
            );
            clusterStateWaiting();
            return true;
        }

        void addMessage(const multi_agent_messages::Communication::Ptr &msg)
        {
            if(msg->receiver.compare(robot_ns_) == 0 || msg->receiver.compare("/all_cluster_robots") == 0)
            {
                ROS_DEBUG("%s:\n%s", robot_ns_.c_str(), Message::messageToString(*msg).c_str());
                msg_in_.push(*msg);
            }
        }

        bool run()
        {
            bool new_messages = !msg_in_.empty();
            if(new_messages)
            {
                readMessages();
            }

            runAuction();
            runCluster();
        }

        private:
        
        void readMessages()
        {
            while (!msg_in_.empty())
            {
                multi_agent_messages::Communication msg = msg_in_.front();
                msg_in_.pop();
                switch(Message::getMessageType(msg))
                {
                    case MessageType::Auction:
                        ROS_INFO("%s: Auction message received.", robot_ns_.c_str());
                        readAuctionMessage(msg);
                        break;
                    case MessageType::Start:
                        ROS_INFO("%s: Start message received.", robot_ns_.c_str());
                        readStartMessage(msg);
                        break;
                    case MessageType::Cluster:
                        ROS_INFO("%s: Cluster message received.", robot_ns_.c_str());
                        readClusterMessage(msg);
                        break;
                    case MessageType::Unknown:
                        ROS_WARN("%s: Message type unknown", robot_ns_.c_str());
                        break;
                }
            }
        }

        void readAuctionMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            switch(auction_state_)
            {
                case AuctionState::Idle:
                    if(action.compare("auction") == 0)
                    {
                        std::string str_x = "", str_y = "";
                        std::getline(ss, str_x, ' ');
                        std::getline(ss, str_y);
                        object_x_ = std::stod(str_x);
                        object_y_ = std::stod(str_y);
                        auctionStateBidding();
                    }
                    break;
                case AuctionState::Result:
                    if(action.compare("result") == 0)
                    {
                        std::string winner = "";
                        std::getline(ss, winner);
                        if(winner.compare(robot_ns_) == 0)
                        {
                            ROS_INFO("%s: I won the auction!.", robot_ns_.c_str());
                        }
                        else
                        {
                            ROS_INFO("%s: I lost the auction!.", robot_ns_.c_str());
                        }
                        msg_out_.push(
                            Message::message(
                                robot_ns_, 
                                "/scout", 
                                MessageType::Auction, 
                                "confirm"
                            )
                        );
                        auctionStateIdle();
                    }
                    break;
                default:
                    ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
                    break;
            }
        }

        void readStartMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            switch(start_state_)
            {
                case StartProcedureState::Idle:
                    if(action.compare("ready") == 0)
                    {
                        msg_out_.push(
                            Message::message(
                                robot_ns_, 
                                "/scout", 
                                MessageType::Start, 
                                "confirm"
                            )
                        );
                    }
                    break;
                default:
                    ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
                    break;
            }
        }

        void readClusterMessage(
            const multi_agent_messages::Communication &msg
        )
        {
            std::stringstream ss;
            ss.str(msg.message);
            std::string action = "";
            std::getline(ss, action, ' ');

            if(action.compare("confirm") == 0 || cluster_state_ == ClusterState::Waiting)
            {
                clusterStateIdle();
            }
            else
            {
                ROS_WARN("%s: Message not expected.\n%s", robot_ns_.c_str(), Message::messageToString(msg).c_str());
            }
        }

        void runAuction()
        {
            switch(auction_state_)
            {
                case AuctionState::Idle:
                    break;
                case AuctionState::Auction:
                    break;
                case AuctionState::Bidding:
                    msg_out_.push(
                        Message::message(
                            robot_ns_, 
                            "/scout", 
                            MessageType::Auction, 
                            "bid " + std::to_string(makeBid())
                        )
                    );
                    auctionStateResult();
                    break;
                case AuctionState::Result:
                    break;
                case AuctionState::Waiting:
                    break;
            }
        }

        void runCluster()
        {
            switch(start_state_)
            {
                case StartProcedureState::Idle:
                    break;
                case StartProcedureState::Ready:
                    break;
                case StartProcedureState::Waiting:
                    // TODO
                    break;
                case StartProcedureState::Start:
                    break;
            }
        }

        double makeBid()
        {
            return std::sqrt(std::pow(object_x_, 2) + std::pow(object_y_, 2)) + (double)robot_queue_size_ * 10.0;
        }

        void auctionStateIdle()
        {
            ROS_INFO("%s: AuctionState::Idle", robot_ns_.c_str());
            auction_state_ = AuctionState::Idle;
        }

        void auctionStateBidding()
        {
            ROS_INFO("%s: AuctionState::Bidding", robot_ns_.c_str());
            auction_state_ = AuctionState::Bidding;
        }

        void auctionStateResult()
        {
            ROS_INFO("%s: AuctionState::Result", robot_ns_.c_str());
            auction_state_ = AuctionState::Result;
        }

        void clusterStateIdle()
        {
            ROS_INFO("%s: ClusterState::Idle", robot_ns_.c_str());
            cluster_state_ = ClusterState::Idle;
        }

        void clusterStateWaiting()
        {
            ROS_INFO("%s: ClusterState::Waiting", robot_ns_.c_str());
            cluster_state_ = ClusterState::Waiting;
        }
    };
    
}

#endif