#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <picar_msgs/CarCmd.h>
#include <picar_msgs/Pose2DStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <thread>

#define DEFAULT_MAX_STEERING_ANGLE_DEG  25.0

namespace gazebo
{
class PicarPlugin : public ModelPlugin
{
public:
    
    ros::Publisher wheelSpeedPub;
    ros::Publisher posePub;
    
    PicarPlugin() : ModelPlugin()
    {
        this->CarCmdMsg.angle = 10.0*M_PI/180.0;
        this->CarCmdMsg.velocity = 0.0;
        timeSinceLastMsg = 0.0;
    }
    
    virtual void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
    {
        // get model
        this->model = _model;
        // get world in which the model is spawned
        this->world = _model->GetWorld();
        
        this->lastSimTime = this->world->SimTime();
        
        // let gazebo handle the ros initialization
        this->gazeboNode = transport::NodePtr(new transport::Node());
        this->gazeboNode->Init(this->world->Name());
        
        if(!ros::isInitialized())
        {
            ROS_FATAL_STREAM("ROS node for Gazebo not initialized.");
            return;
        }
        
        // create ros node handle
        this->nodeHandle = new ros::NodeHandle("");
        ROS_INFO("created ros node handle");
        this->wheelSpeedPub = this->nodeHandle->advertise<geometry_msgs::Quaternion>("simcar/wheel_speed", 1);
        this->posePub = this->nodeHandle->advertise<picar_msgs::Pose2DStamped>("simcar/pose", 1);
        
        
        ros::WallDuration(1.0).sleep();
        
        // set the callback queue
        this->nodeHandle->setCallbackQueue(&this->jointCmdCallbackQueue);
        ROS_INFO("created callback queue");
        this->jointCmdOptions = ros::SubscribeOptions::create<picar_msgs::CarCmd>("simcar/motor_node/car_cmd",
                                                                                  1, 
                                                                                  boost::bind(&PicarPlugin::CarCmdCb,
                                                                                              this,
                                                                                              _1), 
                                                                                  ros::VoidPtr(), 
                                                                                  &this->jointCmdCallbackQueue);
        ROS_INFO("created subscribe options");
        this->jointStateSub = nodeHandle->subscribe(this->jointCmdOptions);
        ROS_INFO("subscribed to joint commands");
        
        
        this->queueThread = std::thread(std::bind(&PicarPlugin::queueHandler, this));
        
        if(!this->connectJoints(_sdf))
            return;
        
        gzmsg << "joints connected";
        
        if(!this->setPIDgains(_sdf))
            return;
        
        this->calculateCarGeometry();
        
        this->linkChassis = _model->GetLink("chassis");
        if(!this->linkChassis)
            return;
        
        
        
        
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PicarPlugin::OnUpdate, this)
        );
        
        ROS_INFO_STREAM("Successfully initialized PicarPlugin!");
        
    }
    
    void OnUpdate()
    {
        std::lock_guard<std::mutex> lock(this->data_mutex);
        
        common::Time currentTime = this->world->SimTime();
        double dt = (currentTime - this->lastSimTime).Double();
        this->lastSimTime = currentTime;
        
        if(dt < 0)
        {
            //TODO Reset the plugin!
            return;
        }
        else if(ignition::math::equal(dt, 0.0))
            return;
        
        // get current state of wheels
        this->updateJointAngles();
        this->updateJointVelocities();
        
        this->chassisVelocityVec = this->linkChassis->WorldCoGLinearVel();
        double chassisVel = this->chassisVelocityVec.Length();
        
        
        double tanSteer = tan(CarCmdMsg.angle);
        this->steeringFLcmd = atan2(tanSteer,
                                    1 - this->trackWidthFront/2/this->wheelBase*tanSteer);
        
        this->steeringFRcmd = atan2(tanSteer,
                                    1 + this->trackWidthFront/2/this->wheelBase*tanSteer);
        
        double steeringFLerror, steeringFRerror;
        steeringFLerror = this->angleFL - this->steeringFLcmd;
        steeringFRerror = this->angleFR - this->steeringFRcmd;
        
        //gzerr << "Desired: " << jointCmdMsg.angle << "\nActual: " << this->angleFL << ", " << this->angleFR;
        
        double leftForce, rightForce;
        leftForce = this->pidFLsteering.Update(steeringFLerror, dt);
        rightForce = this->pidFRsteering.Update(steeringFRerror, dt);
        // gzmsg << "Left/Right: " << leftForce << "/" << rightForce << "\n";
        this->jointFLsteering->SetForce(0, leftForce);
        this->jointFRsteering->SetForce(0, rightForce);
        
        
        double speedRRcmd, speedRLcmd;
        double speedRRerror, speedRLerror;
        speedRRcmd = this->msgVel2wheelVel(this->CarCmdMsg.velocity) * (1.0+trackWidthRear/2.0/wheelBase*tanSteer);
        speedRLcmd = this->msgVel2wheelVel(this->CarCmdMsg.velocity) * (1.0-trackWidthRear/2.0/wheelBase*tanSteer);
        
        speedRRerror = this->velRR - speedRRcmd;
        speedRLerror = this->velRL - speedRLcmd;
        
        // gzmsg << this->velRR << ", " << speedRRcmd << "\n";
        
        // gzmsg << "Left/Right: " << leftForce << "/" << rightForce << "\n";
        
        leftForce = this->pidRLspeed.Update(speedRLerror, dt);
        rightForce = this->pidRRspeed.Update(speedRRerror, dt);
        
        this->jointRLwheel->SetForce(0, leftForce);
        this->jointRRwheel->SetForce(0, rightForce);
        
        timeSinceLastMsg += dt;
        if(timeSinceLastMsg > 0.01)
        {
            picar_msgs::Pose2DStamped poseMsg;
            ignition::math::Pose3d pose = this->model->GetLink("chassis")->WorldPose();
            poseMsg.header.stamp = ros::Time::now();
            poseMsg.x = pose.Pos()[0];
            poseMsg.y = pose.Pos()[1];
            poseMsg.theta = pose.Rot().Euler()[2];
            this->posePub.publish(poseMsg);
            
            geometry_msgs::Quaternion wheelSpeedMsg;
            timeSinceLastMsg = 0.0;
            wheelSpeedMsg.w = this->velFL;
            wheelSpeedMsg.x = this->velFR;
            wheelSpeedMsg.y = this->velRL;
            wheelSpeedMsg.z = this->velRR;
            this->wheelSpeedPub.publish(wheelSpeedMsg);
        }
        
        
        
        
        
    }
    
    
    void CarCmdCb(const boost::shared_ptr<picar_msgs::CarCmd const> msg)
    {
        // avoid race condition between gazebo update callback and ros message callback
        std::lock_guard<std::mutex> lock(this->data_mutex);
        
        // update angle and speed set point
        this->CarCmdMsg.angle = ignition::math::clamp(this->deg2rad(msg->angle), 
                                                        this->deg2rad(-DEFAULT_MAX_STEERING_ANGLE_DEG), 
                                                        this->deg2rad(DEFAULT_MAX_STEERING_ANGLE_DEG)); 
        this->CarCmdMsg.velocity = ignition::math::clamp(double(msg->velocity), -1.0, 1.0);
    }
    
    
    
private:
    physics::WorldPtr world;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    common::PID pidFLsteering, pidFRsteering;
    common::PID pidRLspeed, pidRRspeed;
    transport::NodePtr gazeboNode;
    
    common::Time lastSimTime;
    
    physics::JointPtr jointFLwheel, jointFRwheel, jointRLwheel, jointRRwheel, jointFLsteering, jointFRsteering;
    physics::LinkPtr linkChassis;
    ignition::math::Vector3d chassisVelocityVec;
    
    double velFL, velFR, velRL, velRR;
    double angleFL, angleFR;
    double steeringFLcmd, steeringFRcmd;
    double trackWidthFront;
    double trackWidthRear;
    double wheelBase;
    double wheelRadius;
    double maxVelocity;
    double timeSinceLastMsg;
    
    ros::NodeHandle *nodeHandle;
    ros::Subscriber jointStateSub;
    ros::SubscribeOptions jointCmdOptions;
    ros::CallbackQueue jointCmdCallbackQueue;
    ros::Publisher carSpeedPub;
    
    
    
    std::thread queueThread;
    std::mutex data_mutex;
    
    picar_msgs::CarCmd CarCmdMsg;
    
    bool connectJoints(sdf::ElementPtr _sdf)
    {
        this->jointFLsteering = this->model->GetJoint(_sdf->Get<std::string>("front_left_steering_joint"));
        if (!this->jointFLsteering)
        {
            gzerr << "Unable to find joint: front_left_steering_joint";
            return false;
        }
        
        this->jointFRsteering = this->model->GetJoint(_sdf->Get<std::string>("front_right_steering_joint"));
        if(!this->jointFRsteering)
        {
            gzerr << "Unable to find joint: front_right_steering_joint";
            return false;
        }
        
        this->jointFLwheel = this->model->GetJoint(_sdf->Get<std::string>("front_left_wheel_joint"));
        if(!this->jointFLwheel)
        {
            gzerr << "Unable to find joint: front_left_wheel_joint";
            return false;
        }
        
        this->jointFRwheel = this->model->GetJoint(_sdf->Get<std::string>("front_right_wheel_joint"));
        if(!this->jointFRwheel)
        {
            gzerr << "Unable to find joint: front_right_wheel_joint";
            return false;
        }
        
        this->jointRLwheel = this->model->GetJoint(_sdf->Get<std::string>("rear_left_wheel_joint"));
        if(!this->jointRLwheel)
        {
            gzerr << "Unable to find joint: rear_left";
            return false;
        }
        
        this->jointRRwheel = this->model->GetJoint(_sdf->Get<std::string>("rear_right_wheel_joint"));
        if(!this->jointRRwheel)
        {
            gzerr << "Unable to find joint: rear_right_wheel_joint";
            return false;
        }
        
        return true;
    }
    
    
    bool setPIDgains(sdf::ElementPtr _sdf)
    {
        std::string paramName;
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // steering FL
        paramName = "front_left_steering_p_gain";
        if(_sdf->HasElement(paramName))
            this->pidFLsteering.SetPGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "front_left_steering_i_gain";
        if(_sdf->HasElement(paramName))
            this->pidFLsteering.SetIGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "front_left_steering_d_gain";
        if(_sdf->HasElement(paramName))
            this->pidFLsteering.SetDGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // steering FR
        paramName = "front_right_steering_p_gain";
        if(_sdf->HasElement(paramName))
            this->pidFRsteering.SetPGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "front_right_steering_i_gain";
        if(_sdf->HasElement(paramName))
            this->pidFRsteering.SetIGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "front_right_steering_d_gain";
        if(_sdf->HasElement(paramName))
            this->pidFRsteering.SetDGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // wheel RL
        paramName = "rear_left_wheel_p_gain";
        if(_sdf->HasElement(paramName))
            this->pidRLspeed.SetPGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "rear_left_wheel_i_gain";
        if(_sdf->HasElement(paramName))
            this->pidRLspeed.SetIGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "rear_left_wheel_d_gain";
        if(_sdf->HasElement(paramName))
            this->pidRLspeed.SetDGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // wheel RR
        paramName = "rear_right_wheel_p_gain";
        if(_sdf->HasElement(paramName))
            this->pidRRspeed.SetPGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "rear_right_wheel_i_gain";
        if(_sdf->HasElement(paramName))
            this->pidRRspeed.SetIGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "rear_right_wheel_d_gain";
        if(_sdf->HasElement(paramName))
            this->pidRRspeed.SetDGain(_sdf->Get<double>(paramName));
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "wheel_torque_limit";
        if(_sdf->HasElement(paramName))
        {
            this->pidRRspeed.SetCmdMax(_sdf->Get<double>(paramName));
            this->pidRRspeed.SetCmdMin(-_sdf->Get<double>(paramName));
            this->pidRLspeed.SetCmdMax(_sdf->Get<double>(paramName));
            this->pidRLspeed.SetCmdMin(-_sdf->Get<double>(paramName));
            
        }
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "wheel_radius";
        if(_sdf->HasElement(paramName))
            this->wheelRadius = _sdf->Get<double>(paramName);
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        paramName = "max_velocity";
        if(_sdf->HasElement(paramName))
            this->maxVelocity = _sdf->Get<double>(paramName);
        else
        {
            gzerr << "Could not find param: " << paramName;
            return false;
        }
        
        gzmsg << "Set PID Gains:\nleft: " << this->pidFLsteering.GetPGain() << ", " << this->pidFLsteering.GetIGain() << ", " << this->pidFLsteering.GetDGain() << "\n\n";
        
        return true;
    }
    
    void queueHandler()
    {
        ros::CallbackQueue::CallOneResult result;
        static const double timeout = 1.0;
        while(1)
        {
            result = this->jointCmdCallbackQueue.callOne(ros::WallDuration(timeout));
        }
        
    }
    
    void updateJointVelocities()
    {
        this->velFL = this->jointFLwheel->GetVelocity(0);
        this->velFR = this->jointFRwheel->GetVelocity(0);
        this->velRL = this->jointRLwheel->GetVelocity(0);
        this->velRR = this->jointRRwheel->GetVelocity(0);
    }
    
    void updateJointAngles()
    {
        this->angleFL = this->jointFLsteering->Position();
        this->angleFR = this->jointFRsteering->Position();
    }
    
    void calculateCarGeometry()
    {
        ignition::math::Vector3d flCenter, frCenter, rlCenter, rrCenter, distVector;
        
        // get wheel positions
        unsigned int id = 0;
        flCenter = this->jointFLwheel->GetChild()->GetCollision(id)->WorldPose().Pos();
        frCenter = this->jointFRwheel->GetChild()->GetCollision(id)->WorldPose().Pos();
        rlCenter = this->jointRLwheel->GetChild()->GetCollision(id)->WorldPose().Pos();
        rrCenter = this->jointRRwheel->GetChild()->GetCollision(id)->WorldPose().Pos();
    
        distVector = flCenter - frCenter;
        this->trackWidthFront = distVector.Length();
        
        distVector = rlCenter - rrCenter;
        this->trackWidthRear = distVector.Length();
        
        distVector = flCenter - rlCenter;
        this->wheelBase = distVector.Length();
        
        gzmsg << "Wheel Base: " << this->wheelBase;
        gzmsg << "FrontTrackWidth: " << this->trackWidthFront;
        gzmsg << "RearTrackWidth " << this->trackWidthRear;
        
    }
    
    double msgVel2wheelVel(double msgVel)
    {
        double res = msgVel*this->maxVelocity/this->wheelRadius;
        return res;
    }
    
    double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

};
GZ_REGISTER_MODEL_PLUGIN( PicarPlugin )
}
