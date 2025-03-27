/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_walking_planner/walking_planner.h>

namespace ow_planner
{
  WalkingPlanner::WalkingPlanner() : fs_planner_(),
                                     dcm_planner_(),
                                     foot_size_(ow::Vector3::Zero()),
                                     foot_offset_(ow::Vector3::Zero()),
                                     is_walking_(false),
                                     is_abort_(false),
                                     cur_foot_(ow::FootId::LEFT)
  {
    // hold the current left and right foot position that is updated
    // after every foot landing (end of single support)
    cur_steps_.resize(2);
  }

  WalkingPlanner::~WalkingPlanner()
  {
  }

  bool WalkingPlanner::initalize(ros::NodeHandle &nh)
  {
    // load the ow parameter
    parameter_.add<ow::Scalar>("publish_rate");
    parameter_.add<ow::Scalar>("loop_rate");
    parameter_.add<ow::Scalar>("hip_height");
    parameter_.add<ow::Scalar>("step_time");
    parameter_.add<ow::Scalar>("t_double_support");
    parameter_.add<ow::Scalar>("t_single_support");
    
    parameter_.add<size_t>("walking_planner/n_steps");
    parameter_.add<ow::Scalar>("walking_planner/length");
    parameter_.add<ow::Scalar>("walking_planner/angle");

    parameter_.add<ow::Vector3>("walking_planner/step_vis/foot_size");
    parameter_.add<ow::Vector3>("walking_planner/step_vis/foot_offset");
    parameter_.add<ow::Scalar>("ready_pose/feet_separation");

    if (!parameter_.load("/open_walker"))
    {
      ROS_ERROR("WalkingPlanner::init: Error loading parameters");
      return false;
    }

    // Initialize internal planners.
    if (!fs_planner_.initRequest(parameter_, nh))
    {
      ROS_ERROR("%s::init: Error init module", fs_planner_.name().c_str());
      return false;
    }

    if (!dcm_planner_.initRequest(parameter_, nh))
    {
      ROS_ERROR("%s::init: Error init module", fs_planner_.name().c_str());
      return false;
    }

    parameter_.get("walking_planner/n_steps", n_steps_);
    parameter_.get("walking_planner/length", length_);
    parameter_.get("walking_planner/angle", angle_);
    parameter_.get("walking_planner/step_vis/foot_size", foot_size_);
    parameter_.get("walking_planner/step_vis/foot_offset", foot_offset_);
    parameter_.get("ready_pose/feet_separation", feet_separation_);

    // Create publishers, subscribers and servers.

    // publish
    steps_vis_pub_ = nh.advertise<jsk_footstep_msgs::FootstepArray>("footsteps_vis", 10);

    plan_pub_ = nh.advertise<ow_msgs::WalkingPlan>("walking_plan", 10);

    dcm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dcm_vis", 10);

    // subscriber
    step_feedback_sub_ = nh.subscribe("step_feedback", 100, &WalkingPlanner::stepFeedbackCallback, this);
    
    // services
    
    // stops the current plan
    abort_srv_ = nh.advertiseService("abort", &WalkingPlanner::abortSrv, this);

    // general planner call
    plan_srv_ = nh.advertiseService("plan_fixed", &WalkingPlanner::planSrvClient, this);
    
    // stamping
    stamp_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_stamp", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, 0.0, 0.0, 0.0));

    // line
    line_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_line", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, length_, 0.0, 0.0));

    // circular motion
    circle_left_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_circle_left", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, length_, 0.0, angle_));
    circle_right_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_circle_right", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, length_, 0.0, -angle_));

    // rotation in place
    rotate_left_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_rotate_left", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, 0.0, 0.025, angle_));
    rotate_right_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_rotate_right", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, 0.0, -0.025, -angle_));
    
    // side stepping
    sidestep_left_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_sidestep_left", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, 0.0, 0.3 * feet_separation_, 0.0));
    sidestep_right_srv_ = 
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("plan_fixed_sidestep_right", 
        boost::bind(&WalkingPlanner::planSrvClient, this, _1, _2, 0.0, -0.3 * feet_separation_, 0.0));

    // Prepare standing footsteps.
    ow::CartesianPosition X_l_w = ow::CartesianPosition::Identity();
    ow::CartesianPosition X_r_w = ow::CartesianPosition::Identity();
    X_l_w.position() << 0.0, 0.5 * feet_separation_, 0.0;
    X_r_w.position() << 0.0, -0.5 * feet_separation_, 0.0;

    // initalize the footstep planner
    fs_plan_ = fs_planner_.start(X_l_w, X_r_w);
    cur_steps_ = fs_planner_.footSteps();

    // intialize the dcm planner
    dcm_planner_.start(X_l_w, X_r_w);
    dcm_plan_ = dcm_planner_.plan(fs_plan_, cur_steps_[cur_foot_]);

    // For VRP
    vrp_msg_.color.r = 1.0;
    vrp_msg_.color.g = 0.0;
    vrp_msg_.color.b = 0.0;
    vrp_msg_.color.a = 1.0;
    vrp_msg_.scale = ow::Vector3(0.01, 0.01, 0.01);
    vrp_msg_.pose.orientation = ow::AngularPosition::Zero();
    vrp_msg_.type = vrp_msg_.SPHERE;
    vrp_msg_.header.frame_id = "world";

    // For DCM points
    dcm_msg_.color.r = 0.2;
    dcm_msg_.color.g = 0.2;
    dcm_msg_.color.b = 1.0;
    dcm_msg_.color.a = 1.0;
    dcm_msg_.scale = ow::Vector3(0.01, 0.01, 0.01);
    dcm_msg_.pose.orientation = ow::AngularPosition::Zero();
    dcm_msg_.type = dcm_msg_.SPHERE;
    dcm_msg_.header.frame_id = "world";

    return true;
  }

  bool WalkingPlanner::planSrvClient( std_srvs::EmptyRequest &req,
                std_srvs::EmptyResponse &resp,
                ow::Scalar lenght,
                ow::Scalar lateral,
                ow::Scalar angle)
  {
    // reset plans
    reset();

    if(lateral < 0 || angle < 0)
    {
      // first step should be right foot
      cur_foot_ = ow::FootId::RIGHT;
    }
    else
    {
      // first step should be left foot
      cur_foot_ = ow::FootId::LEFT;
    }

    // create plans and publish
    fs_plan_ = fs_planner_.generateFixedPlan(lenght,
                                              lateral,
                                              angle,
                                              n_steps_);
    dcm_plan_ = dcm_planner_.plan(fs_plan_, cur_steps_[cur_foot_]);
    publishPlans();

    //printPlans();
    is_walking_ = true;
    return true;
  }

  bool WalkingPlanner::planSrvClient(ow_msgs::OWPlanStepsRequest &req,
                               ow_msgs::OWPlanStepsResponse &resp)
  {
    // reset plans
    reset();

    // create plans and publish
    fs_plan_ = fs_planner_.generateFixedPlan(req.length.data,
                                                req.lateral.data,
                                                req.angle.data,
                                                req.n_steps.data);
    dcm_plan_ = dcm_planner_.plan(fs_plan_, cur_steps_[cur_foot_]);
    publishPlans();

    //printPlans();
    is_walking_ = true;
    resp.success.data = true;
    return resp.success.data;
  }

  bool WalkingPlanner::abortSrv( std_srvs::EmptyRequest &req,
              std_srvs::EmptyResponse &resp )
  {
    if(is_walking_)
    {
      ROS_ERROR_STREAM("abort!");
      is_abort_ = true;
    }
    return true;
  }

  void WalkingPlanner::publishVisualization()
  {
    jsk_footstep_msgs::FootstepArray msg;
    visualization_msgs::MarkerArray dcm_vis_msg;

    // empty plan
    if (fs_plan_.empty() && !is_walking_)
    {
      return;
    }

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.footsteps.resize(fs_plan_.size());
    dcm_vis_msg.markers.resize(dcm_plan_.size() * 2);

    // Prepare visualization messages.
    for (unsigned int i = 0; i < fs_plan_.size(); i++)
    {
      // Publish Footsteps. (Allwas on the ground)
      msg.footsteps[i].pose = fs_plan_[i].pos();
      msg.footsteps[i].pose.position.z = 0.0;
      msg.footsteps[i].leg = fs_plan_[i].footId() == ow::FootId::LEFT ? 1 : 2;
      msg.footsteps[i].dimensions = foot_size_;
      msg.footsteps[i].offset = foot_offset_;
      msg.footsteps[i].footstep_group = fs_plan_[i].nStep();
      if (fs_plan_[i].finalStep())
      {
        msg.footsteps[i].leg = msg.footsteps[i].LARM;
      }

      // Publish VRP
      vrp_msg_.id = 2 * i;
      vrp_msg_.pose.position = dcm_plan_[i].vrp();
      dcm_vis_msg.markers[2 * i] = vrp_msg_;

      // Publish DCM
      dcm_msg_.id = 2 * i + 1;
      vrp_msg_.pose.position = dcm_plan_[i].dcm();
      dcm_vis_msg.markers[2 * i + 1] = dcm_msg_;
    }

    // Publish all.
    steps_vis_pub_.publish(msg);
    dcm_vis_pub_.publish(dcm_vis_msg);
  }

  void WalkingPlanner::stepFeedbackCallback(const ow_msgs::FootStep &msg)
  {
    // update current foot steps with the feedback
    cur_foot_ = ow::FootId(static_cast<ow::FootId::Value>(msg.foot_id.data));

    cur_steps_[cur_foot_] = msg;

    // Update footstep plan.
    fs_planner_.update(cur_steps_[cur_foot_]);
    fs_plan_ = fs_planner_.footSteps();

    // Update DCM plan.
    dcm_plan_ = dcm_planner_.plan(fs_plan_, cur_steps_[cur_foot_]);

    // Publish plan
    publishPlans();

    // Print plan some feedback
    ROS_INFO_STREAM("************************************");
    ROS_INFO_STREAM("Updated plan for n_step=" << cur_steps_[cur_foot_].nStep());
    ROS_INFO_STREAM("************************************");

    if(cur_steps_[cur_foot_].finalStep())
    {
      // feedback for last step recived, we are done
      ROS_WARN_STREAM("************************************");
      ROS_WARN_STREAM("Done");
      ROS_WARN_STREAM("************************************");

      // reset step hight back to ground plane
      cur_steps_[cur_foot_].pos().z() = 0.0;
      cur_steps_[cur_foot_.other()].pos().z() = 0.0;

      // clear old plans
      dcm_plan_.clear();
      fs_plan_.clear();
      is_walking_ = false;
    }

    // we need to abort, stop the planner
    if(is_abort_)
    {
      fs_planner_.abort(cur_steps_[cur_foot_]);
      fs_plan_ = fs_planner_.footSteps();
      dcm_plan_ = dcm_planner_.plan(fs_plan_, cur_steps_[cur_foot_]);
      is_abort_ = false;
    }
  }

  void WalkingPlanner::publishPlans()
  {
    ow_msgs::WalkingPlan plan_msg;

    // to plan msg
    plan_msg.header.stamp = ros::Time::now();

    plan_msg.dcm_plan.resize(dcm_plan_.size());
    plan_msg.footstep_plan.resize(fs_plan_.size());
    for (unsigned int i = 0; i < dcm_plan_.size(); i++)
    {
      plan_msg.dcm_plan[i] = dcm_plan_[i];
      plan_msg.footstep_plan[i] = fs_plan_[i];
    }

    plan_pub_.publish(plan_msg);
  }

  void WalkingPlanner::reset()
  { 
    // reset cur_foot, default LEFT
    cur_foot_ = ow::FootId::LEFT;

    // create empty plans
    cur_steps_ = fs_planner_.reset(cur_steps_);
    dcm_planner_.reset(cur_steps_);
  }

  void WalkingPlanner::printPlans()
  {
    ROS_WARN_STREAM("************************************");
    fs_planner_.print();
    ROS_WARN_STREAM("************************************");
    dcm_planner_.print();
  }

} // namespace ow_planner
