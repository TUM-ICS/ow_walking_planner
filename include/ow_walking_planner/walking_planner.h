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

#ifndef OPEN_WALKER_WALKING_PLANNER_H_
#define OPEN_WALKER_WALKING_PLANNER_H_

#include <ow_core/types.h>
#include <ow_core/types/foot_step.h>

#include <ow_fs_planner/footstep_planner.h>
#include <ow_dcm_planner/dcm_planner.h>

#include <ow_msgs/WalkingPlan.h>
#include <ow_msgs/OWPlanSteps.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <jsk_footstep_msgs/FootstepArray.h>
#include <jsk_footstep_msgs/Footstep.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

/*!
 * \brief Open Walker walking planner module namespace. These classes wrap the
 * footstep planner module and the dcm planner module. They also provide
 * interfaces to the user to command the walking.
 */
namespace ow_planner
{

/*!
 * \brief The WalkingPlanner class
 *
 * This class implements the WalkingPlanner module of the
 * openwalker framework.
 *
 * It combines the ow_fs_planner module and ow_dcm_planner and provides
 * services for path generation/replanning.
 */
class WalkingPlanner
{
protected:
  ow::Parameter parameter_;               //!< Configuration of this module.
  bool is_walking_;                       //!< active during execution
  bool is_abort_;                         //!< stop walking

  ow_fs_planner::FootstepPlanner  fs_planner_;  //!< Footstep planner.
  ow_dcm_planner::DCMPlanner      dcm_planner_; //!< DCM planner.

  std::vector<ow::FootStep> fs_plan_;     //!< Footsteps plan.
  ow::DCMPointSetList  dcm_plan_;         //!< DCM plan.

  ros::Publisher steps_vis_pub_;  //!< Step visualization publisher.
  ros::Publisher dcm_vis_pub_;    //!< DCM visualizytion publisher.
  ros::Publisher plan_pub_;       //!< walking plan publisher

  ros::Subscriber step_feedback_sub_;   //!< Landed foot position feebdakc.
  ros::Subscriber vel_cmd_sub_;         //!< Joystic vel command listener.

  ros::ServiceServer stamp_srv_;          //!< Stamp footstep server.
  ros::ServiceServer line_srv_;           //!< Line footstep server.
  ros::ServiceServer circle_left_srv_;    //!< Circle footstep server.
  ros::ServiceServer circle_right_srv_;   //!< Circle footstep server.
  ros::ServiceServer rotate_left_srv_;    //!< Rotate on spot
  ros::ServiceServer rotate_right_srv_;   //!< Rotate on spot
  ros::ServiceServer sidestep_left_srv_;  //!< Sidestepping
  ros::ServiceServer sidestep_right_srv_; //!< Sidestepping
  ros::ServiceServer plan_srv_;           //!< Plan footstep server.

  ros::ServiceServer  abort_srv_;         //!< stop the current plan at next posiblility

  ow::FootStepList cur_steps_;            //!< The last left and right step
  ow::FootId cur_foot_;                   //!< The current foot step

  geometry_msgs::Twist        vel_msg_;   //!<  Received vel. command msg.
  visualization_msgs::Marker  vrp_msg_;   //!<  DCM vis message placeholder.
  visualization_msgs::Marker  dcm_msg_;   //!<  Open Walker DCM msg placeholder.

  size_t n_steps_;            //!< Number of steps to plan.
  ow::Scalar length_;         //!< Forward step length for fixed plans.
  ow::Scalar angle_;          //!< Step angle for fixed plans.

  ow::Vector3 foot_size_;     //!< Robot foot size for Rviz visualization.
  ow::Vector3 foot_offset_; //!< Robot foot offset to sole frame for Rviz.

  ow::Scalar feet_separation_;  //!< Feet separation for module initalization.

public:
  /*!
  * \brief WalkingPlanner Default constructor.
  * 
  */
  WalkingPlanner();

  // destructor
  virtual ~WalkingPlanner();

  /*!
   * \brief Initialization of WalkingPlanner
   */
  bool initalize(ros::NodeHandle& nh);

  /*!
   * \brief publishSteps
   *    Publish last plan with no update.
   */
  void publishVisualization();

  /**
   * @brief reset the planner
   * 
   */
  void reset();

private:

  void publishPlans();

  bool planSrvClient( ow_msgs::OWPlanStepsRequest &req,
                ow_msgs::OWPlanStepsResponse &resp );

  bool planSrvClient( std_srvs::EmptyRequest &req,
                std_srvs::EmptyResponse &resp,
                ow::Scalar lenght,
                ow::Scalar lateral,
                ow::Scalar angle);

  bool abortSrv( std_srvs::EmptyRequest &req,
                std_srvs::EmptyResponse &resp );

private:

  /*!
   * \brief receivedCurrentFootstep
   *    Landed foot position from ow_ftg module.
   *
   * \param msg
   *    Received footstep msg.
   */
  void stepFeedbackCallback(const ow_msgs::FootStep &msg);

  /**
   * @brief print the current plans
   * 
   */
  void printPlans();

};

}

#endif
