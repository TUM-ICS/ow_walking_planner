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

// ros
#include <ros/ros.h>
#include <ros/param.h>

// ow
#include <ow_walking_planner/walking_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"walking_planner",
            ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ow::Scalar rate = 10;

  if(nh.hasParam("walking_planner/update_rate"))
    nh.getParam("walking_planner/update_rate", rate);
  else
    ROS_ERROR_STREAM("No walking_planner/update_rate parameter. Set default "
                     << rate);

  ros::Rate r(rate);

  // create a new planner
  ow_planner::WalkingPlanner planner;

  // initalize planner
  if(!planner.initalize(nh))
  {
    ROS_ERROR("main_walking_planner: error initalize WalkingPlanner");
    return -1;
  }

  // run planner node
  while( ros::ok() )
  {
    planner.publishVisualization();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
