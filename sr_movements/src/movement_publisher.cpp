/**
 * @file   movement_publisher.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Publishes a sequence of movements.
 *
 */

#include "sr_movements/movement_publisher.hpp"

#include <math.h>
#include <sr_utilities/sr_math_utils.hpp>

namespace shadowrobot
{
  MovementPublisher::MovementPublisher(double min_value, double max_value,
                                       double rate, unsigned int repetition, unsigned int nb_mvt_step, std::string controller_type)
    : nh_tilde("~"), publishing_rate( rate ), repetition(repetition),
      min(min_value), max(max_value), last_target_(0.0), nb_mvt_step(nb_mvt_step),
      SError_(0.0), MSError_(0.0), n_samples_(0), controller_type(controller_type)
  {
    pub = nh_tilde.advertise<std_msgs::Float64>("targets", 5);
    pub_2 = nh_tilde.advertise<std_msgs::Float64>("mse_out", 5);
    if(controller_type.compare("sr")==0)
    {
      //nb_mvt_step is used to set the size of the buffer
      sub_ = nh_tilde.subscribe("inputs", nb_mvt_step, &MovementPublisher::calculateErrorCallback, this);
    }
    else if(controller_type.compare("pr2")==0)
    {
      sub_ = nh_tilde.subscribe("inputs", nb_mvt_step, &MovementPublisher::pr2_calculateErrorCallback, this);
    }
    else
    {
      ROS_WARN_STREAM("Warning: You didn't choose a msg_type to listen. sr_robot_msgs was chosen by default");
      //nb_mvt_step is used to set the size of the buffer
      sub_ = nh_tilde.subscribe("inputs", nb_mvt_step, &MovementPublisher::calculateErrorCallback, this);
    }
  }

  MovementPublisher::~MovementPublisher()
  {}

  void MovementPublisher::start()
  {
    double last_target = 0.0;

    for(unsigned int i_rep = 0; i_rep < repetition; ++i_rep)
    {
      for( unsigned int i=0; i<partial_movements.size(); ++i)
      {
        for(unsigned int j=0; j<nb_mvt_step; ++j)
        {
          if( !ros::ok() )
            return;

          //get the target
          msg.data = partial_movements[i].get_target( static_cast<double>(j) / static_cast<double>(nb_mvt_step));
          //there was not target -> resend the last target
          if( msg.data == -1.0 )
            msg.data = last_target;

          //interpolate to the correct range
          msg.data = min + msg.data * (max - min);

          //publish the message
          pub.publish( msg );

          //wait for a bit
          publishing_rate.sleep();

          ros::spinOnce();

          last_target = msg.data;
        }
      }
      //print the error information
      ROS_INFO_STREAM("MSE: " << MSError_ << " Error(deg): " << sr_math_utils::to_degrees( sqrt(MSError_) ) );

      //publish the error information
      msg.data = MSError_;
      pub_2.publish( msg );

      //Reset the error counter
      SError_ = 0.0;
      n_samples_ = 0;
    }
  }

  void MovementPublisher::calculateErrorCallback(const sr_robot_msgs::JointControllerState::ConstPtr& msg)
  {
    double error = msg->set_point - msg->process_value;
    ROS_DEBUG_STREAM("Error: " << error);
    SError_ = SError_ + ( error * error );
    ROS_DEBUG_STREAM("SError: " << SError_);
    n_samples_++;
    ROS_DEBUG_STREAM("Samples: " << n_samples_);
    MSError_ = SError_ / ( static_cast<double>(n_samples_) );
    ROS_DEBUG_STREAM("MSe: " << MSError_);
  }

  void MovementPublisher::pr2_calculateErrorCallback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
  {
    double error = msg->set_point - msg->process_value;
    ROS_DEBUG_STREAM("Error: " << error);
    SError_ = SError_ + ( error * error );
    ROS_DEBUG_STREAM("SError: " << SError_);
    n_samples_++;
    ROS_DEBUG_STREAM("Samples: " << n_samples_);
    MSError_ = SError_ / ( static_cast<double>(n_samples_) );
    ROS_DEBUG_STREAM("MSe: " << MSError_);
  }

  void MovementPublisher::execute_step(int index_mvt_step, int index_partial_movement)
  {
    if( !ros::ok() )
      return;

    //get the target
    msg.data = partial_movements[index_partial_movement].get_target( static_cast<double>(index_mvt_step) / static_cast<double>(nb_mvt_step));
    //interpolate to the correct range
    msg.data = min + msg.data * (max - min);

    //there was not target -> resend the last target
    if( msg.data == -1.0 )
      msg.data = last_target_;

    //publish the message
    pub.publish( msg );

    //wait for a bit
    publishing_rate.sleep();

    last_target_ = msg.data;
  }

  void MovementPublisher::stop()
  {}

  void MovementPublisher::add_movement(PartialMovement mvt)
  {
    partial_movements.push_back( mvt );
  }

  void MovementPublisher::set_publisher(ros::Publisher publisher)
  {
    pub = publisher;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/