/*
* @file   shadowhand_to_cyberglove_remapper.h
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu May 13 09:44:52 2010
*
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @brief This program remaps the force information contained in
* /joint_states coming from the hand to the /cybergraspforces topic
* used to control the cybergrasp.
*
*
*/

#ifndef SR_REMAPPERS_SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_
#define SR_REMAPPERS_SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_

// messages
#include <sensor_msgs/JointState.h>
#include "sr_remappers/calibration_parser.h"
#include <string>
#include <vector>

namespace shadowhand_to_cyberglove_remapper
{

/*
* This program remaps the force information contained in
* /joint_states coming from the hand to the /cybergraspforces topic
* used to control the cybergrasp.
*/
class ShadowhandToCybergloveRemapper
{
 public:
  /*
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  ShadowhandToCybergloveRemapper();
  ~ShadowhandToCybergloveRemapper() {}
 private:
  /*
   * Number of joints in the hand
   */
  static const unsigned int number_hand_joints;

  /*
   * Init the vector containing the joints names
   *
   */
  void init_names();
  /// ROS node handles
  ros::NodeHandle node, n_tilde;
  /// Vector containing all the joints names for the shadowhand.
  std::vector<std::string> joints_names;
  /// subscriber to the jointstates topic from the cyberglove
  ros::Subscriber cyberglove_jointstates_sub;
  /// publish to the shadowhand sendupdate topic
  ros::Publisher shadowhand_pub;
  /// the calibration parser containing the mapping matrix
  CalibrationParser* calibration_parser;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  /*
   * process the joint_states callback: receives the message from the cyberglove node, remap it to the Dextrous hand and
   * publish this message on a given topic
   *
   * @param msg the joint_states message
   */
  void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg);

  /*
   * process the joint_states callback for the finger abductions: processes the message from the cyberglove node, remap it to the Dextrous hand J4s
   * It overwrites whatever was written for the J4s by the calibration parser get_remapped_vector
   *
   * @param msg the joint_states message
   * @param vect the vector where the result is written (only J4s are written)
   */
  void getAbductionJoints(const sensor_msgs::JointStateConstPtr& msg, std::vector<double>& vect);
};  // end class

}  // namespace shadowhand_to_cyberglove_remapper

#endif  // SR_REMAPPERS_SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_
