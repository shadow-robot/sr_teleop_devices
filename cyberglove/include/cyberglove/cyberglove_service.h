/*
* @file   cyberglove_service.h
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu Apr 22 10:25:55 2010
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
* @brief   A service which can stop / start the Cyberglove publisher.
*
*
*/


#ifndef CYBERGLOVE_CYBERGLOVE_SERVICE_H_
#define CYBERGLOVE_CYBERGLOVE_SERVICE_H_

#include <ros/ros.h>
#include <vector>
#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/Start.h"
#include "cyberglove/Calibration.h"
#include <boost/smart_ptr.hpp>


namespace cyberglove
{

class CybergloveService
{
public:
  explicit CybergloveService(boost::shared_ptr<CyberglovePublisher> publish);
  ~CybergloveService() {}

  bool start(cyberglove::Start::Request &req, cyberglove::Start::Response &res);
  bool calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res);
private:
  ros::NodeHandle node;
  boost::shared_ptr<CyberglovePublisher> pub;
  ros::ServiceServer service_start;
  ros::ServiceServer service_calibration;
};

}  // namespace cyberglove
#endif  // CYBERGLOVE_CYBERGLOVE_SERVICE_H_
