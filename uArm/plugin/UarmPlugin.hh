/*
 * Copyright (C) 2014 UFactory Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Duke Fong <duke@evol.net>
 */

#ifndef _GAZEBO_UARM_PLUGIN_
#define _GAZEBO_UARM_PLUGIN_

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/util/system.hh>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>

namespace gazebo
{

struct axis {
	double x, y, z;
};

class GAZEBO_VISIBLE UarmPlugin : public ModelPlugin
{
	/// \brief Constructor
	public: UarmPlugin();

	/// \brief Destructor
	public: virtual ~UarmPlugin();

	/// \brief Load the controller
	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	/// \brief Update the controller
	/// \param[in] _info Update information provided by the server.
	private: void UpdateStates(const common::UpdateInfo &_info);

	private: physics::WorldPtr world;
	private: physics::ModelPtr model;
	private: physics::LinkPtr sucker;
	private: physics::JointPtr joints[3];

	private: transport::NodePtr node;
	private: transport::PublisherPtr pub;
	private: transport::SubscriberPtr sub;
	private: void sub_callback(ConstPosePtr &_msg);

	private: axis axis_origin;
	private: axis axis_dest; // set in sub_callback

	private: double angle_dest[3];

	private: double step_separation[3];
	private: double step_count, step_count_end;
	private: bool is_steping = false;
	private: bool published = true;
	private: void step(void);

	private: bool angle_mv_flag[3] = {false, false, false};

	private: double r_min = 0;
	void axes_to_angle(double x, double y, double z,
			double &angle0, double &angle1, double &angle2);

	private: boost::mutex update_mutex;

	// Pointer to the update event connection
	private: event::ConnectionPtr updateConnection;
};

}
#endif
