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

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>
#include <sstream>

static bool need_exit = false;

static void sub_callback(ConstPosePtr &_msg)
{
	// TODO: check if the pose is same as published

	std::cout << "callback: stop at: x: " << _msg->position().x()
					  << ", y: " << _msg->position().y()
					  << ", z: " << _msg->position().z() << std::endl;
	need_exit = true;
}

int main(int _argc, char **_argv)
{
	if (_argc != 4) {
		std::cout << "Usage: " << _argv[0] << " x y z\n";
		exit(1);
	}

	std::istringstream ss_x(_argv[1]);
	std::istringstream ss_y(_argv[2]);
	std::istringstream ss_z(_argv[3]);
	double x, y, z;
	if (!((ss_x >> x) && (ss_y >> y) && (ss_z >> z))) {
		std::cout << "Usage: " << _argv[0] << " x y z\n";
		exit(1);
	}

	std::cout << "publish x: " << x << ", y: " << y << ", z: " << z
			<< " to ~/uArm/sucker_pose_set\n";

	// Load gazebo
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	gazebo::transport::SubscriberPtr sub =
			node->Subscribe("~/uArm/sucker_pose_end", sub_callback);

	// Publish to a Gazebo topic
	gazebo::transport::PublisherPtr pub =
			node->Advertise<gazebo::msgs::Pose>("~/uArm/sucker_pose_set");

	// Wait for a subscriber to connect
	//pub->WaitForConnection();
	gazebo::common::Time::MSleep(100);

	// Generate a pose
	gazebo::math::Pose pose(x, y, z, 0, 0, 0);

	// Convert to a pose message
	gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, pose);

	pub->Publish(msg);
	std::cout << "publish done, waiting for callback...\n";

	while (!need_exit)
		gazebo::common::Time::MSleep(100);

	// Make sure to shut everything down.
	gazebo::client::shutdown();
}
