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

#include "UarmPlugin.hh"

using namespace gazebo;

UarmPlugin::UarmPlugin()
{
	gzdbg << "UarmPlugin Constructor\n";
}

UarmPlugin::~UarmPlugin()
{
	gzdbg << "UarmPlugin Destructor\n";
	event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

static const double len_b = 0.0215302, len_s = 0.03455;
static const double high_b = 0.0169154 + 0.0895098, high_s = 0.0565664;
static const double len_post = 0.148, len_fore = 0.16;
static const double angle_correct[3] = {
		0,
		(57.79/180.0) * M_PIl,
		((58.4-57.79)/180.0) * M_PIl
};
const double max_step_len = 0.0001;


// forward kinematic
static void angle_to_axes(double angle0, double angle1, double angle2,
		double &x, double &y, double &z)
{
	double x_offset = -1 * (len_b + len_s);
	double z_offset = high_b - high_s;
	double x_post = cos(angle1) * len_post;
	double z_post = sin(angle1) * len_post;
	double x_fore = -1 * cos(angle2) * len_fore;
	double z_fore = -1 * sin(angle2) * len_fore;

	double x_2d = -1 * (x_offset + x_post + x_fore);
	z = z_offset + z_post + z_fore;
	x = cos(angle0) * x_2d;
	y = sin(angle0) * x_2d;
	//std::cout << "angle_to_axes: x: " << x << ", y: " << y
	//		<< ", z: " << z << std::endl;
}

// inverse kinematic
void UarmPlugin::axes_to_angle(double x, double y, double z,
		double &angle0, double &angle1, double &angle2)
{
	double joint0_angle = 0;
	double joint1_angle = 0;
	double joint2_angle = 0;
	double len_a, delta_high, delta_width;
	//printf("update_angle_dest: x: %f, y: %f, z: %f\n", x, y, z);

	joint0_angle = atan2(y, x);
	//printf("joint0_angle: %f\n", joint0_angle);

	delta_high = high_b - z - high_s;
	delta_width = sqrt(pow(x, 2) + pow(y, 2));
	if (delta_width < r_min)
		gzthrow("sqrt(x^2 + y^2) must >= r_min !");
	delta_width = delta_width - len_b - len_s;

	len_a = sqrt(pow(delta_width, 2) + pow(delta_high, 2));
	double angle_a = atan2(delta_high, delta_width);

	//gzmsg << "delta_high: " << delta_high << std::endl;
	//gzmsg << "delta_width: " << delta_width << std::endl;
	//gzmsg << "len_a: " << len_a << ", angle_a: " << angle_a << ", degree: " << angle_a*180/M_PI << std::endl;

	if (len_post + len_fore <= len_a)
		gzthrow("len_post + len_fore <= len_a!");

	joint2_angle = acos((pow(len_fore, 2) + pow(len_a, 2) - pow(len_post, 2)) /
			(2 * len_fore * len_a)) + angle_a;
	joint1_angle = M_PIl - acos((pow(len_post, 2) + pow(len_a, 2) - pow(len_fore, 2)) /
			(2 * len_post * len_a)) + angle_a;

	//gzmsg << "joint2_angle(r): " << joint2_angle << ", degree: " << joint2_angle*180/M_PI << std::endl;
	//gzmsg << "joint1_angle(r): " << joint1_angle << ", degree: " << joint1_angle*180/M_PI << std::endl;

	if (fabs(joint0_angle) > M_PIl * 0.5)
		gzthrow("fabs(joint0_angle) must <= PI/2 !");
	if (joint2_angle < 0)
		gzthrow("joint2_angle must >= 0 !");
	if (joint1_angle - joint2_angle < 0.501934)
		gzthrow("joint1_angle - joint2_angle must >= 0.501934(28.64) !");
	if (joint1_angle - joint2_angle > 2.8222257)
		gzthrow("joint1_angle - joint2_angle must <= 2.8222257(161.7) !");
	if (joint1_angle < 0.897182)
		gzthrow("joint1_angle must >= 0.897182(51.4048) !");
	if (joint1_angle > 3.23562)
		gzthrow("joint1_angle must <= 3.23562(185.387) !");
	if (joint2_angle > 2.00293)
		gzthrow("joint2_angle must <= 2.00293(114.76) !");

	angle0 = joint0_angle;
	angle1 = joint1_angle;
	angle2 = joint2_angle;
}

void UarmPlugin::step(void)
{
	step_count += 1.0; // speed

	if (step_count >= step_count_end) {
		gzmsg << "step done.\n";
		is_steping = false;
		return;
	}

	double cur_x = axis_origin.x + step_separation[0] * step_count;
	double cur_y = axis_origin.y + step_separation[1] * step_count;
	double cur_z = axis_origin.z + step_separation[2] * step_count;

	double delta_r2_y2 = pow(r_min + 0.0001, 2) - pow(cur_y, 2);
	if (delta_r2_y2 > 0) {
		double delta_sqrt = sqrt(delta_r2_y2);
		if (delta_sqrt > cur_x) {
			//gzwarn << "short than r_min, force x form "
			//		<< cur_x << " to " << delta_sqrt << std::endl;
			cur_x = delta_sqrt;
		}
	}

	try {
		axes_to_angle(cur_x, cur_y, cur_z, angle_dest[0], angle_dest[1], angle_dest[2]);
		angle_mv_flag[0] = true;
		angle_mv_flag[1] = true;
		angle_mv_flag[2] = true;
	} catch (common::Exception &e) {
		gzwarn << "destination cannot reach" << std::endl;
		is_steping = false;
		return;
	}

}

// Function is called everytime a message is received.
void UarmPlugin::sub_callback(ConstPosePtr &_msg)
{
	//std::cout << _msg->DebugString();

	boost::mutex::scoped_lock lock(this->update_mutex);
	bool is_paused = this->world->IsPaused();
	if (!is_paused)
		this->world->SetPaused(true);

	axis_dest.x = _msg->position().x();
	axis_dest.y = _msg->position().y();
	axis_dest.z = _msg->position().z();

	gzmsg << "set axis_dest: x: " << axis_dest.x << ", y: " << axis_dest.y
			<< ", z: " << axis_dest.z << std::endl;

	angle_to_axes(joints[0]->GetAngle(0).Radian(),
			joints[1]->GetAngle(0).Radian() + angle_correct[1],
			joints[2]->GetAngle(0).Radian() + angle_correct[2],
			axis_origin.x, axis_origin.y, axis_origin.z);

	/*
	try {
		double angle0, angle1, angle2;
		axes_to_angle(axis_dest.x, axis_dest.y, axis_dest.z,
					  angle0, angle1, angle2);
	} catch (common::Exception &e) {
		gzerr << "destination cannot reach, skip.\n";
		return;
	}
	 */

	double delta_x = axis_dest.x - axis_origin.x;
	double delta_y = axis_dest.y - axis_origin.y;
	double delta_z = axis_dest.z - axis_origin.z;
	double delta_max = std::max(std::max(fabs(delta_x), fabs(delta_y)), fabs(delta_z));
	double delta_len = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));

	if (delta_max == 0 || delta_len < max_step_len) {
		gzwarn << "axis_dest same as axis_cur\n";
		this->world->SetPaused(is_paused);
		return;
	}

	step_separation[0] = (delta_x/delta_len) * max_step_len;
	step_separation[1] = (delta_y/delta_len) * max_step_len;
	step_separation[2] = (delta_z/delta_len) * max_step_len;
	step_count = 0;
	step_count_end = delta_len / max_step_len;
	is_steping = true;

	// resume original pause-state
	this->world->SetPaused(is_paused);
}

void UarmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	gzdbg << "UarmPlugin Load\n";

	// Get the world name.
	this->model = _parent;
	this->world = this->model->GetWorld();

	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->model->GetWorld()->GetName());
	pub = node->Advertise<gazebo::msgs::Pose>(std::string("~/") +
			this->model->GetName() + "/sucker_pose_end");
	sub = node->Subscribe(std::string("~/") +
			this->model->GetName() + "/sucker_pose_set", &UarmPlugin::sub_callback, this);

	this->sucker = _parent->GetLink("sucker_link");

	this->joints[0] = _parent->GetJoint("base_to_body");
	this->joints[1] = _parent->GetJoint("body_to_postarm");
	this->joints[2] = _parent->GetJoint("body_to_postarm_aux2");

	for (int i; i < 3; i++) {
		if (!this->joints[i]) {
			gzthrow("Invalid SDF: joint" << i << " does not exist!");
		}
		this->joints[i]->SetVelocityLimit(0, 1.2);
		// lock joints
		this->joints[i]->SetHighStop(0, joints[i]->GetAngle(0).Radian());
		this->joints[i]->SetLowStop(0, joints[i]->GetAngle(0).Radian());
	}

	// calculate r_min
	double y, z;
	angle_to_axes(0, 3.23562, 2.00293, r_min, y, z);
	gzdbg << "r_min: " << r_min << std::endl;

	// New Mechanism for Updating every World Cycle
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&UarmPlugin::UpdateStates, this, _1));

	gzdbg << "UarmPlugin Load return\n";
}

void UarmPlugin::UpdateStates(const common::UpdateInfo & /*_info*/)
{
	common::Time cur_time = this->world->GetSimTime();

	bool is_paused = this->world->IsPaused();
	if (!is_paused)
		this->world->SetPaused(true);

	boost::mutex::scoped_lock lock(this->update_mutex);
	if (is_steping) {
		step();
		published = false;
	}

	for (int i = 0; i < 3; i++) {
		if (this->angle_mv_flag[i]) {
			double angle_dest = this->angle_dest[i] - angle_correct[i];
			if (fabs(this->joints[i]->GetAngle(0).Radian() - angle_dest) <= 0.001) {
				//gzmsg << "cur" << i << ": " << this->joints[i]->GetAngle(0).Radian() << ", to: "
				//		<< this->angle_dest[i] << ", end.\n";
				this->joints[i]->SetForce(0, 0);
				this->joints[i]->SetHighStop(0, angle_dest);
				this->joints[i]->SetLowStop(0, angle_dest);
				this->angle_mv_flag[i] = false;

			} else if (this->joints[i]->GetAngle(0).Radian() > angle_dest) {
				this->joints[i]->SetForce(0, -100);
				this->joints[i]->SetLowStop(0, angle_dest);
				//gzmsg << "cur" << i << ": " << this->joints[i]->GetAngle(0).Radian() << ", to: "
				//		<< this->angle_dest[i] << ", --\n";
			} else {
				this->joints[i]->SetForce(0, 100);
				this->joints[i]->SetHighStop(0, angle_dest);
				//gzmsg << "cur" << i << ": " << this->joints[i]->GetAngle(0).Radian() << ", to: "
				//		<< this->angle_dest[i] << ", ++\n";
			}
		}
	}

	if (!(is_steping || published || this->angle_mv_flag[0] ||
			this->angle_mv_flag[1] || this->angle_mv_flag[2])) {

		gazebo::msgs::Pose msg;

		if (step_count >= step_count_end) {
			// advertise current axis
			gazebo::msgs::Set(&msg, math::Pose(axis_dest.x, axis_dest.y, axis_dest.z, 0, 0, 0));
		} else {
			// advertise current axis
			double x, y, z;
			angle_to_axes(angle_dest[0], angle_dest[1], angle_dest[2], x, y, z);
			gzwarn << "stop at: x: " << x << ", y: " << y << ", z: " << z << std::endl;
			gazebo::msgs::Set(&msg, math::Pose(x, y, z, 0, 0, 0));
		}
		pub->Publish(msg);

		math::Pose spos = sucker->GetRelativePose();
		gzdbg << "real pose: x: " << spos.pos.y * -1
				<< ", y: " << spos.pos.x << ", z: " << spos.pos.z << std::endl;

		published = true;
	}

	// resume original pause-state
	this->world->SetPaused(is_paused);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UarmPlugin)
