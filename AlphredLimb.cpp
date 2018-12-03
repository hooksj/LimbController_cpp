//
// Created by Joshua Hooks on 11/30/18.
//

#include "AlphredLimb.h"
#include <Eigen/Dense>
#include <math.h>

AlphredLimb::AlphredLimb(Eigen::Vector3i IDs, std::string PORT_NAME, double clocking_angle)
{
    motor_ids_ = IDs;

    // Setup the cBear manager with the correct port name.

    clocking_rot_ << cos(clocking_angle), -sin(clocking_angle), 0,
                     sin(clocking_angle), cos(clocking_angle), 0,
                     0, 0, 1;
}


void AlphredLimb::get_commands()
{
    // Get the commands from shared memory and populate member variables.
}


void AlphredLimb::initialize()
{
    // Check to see if the right BEARs and connected to the USB dongals
    // Set the gains and settings for all BEARs
}


int AlphredLimb::check_mode()
{
    // checks to see if the commanded mode is the same as the current mode in member variables. If not then safely
    // update the mode.

    return 99;
}


int AlphredLimb::set_ee_force(bool id)
{
    // commands torques to the BEARS based off of the desired end effector forces.
    return 0;
}


void AlphredLimb::set_ee_velocity()
{
    ;
}


void AlphredLimb::set_ee_position()
{
    ;
}


void AlphredLimb::set_joint_torques()
{
    ;
}


void AlphredLimb::set_joint_vel()
{
    ;
}


void AlphredLimb::set_joint_pos()
{
    ;
}


void AlphredLimb::update_limb_states()
{
    ;
}


double AlphredLimb::dt()
{
    return dt_;
}