//
// Created by Joshua Hooks on 11/30/18.
//

#ifndef LEG_CONTROLLER_ALPHREDLIMB_H
#define LEG_CONTROLLER_ALPHREDLIMB_H

#include <Eigen/Dense>

/*
 * This class controls a limb on ALPHRED. The only difference between limbs is the motor ID's and the clocking angle.
 *
 * Note: this class is in the Robot coordinate system. All commands must be first rotated into this coordinate system.
 */
class AlphredLimb {

    int bear_mode_ = 99;          // Indicates which mode the actuators are in
    int limb_mode_ = 99;          // Indicates which mode the limb is in

    int baudrate_ = 3000000; // Communication that cBear should run at

    double frequency_ = 100.0; // the frequency that the limb controller will run at
    double dt_ = 1/frequency_;

    /*
     * A clocking rotation matrix is used to rotate all FK and IK commands into the same coordinate system.
     * This allows for all of the limbs to use the same FK and IK functions.
     */
    Eigen::Matrix3d clocking_rot_;     // Rotation matrix that corresponds to the clocking angle of the limb

    // Motor data
    double MOTOR_CONST_ = 0.88;
    double INV_MOTOR_CONST_ = 1/MOTOR_CONST_;
    double VELRATE_ = 4000.0;
    double ENCODER_COUNTS_ = 262144;
    double ENC2RAD_ = 2*3.14159/ENCODER_COUNTS_;
    double RAD2ENC_ = 1.0/ENC2RAD_;


    Eigen::Vector3i motor_ids_;  // List of motor IDs for this limb

    // Currently not implemented just a place holder
    int cbm_;    // cBear Manager, used to connect to the BEARs
    int smm_;    // Shared Memory Manager, used to communicate with the shared memory

    // Commanded values
    int com_mode_ = 0;
    Eigen::Vector3d com_ee_force_;
    Eigen::Vector3d com_ee_pos_;
    Eigen::Vector3d com_ee_vel_;
    Eigen::Vector3d com_joint_torque_;
    Eigen::Vector3d com_joint_pos_;
    Eigen::Vector3d com_joint_vel_;

    // Torque limit: set in Amps
    double MAX_TORQUE_ = 40.0;
    double MAX_IQ = MAX_TORQUE_*INV_MOTOR_CONST_;

    // Joint Limits: set in radians
    double HIP_YAW_LIMIT_PLUS_ = 0.87;
    double HIP_YAW_LIMIT_MINUS_ = -0.87;

    double HIP_PITCH_LIMIT_PLUS_ = 1.70;
    double HIP_PITCH_LIMIT_MINUS_ = -1.70;

    double KNEE_PITCH_LIMIT_PLUS_ = 2.79;
    double KNEE_PITCH_LIMIT_MINUS_ = -2.79;

    /*
     * Motor gains to set on the BEAR actuators
    */
    // Hip Yaw
    // Position PID loop
    double HIP_YAW_POS_P_ = 0.050;
    double HIP_YAW_POS_I_ = 0.00;
    double HIP_YAW_POS_D_ = 0.1;

    // Velocity PID loop
    double HIP_YAW_VEL_P_ = 0.40;
    double HIP_YAW_VEL_I_ = 0.0;
    double HIP_YAW_VEL_D_ = 0.0;

    // Force PID loop
    double HIP_YAW_FOR_P_ = 0.05;
    double HIP_YAW_FOR_I_ = 0.0;
    double HIP_YAW_FOR_D_ = 1.5;

    // Hip Pitch
    // Position PID loop
    double HIP_PITCH_POS_P_ = 0.05;
    double HIP_PITCH_POS_I_ = 0.00;
    double HIP_PITCH_POS_D_ = 0.10;

    // Velocity PID loop
    double HIP_PITCH_VEL_P_ = 0.40;
    double HIP_PITCH_VEL_I_ = 0.00;
    double HIP_PITCH_VEL_D_ = 0.00;

    // Force PID loop
    double HIP_PITCH_FOR_P_ = 0.05;
    double HIP_PITCH_FOR_I_ = 0.00;
    double HIP_PITCH_FOR_D_ = 1.5;

    // Knee Pitch
    // Position PID loop
    double KNEE_PITCH_POS_P_ = 0.05;
    double KNEE_PITCH_POS_I_ = 0.00;
    double KNEE_PITCH_POS_D_ = 0.10;

    // Velocity PID loop
    double KNEE_PITCH_VEL_P_ = 0.80;
    double KNEE_PITCH_VEL_I_ = 0.002;
    double KNEE_PITCH_VEL_D_ = 0.001;

    // Force PID loop
    double KNEE_PITCH_FOR_P_ = 0.075;
    double KNEE_PITCH_FOR_I_ = 0.00;
    double KNEE_PITCH_FOR_D_ = 1.5;


public:

    // 3x1 vectors that contain the current states of the limb
    Eigen::Vector3d curr_ee_pos;
    Eigen::Vector3d curr_ee_vel;
    Eigen::Vector3d curr_joint_torque;
    Eigen::Vector3d curr_joint_pos;
    Eigen::Vector3d curr_joint_vel;

    int connected = 0;      // 1 when the highlevel controller is running


    /*
     * Initializes the class for controlling one of ALPHRED's limbs.
     * - Sets up the cBear manager.
     * - Sets the gains for all motor controllers.
     * - Sets the min/max values for torque and position of each actuator.
     *
     * IDs:              List motor ID's that make up the limb.
     * port_name:        The name of the usb port that the limb is communicating with.
     * clocking_angle:   Angle of the leg in radians measured from the +X axis
     */
    AlphredLimb(Eigen::Vector3i IDs, std::string port_name, double clocking_angle);


    /*
     * Grabs the commands that are currently in the shared memory and updates member values.
     */
    void get_commands();


    /*
     * Initialize all of the BEARs with the correct settings.
     */
    void initialize();


    /*
     * Checks if the current limb mode is the same as the commanded limb mode. If not then it will safely change
     * the actuator mode.
     *
     * returns the current limb mode.
     *
     *   Limb modes
     *   99 - No mode has been set
     *   0  - EE force commands / Inverse Dynamics
     *   1  - EE force commands / Jacobian
     *   2  - EE velocity commands
     *   3  - EE position commands
     *   11 - joint torque commands
     *   12 - joint velocity commands
     *   13 - joint position commands
     *
     *   BEAR modes
     *   99 - No mode has been set
     *   0  - Torque mode
     *   1  - Velocity mode
     *   2  - Position mode
     *   3  - Direct force mode (It is actually a position command and is what is being used over position mode)
     */
    int check_mode();


    /*
     * Sets the torques for each actuator based off of the desired end effector forces.
     *
     * id: a boolean that determines whether the jacobian should be used or the full inverse dynamics
     *
     * returns 0 if all calculations were done successfully.
     */
    int set_ee_force(bool id);


    /*
     * Sets the velocities for each actuator based off of the desired end effector velocities.
     *
     * This is done using the jacobian.
     */
    void set_ee_velocity();


    /*
     * Sets the position of each actuator based off the end effector desired position.
     *
     * This is done through inverse kinematics.
     */
    void set_ee_position();


    /*
     * Set the torques for each actuator in the limb
     */
    void set_joint_torques();


    /*
     * Set the velocity for each actuator in the limb
     */
    void set_joint_vel();


    /*
     * Set the position for each actuator in the limb
     */
    void set_joint_pos();


    /*
     * Write out the current state of the limb to shared memory
     */
    void update_limb_states();

    /*
     * Returns the cycle time for this thread.
     */
    double dt();



};


#endif //LEG_CONTROLLER_ALPHREDLIMB_H
