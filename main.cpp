#include <iostream>
#include "AlphredLimb.h"
#include <Eigen/Dense>
#include <chrono>
#include <unistd.h>

int main() {
    Eigen::Vector3i IDs;
    IDs << 1, 2, 3; // Set motor ids

    double CA = 3.1415/4.0; // set clocking angle

    std::string port_name = "port1"; // set port name

    AlphredLimb limb1(IDs, port_name, CA); // initialize the limb

    std::cout << "Limb1 successfully connected to " << port_name << std::endl;
    std::cout << "Waiting for connection to main thread..." << std::endl;

    bool first = true;

    int count = 0;

    auto last_time = std::chrono::high_resolution_clock::now();

    auto t1 = std::chrono::high_resolution_clock::now();

    while(1)
    {
        limb1.get_commands(); // get the latest commands from shared memory

        // Check to see if the main thread is connected, if not do nothing
        if (limb1.connected)
        {
            // If this is the first loop after being connected then initialize the BEARs
            if (first)
            {
                first = false;
                limb1.initialize();
            }

            int mode;
            mode = limb1.check_mode(); // Determine which mode the limb is in and switch if need be

            // Send commands based of the current mode
            switch (mode)
            {
                case 0:
                    int error1;
                    error1 = limb1.set_ee_force(false);
                    break;
                case 1:
                    int error2;
                    error2 = limb1.set_ee_force(true);
                    break;
                case 2:
                    limb1.set_ee_velocity();
                case 3:
                    limb1.set_ee_position();
                case 11:
                    limb1.set_joint_torques();
                case 12:
                    limb1.set_joint_vel();
                case 13:
                    limb1.set_joint_pos();
                default:
                    // do nothing
                    ;
            }

            limb1.update_limb_states(); // Update the status of the limb

        }

        // Perform timing to keep thread at a constant frequency
        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - last_time;
        int sleep_time = (long) ((limb1.dt() - elapsed_time.count())*1000000000.0*0.6); //  nanoseconds
        struct timespec tim1, tim2;
        tim1.tv_sec = 0;
        tim1.tv_nsec = sleep_time;
        if (sleep_time > 0.0)
            nanosleep(&tim1,&tim2);

        while (limb1.dt() - elapsed_time.count() > 0.0)
            elapsed_time = std::chrono::high_resolution_clock::now() - last_time;
        //std::cout << elapsed_time.count() << std::endl;

        last_time = std::chrono::high_resolution_clock::now();


        count++;

        if (count > 100)
            break;

    }

    std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - t1;
    std::cout << elapsed_time.count() << std::endl;
    return 0;
}