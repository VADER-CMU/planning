#include <iostream>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>
#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>

//Node that is launched alongside the planner.
//When this node is destructed, it gets the PID of the `gzclient` process and kills it with kill -9.

class GazeboAssassin {
public:
    GazeboAssassin() {
        // Constructor: nothing to do
    }

    ~GazeboAssassin() {
        // Find gzclient PID(s)
        std::vector<pid_t> gzclient_pids = getGzclientPIDs();
        for (pid_t pid : gzclient_pids) {
            if (pid > 0) {
                std::cout << "[GazeboAssassin] Killing gzclient PID: " << pid << std::endl;
                kill(pid, SIGKILL);
            }
        }
    }

private:
    std::vector<pid_t> getGzclientPIDs() {
        std::vector<pid_t> pids;
        // Use pgrep to find gzclient processes
        FILE* pipe = popen("pgrep gzclient", "r");
        if (!pipe) return pids;
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::istringstream iss(buffer);
            pid_t pid;
            if (iss >> pid) {
                pids.push_back(pid);
            }
        }
        pclose(pipe);
        return pids;
    }
};
int main(int argc, char** argv) {
    // Minimal ROS initialization (if needed)
    ros::init(argc, argv, "gazebo_assassin");

    GazeboAssassin assassin;

    // Keep the node alive (if using ROS, otherwise just exit)
    ros::spin();

    return 0;
}