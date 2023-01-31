#include "rtabmap_ros/OccupancyGridMapWrapper.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_grid_map");

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    // process "--params" argument
    for(int i = 1; i < argc; ++i)
    {
        if(strcmp(argv[i], "--udebug") == 0)
        {
            ULogger::setLevel(ULogger::kDebug);
        }
        else if(strcmp(argv[i], "--uinfo") == 0)
        {
            ULogger::setLevel(ULogger::kInfo);
        }
    }

    rtabmap_ros::OccupancyGridMapWrapper occupancyGridMapWrapper(argc, argv);
    ros::spin();
    return 0;
}
