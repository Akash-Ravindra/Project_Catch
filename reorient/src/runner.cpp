#include "../include/frame_transform.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_node");
    tr::Transform transform;
    ros::spin();
    return 0;
}