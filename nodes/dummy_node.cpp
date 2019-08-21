#include <iostream>
#include <random>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>//publish

int randVal(int min,int max)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(min, max); // distribution in range [1, 6]

    return dist6(rng);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle n;

    ros::Publisher data1 = n.advertise<geometry_msgs::Vector3>("/random/data1", 300);
    ros::Publisher data2 = n.advertise<geometry_msgs::Vector3>("/random/data2", 300);

    ros::Rate loop_rate(10);
    geometry_msgs::Vector3 buff;

    if(argc > 3)
    {
        while (ros::ok())
        {
            buff.x = randVal(atoi(argv[1]), atoi(argv[2]));
            buff.y = randVal(atoi(argv[1]), atoi(argv[2]));
            buff.z = randVal(atoi(argv[1]), atoi(argv[2]));
            data1.publish(buff);

            buff.x = randVal(atoi(argv[1]), atoi(argv[2]));
            buff.y = randVal(atoi(argv[1]), atoi(argv[2]));
            buff.z = randVal(atoi(argv[1]), atoi(argv[2]));
            data2.publish(buff);

            ros::spinOnce();
            loop_rate.sleep();

        }
    }
    else
    {
        while (ros::ok())
        {
            buff.x = randVal(1, 6);
            buff.y = randVal(1, 6);
            buff.z = randVal(1, 6);
            data1.publish(buff);

            buff.x = randVal(1, 6);
            buff.y = randVal(1, 6);
            buff.z = randVal(1, 6);
            data2.publish(buff);

            ros::spinOnce();
            loop_rate.sleep();

        }

    }
    return 0;


}
