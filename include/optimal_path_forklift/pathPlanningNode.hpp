#include <ros/ros.h>
#include <optimal_path_forklift/pathPlanning.hpp>
#include <bits/stdc++.h>
#include <nav_msgs/Path.h>

using namespace std;


namespace pathPlannings 
{
    class simplePlanningsNode
    {
        public:

        simplePlanningsNode(ros::NodeHandle& nh);
        void setParam();
        void setVariables(Eigen::MatrixXi& goalMap, Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& initialPosition, Eigen::Vector2i& goalPosition);
        void setPublisher();
        

        private:
        ros::NodeHandle& nh_;
        ros::Publisher path_pub_;

        //paramerters
        string pathTopic;
        string markerTopic;

        //class method
        simplePlannings simplePlanning_;
        

    };

}