#include <optimal_path_forklift/pathPlanningNode.hpp>

namespace pathPlannings 
{
    simplePlanningsNode::simplePlanningsNode(ros::NodeHandle& nh): nh_(nh), simplePlanning_(nh_)
    {

    }


    void simplePlanningsNode::setParam()
    {
        if(!nh_.getParam("path_topic", pathTopic))
        {
            pathTopic = "/optimalPath/path";
        }
        if(!nh_.getParam("marker_topic", markerTopic))
        {
            markerTopic = "/optimalPath/marker";
        }
    }

    void simplePlanningsNode::setPublisher()
    {
        path_pub_ = nh_.advertise<nav_msgs::Path>(pathTopic, 1);
    }

    void simplePlanningsNode::setVariables(Eigen::MatrixXi& goalMap, Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& initialPosition, Eigen::Vector2i& goalPosition)
    {
        simplePlanning_.setVariables(goalMap, occupancyMap, initialPosition, goalPosition);
    }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathPlanning_node");
    ros::NodeHandle nh;
    pathPlannings::simplePlanningsNode simplePlanningsNode(nh);
    ros::spin();
    return 0;
}




