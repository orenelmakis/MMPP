#include <optimal_path_forklift/pathPlanningNode.hpp>

namespace pathPlannings 
{
    simplePlanningsNode::simplePlanningsNode(ros::NodeHandle& nh): nh_(nh), simplePlanning_(nh), simpleOptimizer_(nh)
    {
        setParam();
        setPublisher();
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
        markerPub = nh_.advertise<visualization_msgs::MarkerArray>(pathTopic, 1);
    }

    void simplePlanningsNode::setVariables(Eigen::MatrixXi& goalMap, Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& initialPosition, Eigen::Vector2i& goalPosition)
    {
        simplePlanning_.setVariables(goalMap, occupancyMap, initialPosition, goalPosition);
    }


    void simplePlanningsNode::visualizeMarkerbuild(vector<Eigen::Vector2i>& path, Eigen::MatrixXi& occupancyMap)
    {
        occupancyMapitr.push_back(occupancyMap);
        int counter = 0;
        for(auto it=path.begin()+1; it!=path.end();it++)
        {
            Eigen::Vector2i motion = (*it)-(*(it-1));
            ROS_INFO_STREAM("motion " << motion);
            ROS_INFO_STREAM("pose " << *it);
            if(occupancyMap((*it)(0),(*it)(1)) > 0)
            {
                occupancyMap((*it)(0)+motion(0),(*it)(1)+motion(1)) += occupancyMap((*it)(0),(*it)(1));
                occupancyMap((*it)(0),(*it)(1)) = 0;
            }
            Eigen::MatrixXi occupancyMapTemp = occupancyMap;
            occupancyMapitr.push_back(occupancyMapTemp);
            visualization_msgs::Marker pathMarker;
            pathMarker.header.frame_id = "map";
            pathMarker.header.stamp = ros::Time::now();
            pathMarker.id = counter;
            pathMarker.type = visualization_msgs::Marker::CUBE;
            pathMarker.scale.x = 0.5+abs(motion(1));
            pathMarker.scale.y = 0.5+abs(motion(0));
            pathMarker.scale.z = 0.5;
            pathMarker.color.r = 0;
            pathMarker.color.g = 1;
            pathMarker.color.b = 0;
            pathMarker.color.a = 1;
            pathMarker.pose.orientation.w = 1.0;
            pathMarker.pose.position.x = (*it)(0);
            pathMarker.pose.position.y = (*it)(1);
            pathMarker.pose.position.z = 0;
            robotMarkerVector.push_back(pathMarker);
            counter++;




            // ROS_INFO_STREAM("occupancyMapitr: " << occupancyMap);
        }
        
        for(auto it=occupancyMapitr.begin()+1; it!=occupancyMapitr.end();it++)
        {
            counter = 100;
            vector<visualization_msgs::Marker> MaterialMarker;
            for(auto i=0; i<(*it).rows(); i++)
            {
                for(auto j=0; j < (*it).cols(); j++)
                {
                if((*it)(i,j)>0)
                    {
                    visualization_msgs::Marker pathMarker;
                    pathMarker.header.frame_id = "map";
                    pathMarker.header.stamp = ros::Time::now();
                    pathMarker.id = counter;
                    pathMarker.type = visualization_msgs::Marker::CUBE;
                    pathMarker.scale.x = 0.2*(1+(*it)(i,j));
                    pathMarker.scale.y = 0.2*(1+(*it)(i,j));
                    pathMarker.scale.z = 0.5;
                    pathMarker.color.r = 0.4;
                    pathMarker.color.g = 0.3;
                    pathMarker.color.b = 0.3;
                    pathMarker.color.a = 1;
                    pathMarker.pose.orientation.w = 1.0;
                    pathMarker.pose.position.x = i;
                    pathMarker.pose.position.y = j;
                    pathMarker.pose.position.z = 0;
                    MaterialMarker.push_back(pathMarker);
                    counter++;
                    }
                }
            }
            MaterialMarkerVector.push_back(MaterialMarker);

        }


    }

    void simplePlanningsNode::visualizeMarkerPublish(int& time)
    {
        MarkerArray_.markers.clear();
        if(time==robotMarkerVector.size())
        {
            time = 0;
        }
        MarkerArray_.markers.push_back(robotMarkerVector[time]);
        for(auto it=MaterialMarkerVector[time].begin(); it!=MaterialMarkerVector[time].end(); it++)
        {
            MarkerArray_.markers.push_back(*it);
        }
        visualization_msgs::MarkerArray marker_array_msg;
        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array_msg.markers.push_back(marker);

        markerPub.publish(marker_array_msg);
        markerPub.publish(MarkerArray_);



    }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathPlanning_node");
    ros::NodeHandle nh;
    pathPlannings::simplePlanningsNode simplePlanningsNode(nh);
    int value = 12;
    Eigen::MatrixXi occupancyMap(value,value);
    occupancyMap = Eigen::MatrixXi::Zero(value,value);
    occupancyMap(1,2) = 1;
    occupancyMap(2,2) = 1;
    occupancyMap(1,1) = 1;
    occupancyMap(2,1) = 1;
    occupancyMap(3,2) = 1;
    occupancyMap(4,2) = 1;
    // occupancyMap(5,2) = 1;
    // occupancyMap(5,3) = 1;
    // occupancyMap(5,4) = 1;
    // occupancyMap(6,3) = 1;
    // occupancyMap(5,3) = 1;
    // occupancyMap(5,4) = 1;
    // occupancyMap(8,2) = 1;
    // occupancyMap(3,3) = 1;
    // occupancyMap(2,3) = 1;

    Eigen::MatrixXi goalMap(value,value);
    goalMap = Eigen::MatrixXi::Zero(value,value);
    goalMap(1,1) = 8;
    Eigen::Vector2i target(9,9);

    Eigen::Vector2i initialPosition(0,0);

    simplePlanningsNode.setVariables(occupancyMap, goalMap, initialPosition, target);
    // ROS_INFO_STREAM("occupancyMap_: " << occupancyMap);
    simplePlanningsNode.simplePlanning_.pathNodesGenerator(target);
    simplePlanningsNode.simplePlanning_.solvePlanning();

    for (int i = simplePlanningsNode.simplePlanning_.agentPath.size()-1; i >= 0; i--)
    {
        ROS_INFO_STREAM("agentPath_: ");
        for (int j = 0; j < simplePlanningsNode.simplePlanning_.agentPath[i].size(); j++)
        {
            ROS_INFO_STREAM( simplePlanningsNode.simplePlanning_.agentPath[i][j]);
        }
        
    }
    simplePlanningsNode.agentPath = simplePlanningsNode.simpleOptimizer_.pathConnector(simplePlanningsNode.simplePlanning_.agentPath, simplePlanningsNode.simplePlanning_.agentOccupanyMap);
    simplePlanningsNode.visualizeMarkerbuild(simplePlanningsNode.agentPath,occupancyMap);


    int time = 0;
    ros::Rate rate(5);
    while(ros::ok())
    {
        simplePlanningsNode.visualizeMarkerPublish(time);
        time++;
        rate.sleep();
    }
    return 0;
}


