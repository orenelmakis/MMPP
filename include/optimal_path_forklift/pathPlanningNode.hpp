#include <ros/ros.h>
#include <optimal_path_forklift/pathPlanning.hpp>
#include <optimal_path_forklift/pathOptimizer.hpp>
#include <bits/stdc++.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


namespace pathPlannings 
{
    class simplePlanningsNode
    {
        public:

        simplePlanningsNode(ros::NodeHandle& nh);
        void setParam();
        void setVariables(Eigen::MatrixXi& goalMap, Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& initialPosition, Eigen::Vector3i& goalPosition);
        void setPublisher();
        void visualizeMarkerbuild(vector<Eigen::Vector2i>& path, Eigen::MatrixXi& occupancyMap);
        void visualizeMarkerPublish(int& time);
        
        simplePlannings simplePlanning_;
        pathOptimizer simpleOptimizer_;

        vector<Eigen::Vector2i> agentPath;
        vector<Eigen::MatrixXi> occupancyMapitr;

        private:
        ros::NodeHandle& nh_;
        ros::Publisher markerPub;

        //paramerters
        string pathTopic;
        string markerTopic;

        //class method

        visualization_msgs::MarkerArray MarkerArray_;
        vector<vector<visualization_msgs::Marker>> MaterialMarkerVector;
        vector<visualization_msgs::Marker> robotMarkerVector;
        
        

    };

}