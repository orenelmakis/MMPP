#include <ros/ros.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <vector>

using namespace std;

namespace pathPlannings
{
    class Node
    {
        public:
        Node(Eigen::Vector2i& pose);
        
        Node* parent;

        Eigen::Vector2i pose;

        double cost;

        int step;


    };


    class pathOptimizer
    {
        public:
        pathOptimizer(ros::NodeHandle& nh);
        void setVariables();
        
        vector<Eigen::Vector2i> solveAStar(Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& start, Eigen::Vector2i& goal);

        bool saftyFunc(Eigen::Vector2i& pose, Eigen::MatrixXi& occupancyMap);

        double calculateDistance(Eigen::Vector2i& pose, Eigen::Vector2i& goal);
        
        Node* checkFunc(Eigen::Vector2i& pose, vector<Node*>& closedSet);
        Node* minimalNode(vector<Node*>& openSet);
        Node* newNode(Eigen::Vector2i& position, Eigen::Vector2i &goal, Node* parent, Eigen::Vector2i& step);

        vector<Eigen::Vector2i> pathConnector(vector<vector<Eigen::Vector2i>>& path, vector<Eigen::MatrixXi>& occupancyMapVector);

        private:
        vector<Eigen::Vector2i> motion_;
        ros::NodeHandle& nh_;
    };



}