#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <bits/stdc++.h>



using namespace std;


namespace pathPlannings 
{
    class materialNode
    {
        public:
        materialNode(Eigen::Vector2i& location);

        Eigen::Vector2i location;

    };




    class pathNode
    {
        public:
            pathNode(pathNode* parent, Eigen::Vector2i& pose);
            
            Eigen::Vector2i pose_;
            vector<pathNode*> childrens_;
        private:
            pathNode* parent_;
                        
            

    };

    // struct comp
    // {
    //     bool operator()(const pathNode* a, const pathNode* b) const
    //     {
    //         return a->cost > b->cost;
    //     }
    // };


    class planningNode
    {   
        public:
        planningNode(Eigen::Vector2i& pose, planningNode* parent);
        void setVariables(Eigen::MatrixXi& occupancyMap, vector<pathNode*>& path);
        int cost;
        double distance;
        int price;
        Eigen::MatrixXi occupancyMap_;
        Eigen::Vector2i pose;
        Eigen::Vector2i poseTarget;
        vector<pathNode*> path;
        planningNode* parent;

    };

    struct compPlanning
    {
        bool operator()(const planningNode* a, const planningNode* b) const
        {
            return a->distance+a->price > b->distance+b->price;
        }
    };

    class simplePlannings
    {
        public:
            simplePlannings(ros::NodeHandle& nh);
            ~simplePlannings();
            bool limitTest(Eigen::Vector2i& newPosition, Eigen::MatrixXi& occupancyMap);
            void setVariables(Eigen::MatrixXi& goalMap, Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& initialPosition, Eigen::Vector3i& goalPosition);
            void planning();
            void pathNodesGenerator(int& edges,Eigen::Vector3i& target);
            pathNode* pathNodeGenerator(pathNode* parent, Eigen::Vector3i& target, Eigen::Vector2i& materialPose , vector<materialNode*> materials,int& edges);
            
            int calculateDistance(Eigen::Vector2i& pose, Eigen::Vector2i& target);
            int calculateCost(Eigen::MatrixXi& occupancyMap);
            int calculateCostDistance(Eigen::MatrixXi& occupancyMap);
            void solvePlanning();
            int stepPlanning(Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& motion, Eigen::Vector2i& materialPose, Eigen::Vector3i& target);

            planningNode* newPlanningNode(Eigen::Vector2i& pose, planningNode* parent, double& cost, Eigen::MatrixXi& occupancyMap);

        vector<vector<Eigen::Vector2i>> agentPath;
        vector<Eigen::MatrixXi> agentOccupanyMap;
        private:
            ros::NodeHandle nh_;
            Eigen::MatrixXi occupancyMap_;
            Eigen::MatrixXi goalMap_;
            Eigen::Vector3i  target_;
            Eigen::Vector2i initialPosition_;
            vector<Eigen::Vector2i> path_;
            Eigen::Vector2i constructionDirection_;
            vector<Eigen::Vector2i> motion_;
            pathNode* parentNode_;

            vector<materialNode*> materials_;
            







    };
}