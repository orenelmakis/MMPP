#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <bits/stdc++.h>



using namespace std;


namespace pathPlannings 
{




    class pathNode
    {
        public:
            pathNode(pathNode* parent, Eigen::Vector2i& target, Eigen::Vector2i& pose, Eigen::MatrixXi& occupancyMap, int& cost);
            
            void setVariables(Eigen::MatrixXi&);
            int cost;
            Eigen::Vector2i pose_;
            vector<pathNode*> childrens_;
            Eigen::Vector2i motion;
        private:
            pathNode* parent_;
            
            Eigen::Vector2i target_;
            
            Eigen::MatrixXi occupancyMap_;
            

    };

    struct comp
    {
        bool operator()(const pathNode* a, const pathNode* b) const
        {
            return a->cost > b->cost;
        }
    };


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
            simplePlannings(ros::NodeHandle& nh ,Eigen::MatrixXi& occupancyMap, Eigen::MatrixXi& goalMap, Eigen::Vector2i& initialPosition, 
            Eigen::Vector2i& constructionDirection);
            ~simplePlannings();
            bool limitTest(Eigen::Vector2i& newPosition, Eigen::MatrixXi& occupancyMap);
            void setVariables(Eigen::MatrixXi&, Eigen::MatrixXi&);
            void planning();
            void pathNodesGenerator(Eigen::Vector2i& target);
            pathNode* pathNodeGenerator(pathNode* parent, Eigen::Vector2i& target, Eigen::Vector2i& materialPose , Eigen::MatrixXi& occupancyMap);
            
            int calculateDistance(Eigen::Vector2i& pose, Eigen::Vector2i& target);
            int calculateCost(Eigen::MatrixXi& occupancyMap);
            int calculateCostDistance(Eigen::MatrixXi& occupancyMap);
            void solvePlanning();
            int stepPlanning(Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& motion, Eigen::Vector2i& materialPose, Eigen::Vector2i& target);
            planningNode* newPlanningNode(Eigen::Vector2i& pose, planningNode* parent, double& cost, Eigen::MatrixXi& occupancyMap);


        vector<Eigen::Vector2i> agentPath;
        private:
            ros::NodeHandle nh_;
            Eigen::MatrixXi occupancyMap_;
            Eigen::MatrixXi goalMap_;
            Eigen::Vector2i  target_;
            Eigen::Vector2i initialPosition_;
            vector<Eigen::Vector2i> path_;
            Eigen::Vector2i constructionDirection_;
            vector<Eigen::Vector2i> motion_;
            vector<pathNode*> parents_;
            







    };
}