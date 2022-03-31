#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>



using namespace std;


namespace pathPlannings 
{


    class planningNode
    {   
        public:
        planningNode(int& cost);
        int cost;
        Eigen::MatrixXd occupancyMap_;
        Eigen::Vector2d pose;

    };

    struct compPlanning
    {
        bool operator()(const planningNode* a, const planningNode* b) const
        {
            return a->cost> b->cost;
        }
    };

    class pathNode
    {
        public:
            pathNode(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost);
            
            void setVariables(Eigen::MatrixXd&);
            double cost;
            Eigen::Vector2d pose_;
            vector<pathNode*> childrens_;
        private:
            pathNode* parent_;
            
            Eigen::Vector2d target_;
            
            Eigen::MatrixXd occupancyMap_;
            

    };

    struct comp
    {
        bool operator()(const pathNode* a, const pathNode* b) const
        {
            return a->cost> b->cost;
        }
    };

    class simplePlannings
    {
        public:
            simplePlannings(ros::NodeHandle& nh ,Eigen::MatrixXd& occupancyMap, Eigen::MatrixXd& goalMap, Eigen::Vector2d& initialPosition, 
            Eigen::Vector2d& constructionDirection);
            ~simplePlannings();
            bool limitTest(Eigen::Vector2d& newPosition);
            void setVariables(Eigen::MatrixXd&, Eigen::MatrixXd&);
            void planning();
            void pathNodesGenerator(Eigen::Vector2d& target);
            pathNode* pathNodeGenerator(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& materialPose , Eigen::MatrixXd& occupancyMap);
            int calculateCost(Eigen::MatrixXd& occupancyMap);
            void solvePlanning();
            double stepPlanning(Eigen::MatrixXd& occupancyMap, Eigen::Vector2d& motion, Eigen::Vector2d& materialPose, Eigen::Vector2d& target);



        private:
            ros::NodeHandle nh_;
            Eigen::MatrixXd occupancyMap_;
            Eigen::MatrixXd goalMap_;
            Eigen::Vector2d  target_;
            Eigen::Vector2d initialPosition_;
            vector<Eigen::Vector2d> path_;
            Eigen::Vector2d constructionDirection_;
            vector<Eigen::Vector2d> motion_;
            vector<pathNode*> parents_;







    };
}