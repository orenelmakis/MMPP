#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>


using namespace std;


namespace pathPlannings 
{

    struct comp
    {
        bool operator()(const pathNode* a, const pathNode* b) const
        {
            return a->cost> b->cost;
        }
    };


    class pathNode
    {
        public:
            pathNode(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost);
            
            
            double cost;
            vector<pathNode*> childrens_;
        private:
            pathNode* parent_;
            
            Eigen::Vector2d target_;
            Eigen::Vector2d pose_;
            Eigen::MatrixXd occupancyMap_;
            

    };

    class simplePlannings
    {
        public:
            simplePlannings(ros::NodeHandle& nh ,Eigen::MatrixXd& occupancyMap, Eigen::MatrixXd& goalMap, Eigen::Vector2d& initialPosition, 
            Eigen::Vector2d& constructionDirection, vector<Eigen::Vector2d>& motion);
            ~simplePlannings();
            bool limitTest(Eigen::Vector2d& newPosition);
            void setVariables();
            void planning();
            void pathNodesGenerator();
            pathNode* pathNodeGenerator(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost);

        private:
            ros::NodeHandle nh_;
            Eigen::MatrixXd occupancyMap_;
            Eigen::Vector2d  target_;
            Eigen::Vector2d initialPosition_;
            vector<Eigen::Vector2d> path_;
            Eigen::Vector2d constructionDirection_;
            vector<Eigen::Vector2d> motion_;
            vector<pathNode*> parents_;






    };
}