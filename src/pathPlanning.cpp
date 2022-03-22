#include <optimal_path_forklift/pathPlanning.hpp>


namespace pathPlannings
{
    pathNode::pathNode(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost):
    parent_(parent), target_(target), pose_(pose), cost(cost)
    {
        setVariables(occupancyMap);
    }

    void pathNode::setVariables(Eigen::MatrixXd& occupancyMap)
    {
        occupancyMap_.resize(occupancyMap.rows(), occupancyMap.cols());
        occupancyMap_ = occupancyMap;
    }


    simplePlannings::simplePlannings(ros::NodeHandle& nh, Eigen::MatrixXd& occupancyMap, Eigen::MatrixXd& goalMap, Eigen::Vector2d& initialPosition,
        Eigen::Vector2d& constructionDirection, vector<Eigen::Vector2d>& motion):
        nh_(nh),
        initialPosition_(initialPosition),
        constructionDirection_(constructionDirection)
    {
        setVariables(occupancyMap, motion, goalMap);
    }


    simplePlannings::~simplePlannings()
    {
    }

    void simplePlannings::setVariables(Eigen::MatrixXd& occupancyMap, vector<Eigen::Vector2d>& motion, Eigen::MatrixXd& goalMap)
    {
        occupancyMap_.resize(occupancyMap.rows(), occupancyMap.cols());
        occupancyMap_ = occupancyMap;

        motion_.resize(motion.size());
        motion_ = motion;

        goalMap_.resize(goalMap.rows(), goalMap.cols());
        goalMap_ = goalMap;
    
    }

    void simplePlannings::planning()
    {
        ROS_INFO_STREAM("occupancyMap_: " << occupancyMap_);
    }

    int simplePlannings::calculateCost(Eigen::MatrixXd& occupancyMap)
    {
        ROS_INFO_STREAM("occupancyMap_: " << occupancyMap_);
        int cost = 0;
        for(auto i=0; i < occupancyMap_.rows(); i++)
        {
            for(auto j=0; j < occupancyMap_.cols(); j++)
            {
                if(occupancyMap_(i,j) == 1 && occupancyMap_(i,j) != goalMap_(i,j))
                {
                    cost += 1;
                }
            }
        }
        return cost;
    }

    void simplePlannings::solvePlanning()
    {}


    void simplePlannings::pathNodesGenerator(Eigen::Vector2d& target)
    {
        target_ = target;
        ROS_INFO_STREAM("pathNodesGenerator");
        // create list of material locations
        std::vector<Eigen::Vector2d> materialLocations;
        for(auto i=0;i<occupancyMap_.rows();i++)
        {
            for(auto j=0;j<occupancyMap_.cols();j++)
            {
                if(occupancyMap_(i,j)>=1)
                {
                    Eigen::Vector2d materialPose_(i,j);
                    ROS_INFO_STREAM("materialPose_: " << materialPose_);
                    //Without reference to obstacles
                    double cost = 0.0;
                    pathNode* node =  new pathNode(NULL, target, materialPose_, occupancyMap_, cost);
                    Eigen::MatrixXd occupancyMap = occupancyMap_;
                    parents_.push_back(pathNodeGenerator(node, target_, materialPose_ ,occupancyMap));
                    ROS_INFO_STREAM("parents_: " << parents_.back()->childrens_.size());

                }

            }
        }
    
    }

    bool simplePlannings::limitTest(Eigen::Vector2d& checkPosition)
    {
        if(checkPosition(0,0)>=0 && checkPosition(1,0)>=0 && checkPosition(0,0)<occupancyMap_.rows() && checkPosition(1,0)<occupancyMap_.cols())
        {
            if(occupancyMap_(checkPosition(0,0),checkPosition(1,0))==0)
            {
                return true;
            }
        }
        return false;
    }

    pathNode* simplePlannings::pathNodeGenerator(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& materialPose , Eigen::MatrixXd& occupancyMap)
    {
        for (auto k=0;k<motion_.size();k++)
            {
                Eigen::Vector2d checkPosition = materialPose - motion_[k];
                    if(limitTest(checkPosition) && motion_[k].dot(target-materialPose) > 0)
                    {
                        double motionCost = motion_[k].dot(target-materialPose);
                        Eigen::Vector2d materialPoseNew = materialPose + motion_[k]*motionCost;
                        occupancyMap(materialPoseNew(0),materialPoseNew(1)) = 1;
                        occupancyMap(materialPose(0),materialPose(1)) = 0;
                        if((target-materialPoseNew).norm() == 0)
                        {
                            pathNode* node =  new pathNode(parent, target, materialPoseNew, occupancyMap, motionCost);
                            return node;
                        }
                        pathNode* node = new pathNode(parent, target, materialPoseNew, occupancyMap, motionCost);
                        node->childrens_.push_back(pathNodeGenerator(node, target, materialPoseNew, occupancyMap));
                        parent->childrens_.push_back(node);
                    }
            }
        return parent;
    }


}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "simplePlannings");
    ros::NodeHandle nh;
    Eigen::MatrixXd occupancyMap(5,5);
    occupancyMap = Eigen::MatrixXd::Zero(5,5);
    occupancyMap(1,2) = 1;
    occupancyMap(2,2) = 1;

    Eigen::MatrixXd goalMap(5,5);
    goalMap = Eigen::MatrixXd::Zero(5,5);
    goalMap(3,3) = 1;
    Eigen::Vector2d target(3,3);

    Eigen::Vector2d initialPosition(0,0);
    Eigen::Vector2d constructionDirection(0,1);

    vector<Eigen::Vector2d> motion = {Eigen::Vector2d(1,0), Eigen::Vector2d(0,1), Eigen::Vector2d(-1,0), Eigen::Vector2d(0,-1)};

    pathPlannings::simplePlannings simplePlannerClass(nh, occupancyMap, goalMap, initialPosition, constructionDirection,motion);
    // ROS_INFO_STREAM("occupancyMap_: " << occupancyMap);
    simplePlannerClass.pathNodesGenerator(target);
    ros::spin();

    return 0;
}