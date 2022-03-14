#include <optimal_path_forklift/pathPlanning.hpp>


namespace pathPlannings
{
    pathNode::pathNode(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost):
    parent_(parent), target_(target), pose_(pose), occupancyMap_(occupancyMap), cost(cost)
    {
    }



    simplePlannings::simplePlannings(ros::NodeHandle& nh, Eigen::MatrixXd& occupancyMap, Eigen::MatrixXd& goalMap, Eigen::Vector2d& initialPosition,
        Eigen::Vector2d& constructionDirection):
        nh_(nh),
        occupancyMap_(occupancyMap),
        goalMap_(goalMap),
        initialPosition_(initialPosition),
        constructionDirection_(constructionDirection)
    {
    }


    simplePlannings::~simplePlannings()
    {
    }

    void simplePlannings::setVariables()
    {
    
    }

    void simplePlannings::planning()
    {
        ROS_INFO_STREAM("occupancyMap_: " << occupancyMap_);
    }

    void simplePlannings::targetCell()
    {
        if (constructionDirection_(0,0)==1)
        {
            Eigen::VectorXd targetcolumn = goalMap_.colwise().sum();
            ROS_INFO_STREAM("goalMap_: " << goalMap_);
            ROS_INFO_STREAM("targetcolumn: " << targetcolumn);
        }
        else
        {
            Eigen::VectorXd targetrow = goalMap_.rowwise().sum();
            ROS_INFO_STREAM("goalMap_: " << goalMap_);
            ROS_INFO_STREAM("targetrow: " << targetrow);
        }
    }

    void simplePlannings::pathNodesGenerator()
    {
        // create list of material locations
        std::vector<Eigen::Vector2d> materialLocations;
        for(auto i=0;i<occupancyMap_.rows();i++)
        {
            for(auto j=0;j<occupancyMap_.cols();j++)
            {
                if(occupancyMap_(i,j)>=1)
                {
                    materialLocations.push_back(Eigen::Vector2d(i,j));
                }
            }
        }
        
        
        
    }

    pathNode* simplePlannings::pathNodeGenerator(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost)
    {
        return new pathNode(parent, target, pose, occupancyMap, cost);
    }


}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "simplePlannings");
    ros::NodeHandle nh;
    Eigen::MatrixXd occupancyMap(5,5);
    occupancyMap = Eigen::MatrixXd::Zero(5,5);
    occupancyMap(0,3) = 1;

    Eigen::MatrixXd goalMap(5,5);
    goalMap = Eigen::MatrixXd::Zero(5,5);
    goalMap(0,4) = 1;

    Eigen::Vector2d initialPosition(0,0);
    Eigen::Vector2d constructionDirection(0,1);


    pathPlannings::simplePlannings simplePlannings(nh, occupancyMap, goalMap, initialPosition, constructionDirection);
    simplePlannings.targetCell();
    ros::spin();

    return 0;
}