#include <optimal_path_forklift/pathPlanning.hpp>


namespace pathPlannings
{
    pathNode::pathNode(pathNode* parent, Eigen::Vector2d& target, Eigen::Vector2d& pose, Eigen::MatrixXd& occupancyMap, double& cost):
    parent_(parent), target_(target), pose_(pose), occupancyMap_(occupancyMap), cost(cost)
    {
    }



    simplePlannings::simplePlannings(ros::NodeHandle& nh, Eigen::MatrixXd& occupancyMap, Eigen::MatrixXd& goalMap, Eigen::Vector2d& initialPosition,
        Eigen::Vector2d& constructionDirection, vector<Eigen::Vector2d>& motion):
        nh_(nh),
        occupancyMap_(occupancyMap),
        target_(goalMap),
        initialPosition_(initialPosition),
        constructionDirection_(constructionDirection),
        motion_(motion)
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
                    Eigen::Vector2d materialPose_(i,j);
                    //Without reference to obstacles
                    double cost = 0.0;
                    parents_.push_back(pathNodeGenerator(NULL, target_, materialPose_ ,occupancyMap_, cost));
                    for (auto k=0;k<motion_.size();k++)
                    {
                        Eigen::Vector2d checkPosition = Eigen::Vector2d(i,j) - motion_[k];
                            if(limitTest(checkPosition) && motion_[k].dot(target_-materialPose_) > 0)
                            {
                                double cost = motion_[k].dot(target_-materialPose_);
                                Eigen::Vector2d position = Eigen::Vector2d(i,j) + motion_[k]*cost;
                                pathNode* node = pathNodeGenerator(parents_.back(), target_, position, occupancyMap_, cost);
                                parents_.back()->childrens_.push_back(node);
                            }
                        }
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

    vector<Eigen::Vector2d> motion = {Eigen::Vector2d(1,0), Eigen::Vector2d(0,1), Eigen::Vector2d(-1,0), Eigen::Vector2d(0,-1)};


    pathPlannings::simplePlannings simplePlannings(nh, occupancyMap, goalMap, initialPosition, constructionDirection,motion);
    ros::spin();

    return 0;
}