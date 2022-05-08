#include <optimal_path_forklift/pathOptimizer.hpp>


namespace pathPlannings
{
    Node::Node(Eigen::Vector2i& pose)
    {
        this->pose = pose;
        this->parent = NULL;
        this->cost = 0;
        this->step = 0;
    }

    pathOptimizer::pathOptimizer(ros::NodeHandle& nh): nh_(nh)
    {
        setVariables();
    }
    
    void pathOptimizer::setVariables()
    {
        motion_ = {Eigen::Vector2i(0, 1), Eigen::Vector2i(1, 0), Eigen::Vector2i(0, -1), Eigen::Vector2i(-1, 0)};
    }

    vector<Eigen::Vector2i> pathOptimizer::solveAStar(Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& start, Eigen::Vector2i& goal)
    {
        ROS_INFO_STREAM("solveAStar");
        ROS_INFO_STREAM("start " << start);
        ROS_INFO_STREAM("goal " << goal);
        vector<Eigen::Vector2i> path_;

        Node* current = nullptr;
        vector<Node*> closedSet, openSet;
        double cost = 0.0;
        Node* root = new Node(start);
        root->step = 0;
        root->cost = calculateDistance(start,goal) + root->step;

        openSet.push_back(root);
        int count = 0;
        while(openSet.size())
        {
            count++;
            if (count > 100)
            {
                ROS_INFO_STREAM("count > 100");
                break;
            }
            current = minimalNode(openSet);
            closedSet.push_back(current);
            if(current->pose == goal)
            {
                ROS_INFO_STREAM("goal found");
                break;
            }
            for(auto i : motion_)
            {
                Eigen::Vector2i newPose = current->pose + i;
                if(!saftyFunc(newPose, occupancyMap))
                {
                    ROS_INFO_STREAM("newPose " << newPose);
                    continue;
                }
                // ROS_INFO_STREAM(PushPose);
                Node* child = checkFunc(newPose, closedSet);
                if(child == nullptr)
                {
                    child = newNode(newPose, goal, current, i);
                    openSet.push_back(child);
                }
                else if(child->cost > calculateDistance(newPose, goal) + current->step+i.norm())
                {
                    child->parent = current;
                    child->step = current->step + i.norm();
                    child->cost = calculateDistance(newPose, goal) + child->step;

                }
            }


        }
        while(current != nullptr)
        {
            ROS_INFO_STREAM("current pose: " << current->pose(0) << " " << current->pose(1));
            path_.push_back(current->pose);
            current = current->parent;
        }
        reverse(path_.begin(), path_.end());

        return path_;


    }

    Node*  pathOptimizer::newNode(Eigen::Vector2i& position, Eigen::Vector2i &goal, Node* parent, Eigen::Vector2i& step)
    {   
        Node* node = new Node(position);
        node->pose = position;
        node->parent = parent;
        node->step = parent->step + step.norm();
        node->cost = calculateDistance(position, goal) + node->step;
        return node;
        
    }

    Node* pathOptimizer::checkFunc(Eigen::Vector2i& pose, vector<Node*>& closedSet)
    {
       for(auto i:closedSet)
       {
           if(i->pose == pose)
           {
               return i;
           }
       }
       return nullptr;
    }

    double pathOptimizer::calculateDistance(Eigen::Vector2i& pose, Eigen::Vector2i& goal)
    {
        return sqrt(pow(pose(0) - goal(0), 2) + pow(pose(1) - goal(1), 2));
    }

    Node* pathOptimizer::minimalNode(vector<Node*>& openSet)
    {
        Node* min = openSet[0];
        auto it = openSet.begin();
        for(auto i = openSet.begin(); i != openSet.end(); i++)
        {
            if((*i)->cost < min->cost)
            {
                min = *i;
                it = i;
            }
        }
        openSet.erase(it);
        return min;
    }



    bool pathOptimizer::saftyFunc(Eigen::Vector2i& pose, Eigen::MatrixXi& occupancyMap)
    {
         if((pose(0)>=0 && pose(1) >= 0) && (pose(0) < occupancyMap.rows() && pose(1) < occupancyMap.cols()))
         {
             return true;
         }
         return false;
    } 

    vector<Eigen::Vector2i> pathOptimizer::pathConnector(vector<vector<Eigen::Vector2i>>& path, vector<Eigen::MatrixXi>& occupancyMapVector)
    {
        // ROS_INFO_STREAM("pathConnector");
        reverse(path.begin(), path.end());
        vector<Eigen::Vector2i> path_;
        // vector<Eigen::Vector2i> pathTemp = solveAStar(occupancyMapVector[0], path[0][0], path[1][0]);
        // for(auto it = pathTemp.begin(); it != pathTemp.end(); it++)
        // {
        //     path_.push_back(*it);
        // }
        for(auto i = 1; i != path.size(); i++ )
        {
            ROS_INFO_STREAM("path" << " " << path[i][0] << " " << path[i][1]);
            vector<Eigen::Vector2i> pathTemp = solveAStar(occupancyMapVector[i], path[i][0], path[i][1]);
            for(auto it = pathTemp.begin(); it != pathTemp.end(); it++)
            {
                // ROS_INFO_STREAM("###: " << (*it));
                path_.push_back(*it);
            }

        }
        // for(auto it = pathTemp.begin(); it != pathTemp.end(); it++)
        // {
        //         // ROS_INFO_STREAM("2: " << (*it));
        //     path_.push_back(*it);
        // }
        for(auto it = path_.begin(); it != path_.end(); it++)
        {
            ROS_INFO_STREAM("path: " << (*it));
        }
        return path_;

    }


}


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "path_optimizer");
//     ros::NodeHandle nh;
//     pathPlannings::pathOptimizer pathOptimizer(nh);
//     ros::spin();
//     return 0;
// }