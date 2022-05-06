#include <optimal_path_forklift/pathPlanning.hpp>


namespace pathPlannings
{
    planningNode::planningNode(Eigen::Vector2i& pose, planningNode* parent):
    pose(pose), parent(parent)
    {

    }
    
    void planningNode::setVariables(Eigen::MatrixXi& occupancyMap, vector<pathNode*>& path)
    {   
        this->occupancyMap_.resize(occupancyMap.rows(), occupancyMap.cols());
        this->occupancyMap_ = occupancyMap;

        this->path.assign(path.begin(), path.end());
    }




    pathNode::pathNode(pathNode* parent, Eigen::Vector3i& target, Eigen::Vector2i& pose, Eigen::MatrixXi& occupancyMap, int& cost):
    parent_(parent), target_(target), pose_(pose), cost(cost), motion(motion)
    {
        setVariables(occupancyMap);
    }

    void pathNode::setVariables(Eigen::MatrixXi& occupancyMap)
    {
        occupancyMap_.resize(occupancyMap.rows(), occupancyMap.cols());
        occupancyMap_ = occupancyMap;
    }


    simplePlannings::simplePlannings(ros::NodeHandle& nh):
        nh_(nh)
    {
    }


    simplePlannings::~simplePlannings()
    {
    }

    void simplePlannings::setVariables(Eigen::MatrixXi& occupancyMap, Eigen::MatrixXi& goalMap, Eigen::Vector2i& initialPosition, Eigen::Vector3i& goalPosition)
    {
        occupancyMap_.resize(occupancyMap.rows(), occupancyMap.cols());
        occupancyMap_ = occupancyMap;

        motion_ = {Eigen::Vector2i(1,0), Eigen::Vector2i(0,1), Eigen::Vector2i(-1,0), Eigen::Vector2i(0,-1)};

        goalMap_.resize(goalMap.rows(), goalMap.cols());
        goalMap_ = goalMap;

        initialPosition_ = initialPosition;
        target_ = goalPosition;
    
    }

    void simplePlannings::planning()
    {
        ROS_INFO_STREAM("occupancyMap_: " << occupancyMap_);
    }

    int simplePlannings::calculateCost(Eigen::MatrixXi& occupancyMap)
    {
        int cost = 0;
        for(auto i=0; i < occupancyMap.rows(); i++)
        {
            for(auto j=0; j < occupancyMap.cols(); j++)
            {
                if(occupancyMap(i,j) > 0 && (i != target_(0) || j != target_(1)))
                {
                    cost += occupancyMap(i,j);
                }
            }
        }
        return cost;
    }

    int simplePlannings::calculateCostDistance(Eigen::MatrixXi& occupancyMap)
    {
        int cost = 0;
        for(auto i=0; i < occupancyMap.rows(); i++)
        {
            for(auto j=0; j < occupancyMap.cols(); j++)
            {
                if(occupancyMap(i,j) > 0)
                {
                    cost += (abs(i-target_(0)) + abs(j-target_(1))) ;
                }
            }
        }
        return cost;
    }

    int simplePlannings::calculateDistance(Eigen::Vector2i& pose, Eigen::Vector2i& target)
    {
        return abs(pose(1) - target(1)) + abs(pose(0) - target(0));
    }

    void simplePlannings::solvePlanning()
    {
        priority_queue<planningNode*, vector<planningNode*>, compPlanning> priorityPaths;
        Eigen::Vector2i pose(initialPosition_);
        Eigen::MatrixXi occupancyMap(occupancyMap_);
        
        planningNode* root = new planningNode(pose, NULL);
        root->setVariables(occupancyMap, parents_);
        root->cost = calculateCost(occupancyMap_);

        priorityPaths.push(root);
        int counter = 0;
        while(!priorityPaths.empty())
        {
            counter ++;
            if(counter == 100000)
            {
                break;
            }
            // ROS_INFO_STREAM("counter: " << counter);
            planningNode* currentNode = priorityPaths.top();
            // ROS_INFO_STREAM(currentNode->occupancyMap_);
            priorityPaths.pop();
            if(currentNode->cost == 0)
            {
                while(currentNode->parent != NULL)
                {
                    ROS_INFO_STREAM(currentNode->occupancyMap_);
                    Eigen::Vector2i initNonde = currentNode->pose;
                    Eigen::Vector2i targetNode = currentNode->poseTarget;
                    vector<Eigen::Vector2i> tempPath;
                    tempPath.push_back(initNonde);
                    tempPath.push_back(targetNode);
                    agentPath.push_back(tempPath);
                    currentNode = currentNode->parent;
                    agentOccupanyMap.push_back(currentNode->occupancyMap_);
                }
                ROS_INFO_STREAM(currentNode->occupancyMap_);
                agentOccupanyMap.push_back(currentNode->occupancyMap_);
                Eigen::Vector2i initNonde = currentNode->pose;
                vector<Eigen::Vector2i> tempPath;
                tempPath.push_back(initNonde);
                agentPath.push_back(tempPath);
                return;
            }
            // ROS_INFO_STREAM(currentNode->path.size());
            for(auto i = 0; i<currentNode->path.size(); i++)
            {
                for(auto j = 0; j < currentNode->path[i]->childrens_.size();j++)
                {
                    // ROS_INFO_STREAM("i: " << i << " j: " << j);
                    Eigen::Vector2i pose = currentNode->path[i]->pose_ - currentNode->path[i]->childrens_[j]->motion;
                    planningNode* node = new planningNode(pose, currentNode);
                    Eigen::MatrixXi occupancyMapNode(currentNode->occupancyMap_);
                    vector<pathNode*> pathNode = currentNode->path;
                    pathNode[i] = currentNode->path[i]->childrens_[j];
                    Eigen::Vector2i& materialPose = currentNode->path[i]->childrens_[j]->pose_;
                    Eigen::Vector2i& prevMaterialPose = currentNode->path[i]->pose_;
                    Eigen::Vector2i& nodeMotion = currentNode->path[i]->childrens_[j]->motion;
                    // ROS_INFO_STREAM(currentNode->path[i]->childrens_[j]->pose_(0) << " " << currentNode->path[i]->childrens_[j]->pose_(1));
                    int value = 0;
                    // ROS_INFO_STREAM("prevMaterialPose: " << prevMaterialPose << " materialPose: " << materialPose);
                    // ROS_INFO_STREAM("motion: " << nodeMotion);
                    for(auto k = prevMaterialPose; k!=materialPose; k+=nodeMotion)
                    {
                            value += currentNode->occupancyMap_(k(0),k(1));
                            occupancyMapNode(k(0),k(1)) = 0;
                            // ROS_INFO_STREAM("value:" << value);
                    }
                    occupancyMapNode(materialPose(0),materialPose(1)) = value + currentNode->occupancyMap_(materialPose(0),materialPose(1));
                    // ROS_INFO_STREAM("occupancyMapNode(materialPose(0),materialPose(1)): " << occupancyMapNode(materialPose(0),materialPose(1)));
                    node->setVariables(occupancyMapNode,pathNode);
                    Eigen::Vector2i prevPose = currentNode->pose;
                    
                    // ROS_INFO_STREAM("prevPose: " << prevPose);
                    // ROS_INFO_STREAM("pose: " << pose);
                    // ROS_INFO_STREAM("node cost: " << currentNode->path[i]->childrens_[j]->cost);
                    node->distance = calculateDistance(pose, prevPose) + currentNode->path[i]->childrens_[j]->cost + currentNode->distance;
                    node->price = calculateCostDistance(occupancyMapNode);
                    node->cost = calculateCost(occupancyMapNode);
                    node->poseTarget = currentNode->path[i]->childrens_[j]->pose_ - currentNode->path[i]->childrens_[j]->motion;
                    // ROS_INFO_STREAM("distance:" << node->distance << " cost: " <<  node->cost );
                    priorityPaths.push(node);
                    // ROS_INFO_STREAM("i: " << i << " j: " << j);
                    
                }
               
            }
            

        }

    }

    void simplePlannings::pathNodesGenerator(Eigen::Vector3i& target)
    {
        target_ = target;
        ROS_INFO_STREAM("pathNodesGenerator");
        // create list of material locations
        for(auto i=0;i<occupancyMap_.rows();i++)
        {
            for(auto j=0;j<occupancyMap_.cols();j++)
            {
                if(occupancyMap_(i,j)>=1)
                {
                    Eigen::Vector2i materialPose_(i,j);
                    // ROS_INFO_STREAM("materialPose_: " << materialPose_(0) << " " << materialPose_(1));
                    //Without reference to obstacles
                    int cost = 0;
                    pathNode* node =  new pathNode(NULL, target, materialPose_, occupancyMap_, cost);
                    Eigen::MatrixXi occupancyMap = occupancyMap_;
                    node = pathNodeGenerator(node, target_, materialPose_ ,occupancyMap);
                    parents_.push_back(node);
                    ROS_INFO_STREAM("parents_: " << parents_.size());

                }

            }
        }
    
    }

    bool simplePlannings::limitTest(Eigen::Vector2i& checkPosition, Eigen::MatrixXi& occupancyMap)
    {
        if(checkPosition(0)>=0 && checkPosition(1)>=0 && checkPosition(0)<occupancyMap.rows() && checkPosition(1)<occupancyMap.cols())
        {
            if(occupancyMap(checkPosition(0),checkPosition(1))==0)
            {
                return true;
            }
        }
        return false;
    }

    pathNode* simplePlannings::pathNodeGenerator(pathNode* parent, Eigen::Vector3i& target, Eigen::Vector2i& materialPose , Eigen::MatrixXi& occupancyMap)
    {
        ROS_INFO_STREAM("pathNodeGenerator");
        for (auto k=0;k<motion_.size();k++)
            {
                // ROS_INFO_STREAM("motion_: " << motion_[k]);
                Eigen::Vector2i checkPosition = materialPose - motion_[k];
                ROS_INFO_STREAM("motion_[k].dot(target.head(2)-materialPose): " << motion_[k].dot(target.head(2)-materialPose));
                    if(limitTest(checkPosition, occupancyMap) && motion_[k].dot(target.head(2)-materialPose) > 0)
                    {
                        int motionCost = stepPlanning(occupancyMap, motion_[k], materialPose, target);
                        ROS_INFO_STREAM("motionCost: " << motionCost);
                        if (motionCost > 0)
                        {
                            Eigen::Vector2i materialPoseNew = materialPose + motion_[k] * motionCost;
                            Eigen::MatrixXi newOccupancyMap = occupancyMap;
                            int value = 0;
                            for(auto z = materialPose; z!=materialPoseNew; z += motion_[k])
                            {
                                    value += newOccupancyMap(z(0),z(1));
                                    newOccupancyMap(z(0),z(1)) = 0;
                                    // ROS_INFO_STREAM("value:" << value);
                            }
                            newOccupancyMap(materialPoseNew(0),materialPoseNew(1)) = value + newOccupancyMap(materialPoseNew(0),materialPoseNew(1));
                            if((target.head(2)-materialPoseNew).norm() == 0)
                            {
                                pathNode* node =  new pathNode(parent, target, materialPoseNew, newOccupancyMap, motionCost);
                                node->motion = motion_[k];
                                parent->childrens_.push_back(node);
                                return parent;
                                ROS_INFO_STREAM(newOccupancyMap);

                            }
                            pathNode* node = new pathNode(parent, target, materialPoseNew, newOccupancyMap, motionCost);
                            node->motion = motion_[k];
                            node = pathNodeGenerator(node, target, materialPoseNew, newOccupancyMap);
                            parent->childrens_.push_back(node);
                            ROS_INFO_STREAM(newOccupancyMap);
                        }
                    }
            }

        return parent;
    }

    int simplePlannings::stepPlanning(Eigen::MatrixXi& occupancyMap, Eigen::Vector2i& motion, Eigen::Vector2i& materialPose, Eigen::Vector3i& target)
    {
        int cost = 0;
        if(norm(motion(0)) > 0)
        {
            
            if(occupancyMap(materialPose(0),materialPose(1)) == target(2))
            {
                
                return abs(target(0)-materialPose(0));
            }
            Eigen::VectorXi occupancyVector = occupancyMap.rowwise().sum();
            // ROS_INFO_STREAM("occupancyVector: " << occupancyVector << " row");
            for(auto i = materialPose(0)+motion(0);; i += motion(0))
            {
                if(i >= occupancyVector.size() || i<0)
                {
                    cost = 0;
                    break;
                }
                cost++;
                // ROS_INFO_STREAM("i: " << i);
                if(occupancyVector(i) != 0)
                {
                    break;
                }
                
            }
        }
        else
        {
            if(occupancyMap(materialPose(0),materialPose(1))== target(2))
            {
                return abs(target(1)-materialPose(1));
            }
            Eigen::VectorXi occupancyVector = occupancyMap.colwise().sum();
            // ROS_INFO_STREAM("occupancyVector: " << occupancyVector);
            for(auto i = materialPose(1)+motion(1);  ; i+= motion(1))
            {
                if(i >= occupancyVector.size() || i<0)
                {
                    cost = 0;
                    break;
                }
                cost++;
                // ROS_INFO_STREAM("i: " << i);
                if(occupancyVector(i) != 0)
                {
                    break;
                }
                
            }
        }
        return cost;
        

    }


}




