#include <optimal_path_forklift/pathPlanning.hpp>


namespace pathPlannings
{
    
    materialNode::materialNode(Eigen::Vector2i& location):
    location(location)
    {

    }


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




    pathNode::pathNode(pathNode* parent, Eigen::Vector2i& pose):
    parent_(parent), pose_(pose)
    {
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
        // ROS_INFO_STREAM("occupancyMap_: " << occupancyMap_);
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
        
        planningNode* root = new planningNode(initialPosition_, NULL);
        root->setVariables(occupancyMap, parentNode_->childrens_);
        root->cost = calculateCost(occupancyMap_);

        priorityPaths.push(root);
        int counter = 0;
        while(!priorityPaths.empty())
        {
            counter ++;
            if(counter == 1000)
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
                    // ROS_INFO_STREAM(currentNode->occupancyMap_);
                    Eigen::Vector2i initNonde = currentNode->pose;
                    Eigen::Vector2i targetNode = currentNode->poseTarget;
                    vector<Eigen::Vector2i> tempPath;
                    tempPath.push_back(initNonde);
                    tempPath.push_back(targetNode);
                    agentPath.push_back(tempPath);
                    currentNode = currentNode->parent;
                    agentOccupanyMap.push_back(currentNode->occupancyMap_);
                }
                // ROS_INFO_STREAM(currentNode->occupancyMap_);
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
                    Eigen::Vector2i pose = currentNode->path[i]->pose_;
                    planningNode* node = new planningNode(pose, currentNode);
                    Eigen::MatrixXi occupancyMapNode(currentNode->occupancyMap_);
                    vector<pathNode*> pathNode = currentNode->path;
                    pathNode[i] = currentNode->path[i]->childrens_[j];
                    Eigen::Vector2i& materialPose = currentNode->path[i]->childrens_[j]->pose_;
                    Eigen::Vector2i& prevMaterialPose = currentNode->path[i]->pose_;
                    int value = 0;
                    occupancyMapNode(materialPose(0),materialPose(1)) += occupancyMapNode(prevMaterialPose(0),prevMaterialPose(1));
                    occupancyMapNode(prevMaterialPose(0),prevMaterialPose(1)) = 0;
                    node->setVariables(occupancyMapNode,pathNode);
                    Eigen::Vector2i prevPose = currentNode->pose;
                    node->distance = calculateDistance(pose, prevPose) + currentNode->distance;
                    node->price = calculateCostDistance(occupancyMapNode);
                    node->cost = calculateCost(occupancyMapNode);
                    node->poseTarget = currentNode->path[i]->childrens_[j]->pose_;
                    priorityPaths.push(node);
                    
                }
               
            }
            

        }

    }

    void simplePlannings::pathNodesGenerator(int& edges, Eigen::Vector3i& target)
    {
        target_ = target;
        // ROS_INFO_STREAM("pathNodesGenerator");
        // create list of material locations
        int cost = 0;
        pathNode* node =  new pathNode(NULL, initialPosition_);
        for(auto i=0;i<occupancyMap_.rows();i++)
        {
            for(auto j=0;j<occupancyMap_.cols();j++)
            {
                if(occupancyMap_(i,j)>=1)
                {
                    Eigen::Vector2i location(i,j);
                    materialNode* mNode =  new materialNode(location);
                    materials_.push_back(mNode);

                }

            }
        }
        parentNode_ = pathNodeGenerator(node, target_, initialPosition_ ,materials_, edges);
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

    pathNode* simplePlannings::pathNodeGenerator(pathNode* parent, Eigen::Vector3i& target, Eigen::Vector2i& materialPose , vector<materialNode*> materials, int& edges)
    {
        // ROS_INFO_STREAM(parent->pose_(0) << " " << parent->pose_(1));
        for (auto k=0;k<edges && k < materials.size();k++)
            {
                vector<materialNode*> refMaterials = materials;
                auto v = refMaterials.begin();
                Eigen::Vector2i minPosition = (*v)->location;
                int minDis = calculateDistance(materialPose,(*v)->location);
                vector<materialNode*>::iterator it = v;
                for (; v != refMaterials.end(); ++v)
                    {
                        if(calculateDistance(materialPose,(*v)->location) < minDis)
                        {
                            // ROS_INFO_STREAM((*v)->location);
                            minPosition = (*v)->location;
                            minDis = calculateDistance(materialPose,(*v)->location);
                            it = v;
                        }
                    }
                pathNode* node =  new pathNode(parent, minPosition);  
                
                refMaterials.erase(it,it+1);
                // ROS_INFO_STREAM("%%%%" << parent->pose_(0) << " " << parent->pose_(1));
                parent->childrens_.push_back(pathNodeGenerator(node, target, minPosition, refMaterials, edges));
            }
        if(materials.size()==0)
        {
            Eigen::Vector2i targetPose = target.head(2);
            pathNode* node =  new pathNode(parent, targetPose);
            parent->childrens_.push_back(node);
        }
        
        return parent;
    }


}




