#include <ros/ros.h>
#include <stdio.h>
#include <cmath>

#include <robokata/moveit_wrap.h>

#include <robokata/CapMap.h>
#include <map>

robokata::CapMap::CapMap(robot_model::RobotModelConstPtr const&robotModelInit):
    robotModel(new robot_model::RobotModel(*robotModelInit.get())),
    groupName(""),
    joints(),
    links(),
    jointAngles(),
    eefPoses(),
    costs(),
    loaded(false),
    workingPosition(),
    workingPose(),
    workingConfiguration(robotModel),
    workSpaceNearQuery(),
    orientationNearQuery(),
    jointSpaceNearQuery()
{
    workSpaceNearQuery.setDistanceFunction(boost::bind(workSpaceDistance, this, _1, _2));
    orientationNearQuery.setDistanceFunction(boost::bind(orientationDistance, this, _1, _2));
    jointSpaceNearQuery.setDistanceFunction(boost::bind(jointSpaceDistance, this, _1, _2));
}
robokata::CapMap::CapMap(CapMap const&orig):
    robotModel(new robot_model::RobotModel(*orig.robotModel.get())),
    groupName(orig.groupName),
    joints(orig.joints),
    links(orig.links),
    jointAngles(orig.jointAngles),
    eefPoses(orig.eefPoses),
    costs(orig.costs),
    loaded(orig.loaded),
    workingPosition(orig.workingPosition),
    workingPose(orig.workingPose),
    workingConfiguration(orig.workingConfiguration),
    workSpaceNearQuery(),
    orientationNearQuery(),
    jointSpaceNearQuery()
{
    workSpaceNearQuery.setDistanceFunction(boost::bind(workSpaceDistance, this, _1, _2));
    orientationNearQuery.setDistanceFunction(boost::bind(orientationDistance, this, _1, _2));
    jointSpaceNearQuery.setDistanceFunction(boost::bind(jointSpaceDistance, this, _1, _2));

    std::vector<int> aux;
    aux.clear(); aux.reserve(eefPoses.size());
    for(int k = 0; k < eefPoses.size(); k++)
        aux.push_back(k);

    if(aux.size())
    {
        workSpaceNearQuery.add(aux);
        orientationNearQuery.add(aux);
        jointSpaceNearQuery.add(aux);
    }
}
robokata::CapMap::~CapMap()
{
}

robokata::CapMap & robokata::CapMap::operator=(robokata::CapMap const &orig)
{
    costs = orig.costs;
    eefPoses = orig.eefPoses;
    groupName = orig.groupName;
    jointAngles = orig.jointAngles;
    joints = orig.joints;
    links = orig.links;
    loaded = orig.loaded;
    workingPosition = orig.workingPosition;
    workingPose = orig.workingPose;
    workingConfiguration = orig.workingConfiguration;
    robotModel.reset(new robot_model::RobotModel(*orig.robotModel.get()));

    workSpaceNearQuery.clear();
    orientationNearQuery.clear();
    jointSpaceNearQuery.clear();
    
    std::vector<int> aux;
    aux.clear(); aux.reserve(eefPoses.size());
    for(int k = 0; k < eefPoses.size(); k++)
        aux.push_back(k);

    if(aux.size())
    {
        workSpaceNearQuery.add(aux);
        orientationNearQuery.add(aux);
        jointSpaceNearQuery.add(aux);
    }

    return(*this);
}

/* Load arm poses from a file. Last link in the file is the end effector. Computes end effector poses relative to the arm base frame. Will then create the NearQuery structures for:
       cartesian translation (3D); used for getRegionCapDescription to obtain capability info around a point in work-space.
       cartesian rotation (represented as a 6D space of two 3D vectors, but some interdepencies between components are implicit). Used for getCapablePoses to obtain close orientations to a query from the CapMap.
       joint angles (DoF num equal to arm DoF num). Used for cost-related functions (costBump, getRegionCost) when searching for configurations near a query.
*/

bool robokata::CapMap::load(std::string const &fileName, std::string const &groupNameIni)
{
    FILE *data = fopen(fileName.c_str(), "rt");
    if(!data)
    {
        std::string warning("File not found: ");
        ROS_WARN((warning+fileName).c_str());
        return false;
    }
    groupName = groupNameIni;
    int jointCount, stateCount;
    fscanf(data, "joints %d states %d\n", &jointCount, &stateCount);
    if((0 == stateCount) || (0 == jointCount))
    {
        loaded = false;
        return false;
    }
    joints.clear(); joints.resize(jointCount);
    links.clear(); links.resize(jointCount);
    char name[10000];
    for(int k = 0; k < jointCount; k++)
    {
        fscanf(data, "%s\n", name);
        joints[k] = name;
    }
    for(int k = 0; k < jointCount; k++)
    {
        fscanf(data, "%s\n", name);
        links[k] = name;
    }

    robot_state::RobotState workStateCapMap(robotModel);
    workStateCapMap.setToDefaultValues();
    jointAngles.clear(); jointAngles.resize(stateCount);
    eefPoses.clear(); eefPoses.resize(stateCount);
    costs.clear(); costs.resize(stateCount); resetCosts();

    Eigen::Affine3d basePose = MOVEIT_GETGLOBALLINKTRANSFORM(workStateCapMap, links[0]);


    for(int k = 0; k < stateCount; k++)
    {
        double aux[10];
        jointAngles[k].clear(); jointAngles[k].resize(jointCount);
        for(int j = 0; j < jointCount; j++)
        {
            fscanf(data, "%lf", aux);
            jointAngles[k][j] = (aux[0]);
            MOVEIT_SETJOINTPOSITIONS(workStateCapMap, joints[j], aux);
        }
        eefPoses[k] = basePose.inverse()*MOVEIT_GETGLOBALLINKTRANSFORM(workStateCapMap, links[jointCount-1]);
    }

    workingPosition = eefPoses[0].translation();
    workingPose = eefPoses[0];
    double auxD[10];
    for(int k = 0; k < joints.size(); k++)
    {
        auxD[0] = jointAngles[0][k];

        MOVEIT_SETJOINTPOSITIONS(workingConfiguration, joints[k], auxD);
    }
    
    workSpaceNearQuery.clear();
    orientationNearQuery.clear();
    jointSpaceNearQuery.clear();
    
    std::vector<int> aux;
    aux.clear(); aux.reserve(eefPoses.size());
    for(int k = 0; k < eefPoses.size(); k++)
        aux.push_back(k);

    workSpaceNearQuery.add(aux);
    orientationNearQuery.add(aux);
    jointSpaceNearQuery.add(aux);

    fclose(data);
    loaded = true;
    return true;
}

bool robokata::CapMap::isLoaded(void) const
{
    return loaded;
}

std::string const& robokata::CapMap::getGroupName(void) const
{
    return groupName;
}
std::vector<std::string> const& robokata::CapMap::getJointNames(void) const
{
    return joints;
}
std::vector<std::string> const& robokata::CapMap::getLinkNames(void) const
{
    return links;
}
robot_model::RobotModelPtr const& robokata::CapMap::getRobotModel(void) const
{
    return robotModel;
}

/* Get information about poses from the CapMap that are near the query pose.
   Inputs:
       eefPose: end effector query pose. Translational neighbors to this pose will be sought. Pose must be relative to the arm base (typically the shoulder link frame, NOT the robot base or world frame!).
       radius: parameter for a Near in Range query.
       eefPointingVector: a vector in the end effector frame which describes which direction the end effector points to. For example, this would be a direction in the palm plane, parallel to the metacarpal bones, oriented from the wrist along the metacarpals.
       eefFacingVector: a vector in the end effector frame which describes which direction the end effector faces. For example, this would be the normal to the palm plane, oriented from the back of the hand, through the hand, and out through the palm.
   Output:
       RegionCapDescription: contains statistics for neighbors of the query pose from the CapMap. Includes:
           average end effector positions in neighbors from CapMap. This gives an estimate of an offset of the query pose from the reachable space, and the direction of the offset. Averages close to 0 mean the query is close or inside the reachable region (in terms of desired position).
           variance in pointing and facing vectors in neighbors from CapMap. This gives an estimate of the robot´s ability to orient the end effector near the query pose. Higher variance is better.
           average dot product between query {pointing | facing} vector and neighbors' {pointing | facing} vectors. This gives an estimate of whether the desired orientation is opposite to what the robot can easily produce at the query position. Negative values indicate this might be the case.
*/
robokata::RegionCapDescription robokata::CapMap::getRegionCapDescription(Eigen::Affine3d const&eefPose, double radius, Eigen::Vector3d const&eefPointingVector, Eigen::Vector3d const&eefFacingVector) const
{
    RegionCapDescription retq;
    retq.neighborCount = 0;
    if(!loaded)
        return retq;
    std::vector<int> neighbors;
    neighbors.clear();
    workingPosition = eefPose.translation();
    workSpaceNearQuery.nearestR(eefPoses.size(), radius, neighbors);
    retq.neighborCount = neighbors.size();
    retq.neighborPositionAverage = averageVector3d(neighbors);
    std::vector<Eigen::Vector3d> orientationVectors;
    std::vector<double> dots;
    Eigen::Vector3d reqOrientationVector;
    orientationVectors.clear(); orientationVectors.resize(neighbors.size());
    dots.clear(); dots.resize(neighbors.size());
    reqOrientationVector = eefPose.rotation()*eefPointingVector;
    for(int k = 0; k < neighbors.size(); k++)
    {
        orientationVectors[k] = eefPoses[k].rotation()*eefPointingVector;
        dots[k] = reqOrientationVector(0)*orientationVectors[k](0) + reqOrientationVector(1)*orientationVectors[k](1) + reqOrientationVector(2)*orientationVectors[k](2);
    }
    retq.pointingVectorVariance = varianceVector3d(orientationVectors);
    retq.pointingDotProductAverage = averageDouble(dots);
    reqOrientationVector = eefPose.rotation()*eefFacingVector;
    for(int k = 0; k < neighbors.size(); k++)
    {
        orientationVectors[k] = eefPoses[k].rotation()*eefFacingVector;
        dots[k] = reqOrientationVector(0)*orientationVectors[k](0) + reqOrientationVector(1)*orientationVectors[k](1) + reqOrientationVector(2)*orientationVectors[k](2);
    }
    retq.facingVectorVariance = varianceVector3d(orientationVectors);
    retq.facingDotProductAverage = averageDouble(dots);
    return retq;
}

/* Get poses from the CapMap where the robot can assume orientations close to the query.
   Inputs:
       eefPose: end effector query pose. Orientation neighbors to this pose will be sought. Pose must be relative to the arm base (typically the shoulder link frame, NOT the robot base or world frame!).
       candidates: number of candidates to return.
   Output:
       std::vector<Eigen::Affine3d>: vector of end effector poses which have orientation close to the query. Closer orientations are at the beginning of the vector.
*/
void robokata::CapMap::getCapablePoses(Eigen::Affine3d const&eefPose, int candidates, std::vector<int> &candidatePoses) const
{
    candidatePoses.clear(); candidatePoses.reserve(candidates);
    if(!loaded)
        return;
    std::vector<int> neighbors;
    neighbors.clear(); neighbors.reserve(candidates);
    workingPose = eefPose;
    orientationNearQuery.nearestK(eefPoses.size(), candidates, neighbors);
    int maxK = neighbors.size();
    for(int k = 0; k < maxK; k++)
    {
        candidatePoses.push_back(neighbors[k]);
    }
}

/* Increase vertex costs around a configuration (defined in joint space, not cartesian space!). 
   Inputs:
       invalidJointAngles: arm configuration that was found invalid. Cost bump will be done relative to this configuration: vertices in the CapMap that are close to it in configuration space will receive higher cost increases.
       radius: controls the fall-off of the cost bump with distance from the invalidJointAngles. Higher radius means cost bump falls slower. In particular, a configuration at a distance of radius from invalidJointAngles receives half the cost given by amplitude.
       amplitude: controls the magnitude of the cost bump. In particular, a configuration exactly equal to invalidJointAngles receives this cost bump.
   Output:
       std::vector<Eigen::Affine3d>: vector of end effector poses which have orientation close to the query. Closer orientations are at the beginning of the vector.
*/
void robokata::CapMap::costBump(robot_state::RobotState const&invalidJointAngles, double radius, double amplitude)
{
    if(!loaded)
        return;
    int maxK = costs.size();
    robot_state::RobotState workStateCapMap(robotModel);
    workStateCapMap.setToDefaultValues();
    robot_state::RobotState workStateQuery(workStateCapMap);

    workStateQuery = invalidJointAngles;
    /*
        cost bump formula is amplitude/(1 + (distance/radius)^2) so will square the radius and leave distance squared as well
    */
    radius = radius*radius;
    for(int k = 0; k < maxK; k++)
    {
        int maxJ = joints.size();
        double aux[1];
        for(int j = 0; j < maxJ; j++)
        {
            aux[0] = jointAngles[k][j];
            MOVEIT_SETJOINTPOSITIONS(workStateCapMap, joints[j], aux);
        }
        double distance = workStateQuery.distance(workStateCapMap);
        distance = distance*distance;
        costs[k] += amplitude/(1 + distance/radius);
    }
}

/* Reset all vertex costs to 0.
*/
void robokata::CapMap::resetCosts(void)
{
    int maxK = costs.size();
    for(int k = 0; k < maxK; k++)
        costs[k] = 0.0;
}

/* Get information about costs from the CapMap that are near the query configuration.
   Inputs:
       queryJointAngles: query configuration. Neighbors in joint space will be sought.
       radius: parameter for a Near in Range query.
   Output:
       RegionCostDescription: contains statistics for neighbors of the query configuration from the CapMap. Includes:
           average cost of neighbors from CapMap. This gives an estimate of the closeness of the query to previously detected obstacles. Higher average means obstacles are thought more likely to be present.
           centroid of neighbor costs. This point, implicitly, gives a direction to move opposite to, so as to move away from the obstacles believed to be there.
        */
robokata::RegionCostDescription robokata::CapMap::getRegionCost(robot_state::RobotState const&queryJointAngles, double radius) const
{
    RegionCostDescription retq(robotModel);
    retq.neighborCount = 0;
    if(!loaded)
        return retq;
    std::vector<int> neighbors;
    neighbors.clear();
    workingConfiguration = queryJointAngles;
    jointSpaceNearQuery.nearestR(eefPoses.size(), radius, neighbors);
    retq.neighborCount = neighbors.size();

    std::vector<double> neighborCosts;
    neighborCosts.clear(); neighborCosts.resize(neighbors.size());
    for(int k = 0; k < neighbors.size(); k++)
        neighborCosts[k] = costs[neighbors[k]];
    retq.costAverage = averageDouble(neighborCosts);

    int maxIndex = 0;
    for(int k = 1; k < neighbors.size(); k++)
        if(costs[neighbors[maxIndex]] < costs[neighbors[k]])
            maxIndex = k;
    
    robot_state::RobotState maxCostState(queryJointAngles);
    double aux[10];
    for(int k = 0; k < joints.size(); k++)
    {
        aux[0] = jointAngles[maxIndex][k];
        MOVEIT_SETJOINTPOSITIONS(maxCostState, joints[k], aux);
    }
    retq.costCentroid = maxCostState;

    return retq;
}

/* Distance between two 3D positions.
*/
double robokata::CapMap::workSpaceDistance(robokata::CapMap const *obj, int a, int b)
{
    Eigen::Vector3d aV, bV;
    if(obj->eefPoses.size() <= a)
        aV = obj->workingPosition;
    else
        aV = obj->eefPoses[a].translation();
    if(obj->eefPoses.size() <= b)
        bV = obj->workingPosition;
    else
        bV = obj->eefPoses[b].translation();
    double dx, dy, dz;
    dx = aV(0) - bV(0);
    dy = aV(1) - bV(1);
    dz = aV(2) - bV(2);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
/* Absolute value of the angle of a rotation needed to go from orientation a to orientation b.
*/
double robokata::CapMap::orientationDistance(robokata::CapMap const *obj, int a, int b)
{
    Eigen::Matrix3d aV, bV, R;
    if(obj->eefPoses.size() <= a)
        aV = obj->workingPose.rotation();
    else
        aV = obj->eefPoses[a].rotation();
    if(obj->eefPoses.size() <= b)
        bV = obj->workingPose.rotation();
    else
        bV = obj->eefPoses[b].rotation();
    R = aV*bV.transpose();
    double angle = std::acos(0.5*(R(0,0) + R(1,1) + R(2, 2) - 1.0));
    if(angle < 0.0)
        angle = -angle;
    return angle;
}
/* Distance between joint angle values. Uses the robot model from moveit to handle angle wrap-around cases.
*/
double robokata::CapMap::jointSpaceDistance(robokata::CapMap const *obj, int a, int b)
{
    robot_state::RobotState aS(obj->robotModel), bS(obj->robotModel);
    if(obj->eefPoses.size() <= a)
    {
        aS = obj->workingConfiguration;
    }
    else
    {
        double aux[10];
        for(int k = 0; k < obj->joints.size(); k++)
        {
            aux[0] = obj->jointAngles[a][k];
            MOVEIT_SETJOINTPOSITIONS(aS, obj->joints[k], aux);
        }
    }
    if(obj->eefPoses.size() <= b)
    {
        bS = obj->workingConfiguration;
    }
    else
    {
        double aux[10];
        for(int k = 0; k < obj->joints.size(); k++)
        {
            aux[0] = obj->jointAngles[b][k];
            MOVEIT_SETJOINTPOSITIONS(bS, obj->joints[k], aux);
        }
    }
    return aS.distance(bS);
}

/* Distance between two 3D positions.
*/
double robokata::CapMap::workSpaceODistance(int const &a, int const &b) const
{
    Eigen::Vector3d aV, bV;
    if(eefPoses.size() <= a)
        aV = workingPosition;
    else
        aV = eefPoses[a].translation();
    if(eefPoses.size() <= b)
        bV = workingPosition;
    else
        bV = eefPoses[b].translation();
    double dx, dy, dz;
    dx = aV(0) - bV(0);
    dy = aV(1) - bV(1);
    dz = aV(2) - bV(2);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
/* Absolute value of the angle of a rotation needed to go from orientation a to orientation b.
*/
double robokata::CapMap::orientationODistance(int const &a, int const &b) const
{
    Eigen::Matrix3d aV, bV, R;
    if(eefPoses.size() <= a)
        aV = workingPose.rotation();
    else
        aV = eefPoses[a].rotation();
    if(eefPoses.size() <= b)
        bV = workingPose.rotation();
    else
        bV = eefPoses[b].rotation();
    R = aV*bV.transpose();
    double angle = std::acos(0.5*(R(0,0) + R(1,1) + R(2, 2) - 1.0));
    if(angle < 0.0)
        angle = -angle;
    return angle;
}
/* Distance between joint angle values. Uses the robot model from moveit to handle angle wrap-around cases.
*/
double robokata::CapMap::jointSpaceODistance(int const &a, int const &b) const
{
    robot_state::RobotState aS(robotModel), bS(robotModel);
    if(eefPoses.size() <= a)
    {
        aS = workingConfiguration;
    }
    else
    {
        double aux[10];
        for(int k = 0; k < joints.size(); k++)
        {
            aux[0] = jointAngles[a][k];
            MOVEIT_SETJOINTPOSITIONS(aS, joints[k], aux);
        }
    }
    if(eefPoses.size() <= b)
    {
        bS = workingConfiguration;
    }
    else
    {
        double aux[10];
        for(int k = 0; k < joints.size(); k++)
        {
            aux[0] = jointAngles[b][k];
            MOVEIT_SETJOINTPOSITIONS(bS, joints[k], aux);
        }
    }
    return aS.distance(bS);
}

double robokata::CapMap::averageDouble(std::vector<double> const &data)
{
    double retq = 0.0;
    int maxK = data.size();
    if(0 == maxK)
        return 0.0;
    for(int k = 0; k < maxK; k++)
        retq += data[k];
    return (retq/(1.0*maxK));
}
double robokata::CapMap::varianceDouble(std::vector<double> const &data)
{
    double avg = averageDouble(data);
    double retq = 0.0;
    int maxK = data.size();
    if(maxK < 2)
        return 0.0;
    for(int k = 0; k < maxK; k++)
        retq += (data[k] - avg)*(data[k] - avg);
    return retq/(1.0*(maxK-1));
}
Eigen::Vector3d robokata::CapMap::averageVector3d(std::vector<int> const &data) const
{
    Eigen::Vector3d retq(0.0, 0.0, 0.0);
    int maxK = data.size();
    if(0 == maxK)
        return retq;
    for(int k = 0; k < maxK; k++)
        retq += eefPoses[data[k]].translation();
    return (retq/(1.0*maxK));
}
Eigen::Vector3d robokata::CapMap::varianceVector3d(std::vector<int> const &data) const
{
    Eigen::Vector3d avg = averageVector3d(data);
    Eigen::Vector3d retq(0.0, 0.0, 0.0);
    int maxK = data.size();
    if(maxK < 2)
        return retq;
    for(int k = 0; k < maxK; k++)
        retq += Eigen::Vector3d((eefPoses[data[k]].translation()(0) - avg(0))*(eefPoses[data[k]].translation()(0) - avg(0)), 
                                (eefPoses[data[k]].translation()(1) - avg(1))*(eefPoses[data[k]].translation()(1) - avg(1)), 
                                (eefPoses[data[k]].translation()(2) - avg(2))*(eefPoses[data[k]].translation()(2) - avg(2)));
    return retq/(1.0*(maxK-1));
}
Eigen::Vector3d robokata::CapMap::averageVector3d(std::vector<Eigen::Vector3d> const &data)
{
    Eigen::Vector3d retq(0.0, 0.0, 0.0);
    int maxK = data.size();
    if(0 == maxK)
        return retq;
    for(int k = 0; k < maxK; k++)
        retq += data[k];
    return (retq/(1.0*maxK));
}
Eigen::Vector3d robokata::CapMap::varianceVector3d(std::vector<Eigen::Vector3d> const &data)
{
    Eigen::Vector3d avg = averageVector3d(data);
    Eigen::Vector3d retq(0.0, 0.0, 0.0);
    int maxK = data.size();
    if(maxK < 2)
        return retq;
    for(int k = 0; k < maxK; k++)
        retq += Eigen::Vector3d((data[k](0) - avg(0))*(data[k](0) - avg(0)), (data[k](1) - avg(1))*(data[k](1) - avg(1)), (data[k](2) - avg(2))*(data[k](2) - avg(2)));
    return retq/(1.0*(maxK-1));
}


void robokata::CapMap::getRobotState(int vertex, robot_state::RobotState &state) const
{
    double aux[10];
    for(int k = 0; k < joints.size(); k++)
    {
        aux[0] = jointAngles[vertex][k];
        MOVEIT_SETJOINTPOSITIONS(state, joints[k], aux);
    }
}

