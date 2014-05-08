#include <vector>
#include <string>
#include <utility>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <vector>
#include <utility>


#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <eigen_conversions/eigen_msg.h>


/*
    Terminology:
        base frame: frame relative to which transforms are computed for CapMap activities. Typically NOT the robot base or world frame. Instead, it is the base of the arm (usually the shoulder or shoulder parent link).

        configuration [of the robot arm]: a set of values for joint angles in the arm. Defined in joint space, not cartesian.

        configuration space [of the robot arm]: also known as joint space. The space of all possible joint angle value combinations for the robot arm.

        facing vector: a vector in the end effector frame which describes which direction the end effector faces. For example, this would be the normal to the palm plane, oriented from the back of the hand, through the hand, and out through the palm.

        [link] frame: a cartesian coordinate frame associated with a robot link. For example, the end effector frame is associated to the end effector.

        joint space [of the robot arm]: see configuration space.

        orientation [of a {link | the link's associated frame}]: the rotation between a base frame and the given link. Described by a rotation matrix. Specified by two vector pairs: 
                first pair: pointing vector (associated to the link frame) and its base frame correspondent, 
                second pair: facing vector (associated to the link frame) and its base frame correspondent. 
            The orientation transforms the link frame {pointing | facing} vector to its base frame correspondent.

        pointing vector: a vector in the end effector frame which describes which direction the end effector points to. For example, this would be a direction in the palm plane, parallel to the metacarpal bones, oriented from the wrist along the metacarpals.

        pose [of {the end effector | a link}]: position and orientation of a link in a cartesian space. Described by an affine matrix.

*/

namespace robokata
{

typedef struct
{
    /* Number of neighbors from CapMap to query pose (in 3D space defined by end effector translational position). */
    int neighborCount;
    /* Average end effector position for neighbors from CapMap. */
    Eigen::Vector3d neighborPositionAverage;
    /* Variance of pointing vector components for neighbors from CapMap. Computed componentwise: cross-variances not included. */
    Eigen::Vector3d pointingVectorVariance;
    /* Variance of facing vector components for neighbors from CapMap. Computed componentwise: cross-variances not included. */
    Eigen::Vector3d facingVectorVariance;
    /* Average of dot product between query pointing vector and neighbor from CapMap pointing vector. */
    double pointingDotProductAverage;
    /* Average of dot product between query facing vector and neighbor from CapMap facing vector. */
    double facingDotProductAverage;
}RegionCapDescription;

class RegionCostDescription
{
    public:
    /* Number of neighbors from CapMap to query joint angles (in joint space). */
    int neighborCount;
    /* Average vertex cost for neighbors from CapMap. */
    double costAverage;
    /* Center of mass for cost values (given as a point in joint space).*/
    robot_state::RobotState costCentroid;
    RegionCostDescription(robot_model::RobotModelConstPtr const&robotModelInit):
      costCentroid(robotModelInit)
    {
    };
    RegionCostDescription(RegionCostDescription const&orig):
      neighborCount(orig.neighborCount),
      costAverage(orig.costAverage),
      costCentroid(orig.costCentroid)
    {
    };
};

class CapMap
{
    public:
        CapMap(robot_model::RobotModelConstPtr const&robotModelInit);
        CapMap(CapMap const&orig);
        ~CapMap();
        
        CapMap &operator=(CapMap const &orig);

        /* Load arm poses from a file. Last link in the file is the end effector. Computes end effector poses relative to the arm base frame. Will then create the NearQuery structures for:
               cartesian translation (3D); used for getRegionCapDescription to obtain capability info around a point in work-space.
               cartesian rotation (represented as a 3x3 matrix). Used for getCapablePoses to obtain close orientations to a query from the CapMap.
               joint angles (DoF num equal to arm DoF num). Used for cost-related functions (costBump, getRegionCost) when searching for configurations near a query.
        */
        bool load(std::string const &fileName, std::string const &groupNameIni);

        bool isLoaded(void) const;

        std::string const&getGroupName(void) const;
        std::vector<std::string> const&getJointNames(void) const;
        std::vector<std::string> const&getLinkNames(void) const;
        robot_model::RobotModelPtr const&getRobotModel(void) const;

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
        RegionCapDescription getRegionCapDescription(Eigen::Affine3d const&eefPose, double radius, Eigen::Vector3d const&eefPointingVector, Eigen::Vector3d const&eefFacingVector) const; 

        /* Get poses from the CapMap where the robot can assume orientations close to the query.
           Inputs:
               eefPose: end effector query pose. Orientation neighbors to this pose will be sought. Pose must be relative to the arm base (typically the shoulder link frame, NOT the robot base or world frame!).
               candidates: number of candidates to return.
           Output:
               std::vector<Eigen::Affine3d>: vector of end effector poses which have orientation close to the query. Closer orientations are at the beginning of the vector.
        */
        void getCapablePoses(Eigen::Affine3d const&eefPose, int candidates, std::vector<int> &candidatePoses) const;

        /* Increase vertex costs around a configuration (defined in joint space, not cartesian space!). 
           Inputs:
               invalidJointAngles: arm configuration that was found invalid. Cost bump will be done relative to this configuration: vertices in the CapMap that are close to it in configuration space will receive higher cost increases.
               radius: controls the fall-off of the cost bump with distance from the invalidJointAngles. Higher radius means cost bump falls slower. In particular, a configuration at a distance of radius from invalidJointAngles receives half the cost given by amplitude.
               amplitude: controls the magnitude of the cost bump. In particular, a configuration exactly equal to invalidJointAngles receives this cost bump.
           Output:
               std::vector<Eigen::Affine3d>: vector of end effector poses which have orientation close to the query. Closer orientations are at the beginning of the vector.
        */
        void costBump(robot_state::RobotState const&invalidJointAngles, double radius, double amplitude);

        /* Reset all vertex costs to 0.
        */
        void resetCosts(void);

        /* Get information about costs from the CapMap that are near the query configuration.
           Inputs:
               queryJointAngles: query configuration. Neighbors in joint space will be sought.
               radius: parameter for a Near in Range query.
           Output:
               RegionCostDescription: contains statistics for neighbors of the query configuration from the CapMap. Includes:
                   average cost of neighbors from CapMap. This gives an estimate of the closeness of the query to previously detected obstacles. Higher average means obstacles are thought more likely to be present.
                   centroid of neighbor costs. This point, implicitly, gives a direction to move opposite to, so as to move away from the obstacles believed to be there.
        */
        RegionCostDescription getRegionCost(robot_state::RobotState const&queryJointAngles, double radius) const;

        double getVertexCost(int vertex) const
        {
            return costs[vertex];
        }
        void getRobotState(int vertex, robot_state::RobotState &state) const
        {
            double aux[10];
            for(int k = 0; k < joints.size(); k++)
            {
                aux[0] = jointAngles[vertex][k];
                state.setJointPositions(joints[k], aux);
            }
        }

        std::string getBaseLinkName(void) const
        {
          return links[0];
        }

    private:
    protected:
        robot_model::RobotModelPtr robotModel;
        std::string groupName;
        std::vector<std::string> joints;
        std::vector<std::string> links;
        std::vector<std::vector<double> > jointAngles;
        std::vector<Eigen::Affine3d> eefPoses;
        std::vector<double> costs;
        bool loaded;
        mutable Eigen::Vector3d workingPosition;
        mutable Eigen::Affine3d workingPose;
        mutable robot_state::RobotState workingConfiguration;

        /* NearQuery data structures and associated distance functions. The structures are defined over CapMap vertex indices, but will use whatever information from the vertices as appropiate (be that translation, rotation etc).
           To query for neighbors to an object not in the CapMap (as will be typical; we will want the CapMap stats at some random location), values of indices outside the CapMap graph refer to the temporary "working objects". These working objects will contain information about the pose we want capability information for, and run a near query around.
        */
        ompl::NearestNeighborsGNAT<int> workSpaceNearQuery;
        ompl::NearestNeighborsGNAT<int> orientationNearQuery;
        ompl::NearestNeighborsGNAT<int> jointSpaceNearQuery;

        /* Distance between two 3D positions.
        */
        static double workSpaceDistance(CapMap const &obj, int a, int b);
        /* Absolute value of the angle of a rotation needed to go from orientation a to orientation b.
        */
        static double orientationDistance(CapMap const &obj, int a, int b);
        /* Distance between joint angle values. Uses the robot model from moveit to handle angle wrap-around cases.
        */
        static double jointSpaceDistance(CapMap const &obj, int a, int b);

        static double averageDouble(std::vector<double> const &data);
        static double varianceDouble(std::vector<double> const &data);
        Eigen::Vector3d averageVector3d(std::vector<int> const &data) const;
        Eigen::Vector3d varianceVector3d(std::vector<int> const &data) const;
        static Eigen::Vector3d averageVector3d(std::vector<Eigen::Vector3d> const &data);
        static Eigen::Vector3d varianceVector3d(std::vector<Eigen::Vector3d> const &data);
};

}

