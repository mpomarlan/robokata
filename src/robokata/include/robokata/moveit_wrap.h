#ifndef __ROBOKATA_MOVEIT_WRAP__

#define __ROBOKATA_MOVEIT_WRAP__

#include <ros/common.h>

#if ROS_VERSION_MINIMUM(1, 10, 2)
    #define MOVEIT_SETJOINTPOSITIONS(state, name, position) (state.setJointPositions(name, position))

    #define MOVEIT_GETGLOBALLINKTRANSFORM(state, name) (state.getGlobalLinkTransform(name))

    #define MOVEIT_ROBOTMODEL_NAMESPACE moveit::core
#else
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/link_state.h>
    #define MOVEIT_SETJOINTPOSITIONS(state, name, position) (state.setStateValues(std::vector<std::string>(1, name), std::vector<double>(1, *position)))

    #define MOVEIT_GETGLOBALLINKTRANSFORM(state, name) (state.getLinkState(name)->getGlobalLinkTransform())

    #define MOVEIT_ROBOTMODEL_NAMESPACE robot_model
#endif

#endif

