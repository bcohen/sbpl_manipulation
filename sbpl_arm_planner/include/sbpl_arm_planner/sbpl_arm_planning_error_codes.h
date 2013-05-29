
#ifndef _SBPL_DEBUG_CODES_
#define _SBPL_DEBUG_CODES_

namespace sbpl_arm_planner
{
  enum DebugCode 
  {
    SUCCESS,
    COLLISION_BETWEEN_ARMS,
    RIGHT_ARM_IN_COLLISION,
    LEFT_ARM_IN_COLLISION,
    ATTACHED_OBJECT_IN_COLLISION,
    RIGHT_IK_FAIL_IK_SEARCH_SUCCESS,
    RIGHT_IK_FAIL_IK_SEARCH_FAIL, 
    LEFT_IK_FAIL_IK_SEARCH_SUCCESS,
    LEFT_IK_FAIL_IK_SEARCH_FAIL,
    INVALID_RIGHT_SHOULDER_PAN_ANGLE,
    INVALID_RIGHT_SHOULDER_PITCH_ANGLE,
    INVALID_RIGHT_UPPER_ARM_ROLL_ANGLE,
    INVALID_RIGHT_ELBOW_FLEX_ANGLE,
    INVALID_RIGHT_FOREARM_ROLL_ANGLE,
    INVALID_RIGHT_WRIST_PITCH_ANGLE,
    INVALID_RIGHT_WRIST_ROLL_ANGLE,
    INVALID_LEFT_SHOULDER_PAN_ANGLE,
    INVALID_LEFT_SHOULDER_PITCH_ANGLE,
    INVALID_LEFT_UPPER_ARM_ROLL_ANGLE,
    INVALID_LEFT_ELBOW_FLEX_ANGLE,
    INVALID_LEFT_FOREARM_ROLL_ANGLE,
    INVALID_LEFT_WRIST_PITCH_ANGLE,
    INVALID_LEFT_WRIST_ROLL_ANGLE,
    NUM_DEBUG_CODES
  };

/*
  static const char* DebugCodeNames[] =
        {"success",
        "right ik fail ik search success",
        "right ik fail ik search fail",
        "left ik fail ik search success",
        "left ik fail ik search fail",
        "invalid right shoulder pan",
        "invalid right shoulder pitch",
        "invalid right upper arm roll",
        "invalid right elbow flex",
        "invalid right forearm roll",
        "invalid right wrist pitch",
        "invalid right wrist roll",
        "invalid left shoulder pan",
        "invalid left shoulder pitch",
        "invalid left upper arm roll",
        "invalid left elbow flex",
        "invalid left forearm roll",
        "invalid left wrist pitch",
        "invalid left wrist roll",
        "collision between arms",
        "right arm in collision",
        "left arm in collision",
        "attached object in collision"}; 
 */
}

#endif

