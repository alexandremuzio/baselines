import numpy as np

OB_DIM = 89
AC_DIM = 20

def symmetric_ac(o):
    assert o.shape[0] == AC_DIM

    # currJoints.leftShoulderPitch
    currJoints_leftShoulderPitch = o[0]

    # currJoints.leftShoulderYaw
    currJoints_leftShoulderYaw = o[1]

    # currJoints.leftArmRoll
    currJoints_leftArmRoll = o[2]

    # currJoints.leftArmYaw
    currJoints_leftArmYaw = o[3]

    # currJoints.rightShoulderPitch
    currJoints_rightShoulderPitch = o[4]

    # currJoints.rightShoulderYaw
    currJoints_rightShoulderYaw = o[5]

    # currJoints.rightArmRoll
    currJoints_rightArmRoll = o[6]

    # currJoints.rightArmYaw
    currJoints_rightArmYaw = o[7]

    # currJoints.leftHipYawPitch
    currJoints_leftHipYawPitch = o[8]

    # currJoints.leftHipRoll
    currJoints_leftHipRoll = o[9]

    # currJoints.leftHipPitch
    currJoints_leftHipPitch = o[10]

    # currJoints.leftKneePitch
    currJoints_leftKneePitch = o[11]

    # currJoints.leftFootPitch
    currJoints_leftFootPitch = o[12]

    # currJoints.leftFootRoll
    currJoints_leftFootRoll = o[13]

    # currJoints.rightHipYawPitch
    currJoints_rightHipYawPitch = o[14]

    # currJoints.rightHipRoll
    currJoints_rightHipRoll = o[15]

    # currJoints.rightHipPitch
    currJoints_rightHipPitch = o[16]

    # currJoints.rightKneePitch
    currJoints_rightKneePitch = o[17]

    # currJoints.rightFootPitch
    currJoints_rightFootPitch = o[18]

    # currJoints.rightFootRoll
    currJoints_rightFootRoll = o[19]

    ### Apply symmetry to current joints
    o[0]  = currJoints_rightShoulderPitch
    o[1]  = -1.0 * currJoints_rightShoulderYaw  
    o[2]  = -1.0 * currJoints_rightArmRoll 
    o[3]  = -1.0 * currJoints_rightArmYaw 
    o[4]  = currJoints_leftShoulderPitch 
    o[5]  = -1.0 * currJoints_leftShoulderYaw 
    o[6]  = -1.0 * currJoints_leftArmRoll 
    o[7]  = -1.0 * currJoints_leftArmYaw 
    o[8]  = -1.0 * currJoints_rightHipYawPitch 
    o[9]  = -1.0 * currJoints_rightHipRoll 
    o[10] = currJoints_rightHipPitch 
    o[11] = currJoints_rightKneePitch 
    o[12] = currJoints_rightFootPitch 
    o[13] = -1.0 * currJoints_rightFootRoll 
    o[14] = -1.0 * currJoints_leftHipYawPitch 
    o[15] = -1.0 * currJoints_leftHipRoll 
    o[16] = currJoints_leftHipPitch 
    o[17] = currJoints_leftKneePitch 
    o[18] = currJoints_leftFootPitch 
    o[19] = -1.0 * currJoints_leftFootRoll 

    return o

def symmetric_ob(o):
    assert o.shape[0] == OB_DIM

    # nbSteps

    # currJoints.leftShoulderPitch
    currJoints_leftShoulderPitch = o[1]

    # currJoints.leftShoulderYaw
    currJoints_leftShoulderYaw = o[2]

    # currJoints.leftArmRoll
    currJoints_leftArmRoll = o[3]

    # currJoints.leftArmYaw
    currJoints_leftArmYaw = o[4]

    # currJoints.rightShoulderPitch
    currJoints_rightShoulderPitch = o[5]

    # currJoints.rightShoulderYaw
    currJoints_rightShoulderYaw = o[6]

    # currJoints.rightArmRoll
    currJoints_rightArmRoll = o[7]

    # currJoints.rightArmYaw
    currJoints_rightArmYaw = o[8]

    # currJoints.leftHipYawPitch
    currJoints_leftHipYawPitch = o[9]

    # currJoints.leftHipRoll
    currJoints_leftHipRoll = o[10]

    # currJoints.leftHipPitch
    currJoints_leftHipPitch = o[11]

    # currJoints.leftKneePitch
    currJoints_leftKneePitch = o[12]

    # currJoints.leftFootPitch
    currJoints_leftFootPitch = o[13]

    # currJoints.leftFootRoll
    currJoints_leftFootRoll = o[14]

    # currJoints.rightHipYawPitch
    currJoints_rightHipYawPitch = o[15]

    # currJoints.rightHipRoll
    currJoints_rightHipRoll = o[16]

    # currJoints.rightHipPitch
    currJoints_rightHipPitch = o[17]

    # currJoints.rightKneePitch
    currJoints_rightKneePitch = o[18]

    # currJoints.rightFootPitch
    currJoints_rightFootPitch = o[19]

    # currJoints.rightFootRoll
    currJoints_rightFootRoll = o[20]

    ### Apply symmetry to current joints
    o[1]  = currJoints_rightShoulderPitch 
    o[2]  = -1.0 * currJoints_rightShoulderYaw 
    o[3]  = -1.0 * currJoints_rightArmRoll 
    o[4]  = -1.0 * currJoints_rightArmYaw 
    o[5]  = currJoints_leftShoulderPitch 
    o[6]  = -1.0 * currJoints_leftShoulderYaw 
    o[7]  = -1.0 * currJoints_leftArmRoll 
    o[8]  = -1.0 * currJoints_leftArmYaw 
    o[9]  = -1.0 * currJoints_rightHipYawPitch 
    o[10] = -1.0 * currJoints_rightHipRoll 
    o[11] = currJoints_rightHipPitch 
    o[12] = currJoints_rightKneePitch 
    o[13] = currJoints_rightFootPitch 
    o[14] = -1.0 * currJoints_rightFootRoll 
    o[15] = -1.0 * currJoints_leftHipYawPitch 
    o[16] = -1.0 * currJoints_leftHipRoll 
    o[17] = currJoints_leftHipPitch 
    o[18] = currJoints_leftKneePitch 
    o[19] = currJoints_leftFootPitch 
    o[20] = -1.0 * currJoints_leftFootRoll 
    

    # jointsDerivatives.leftShoulderPitch
    jointsDerivatives_leftShoulderPitch = o[21]

    # jointsDerivatives.leftShoulderYaw
    jointsDerivatives_leftShoulderYaw = o[22]

    # jointsDerivatives.leftArmRoll
    jointsDerivatives_leftArmRoll = o[23]

    # jointsDerivatives.leftArmYaw
    jointsDerivatives_leftArmYaw = o[24]

    # jointsDerivatives.rightShoulderPitch
    jointsDerivatives_rightShoulderPitch = o[25]

    # jointsDerivatives.rightShoulderYaw
    jointsDerivatives_rightShoulderYaw = o[26]

    # jointsDerivatives.rightArmRoll
    jointsDerivatives_rightArmRoll = o[27]

    # jointsDerivatives.rightArmYaw
    jointsDerivatives_rightArmYaw = o[28]

    # jointsDerivatives.leftHipYawPitch
    jointsDerivatives_leftHipYawPitch = o[29]

    # jointsDerivatives.leftHipRoll
    jointsDerivatives_leftHipRoll = o[30]

    # jointsDerivatives.leftHipPitch
    jointsDerivatives_leftHipPitch = o[31]

    # jointsDerivatives.leftKneePitch
    jointsDerivatives_leftKneePitch = o[32]

    # jointsDerivatives.leftFootPitch
    jointsDerivatives_leftFootPitch = o[33]

    # jointsDerivatives.leftFootRoll
    jointsDerivatives_leftFootRoll = o[34]

    # jointsDerivatives.rightHipYawPitch
    jointsDerivatives_rightHipYawPitch = o[35]

    # jointsDerivatives.rightHipRoll
    jointsDerivatives_rightHipRoll = o[36]

    # jointsDerivatives.rightHipPitch
    jointsDerivatives_rightHipPitch = o[37]

    # jointsDerivatives.rightKneePitch
    jointsDerivatives_rightKneePitch = o[38]

    # jointsDerivatives.rightFootPitch
    jointsDerivatives_rightFootPitch = o[39]

    # jointsDerivatives.rightFootRoll
    jointsDerivatives_rightFootRoll = o[40]



    ### Apply symmetry to derivative joints
    o[21] = jointsDerivatives_rightShoulderPitch 
    o[22] = -1.0 * jointsDerivatives_rightShoulderYaw 
    o[23] = -1.0 * jointsDerivatives_rightArmRoll 
    o[24] = -1.0 * jointsDerivatives_rightArmYaw 
    o[25] = jointsDerivatives_leftShoulderPitch 
    o[26] = -1.0 * jointsDerivatives_leftShoulderYaw 
    o[27] = -1.0 * jointsDerivatives_leftArmRoll 
    o[28] = -1.0 * jointsDerivatives_leftArmYaw 
    o[29] = -1.0 * jointsDerivatives_rightHipYawPitch 
    o[30] = -1.0 * jointsDerivatives_rightHipRoll 
    o[31] = jointsDerivatives_rightHipPitch 
    o[32] = jointsDerivatives_rightKneePitch 
    o[33] = jointsDerivatives_rightFootPitch 
    o[34] = -1.0 * jointsDerivatives_rightFootRoll 
    o[35] = -1.0 * jointsDerivatives_leftHipYawPitch 
    o[36] = -1.0 * jointsDerivatives_leftHipRoll 
    o[37] = jointsDerivatives_leftHipPitch 
    o[38] = jointsDerivatives_leftKneePitch 
    o[39] = jointsDerivatives_leftFootPitch 
    o[40] = -1.0 * jointsDerivatives_leftFootRoll


    # getAgentAngle()
    # orientationDerivative
    # wiz.getAgentTranslation().z
    # heightDerivative
    # modeling.getAgentModel().getCenterOfMass().x
    # modeling.getAgentModel().getCenterOfMass().y
    CenterOfMass_y = -1.0 * o[46]

    # modeling.getAgentModel().getCenterOfMass().z
    # centerOfMassDerivative.x
    # centerOfMassDerivative.y
    centerOfMassDerivative_y = -1.0 * o[49]

    # centerOfMassDerivative.z
    # modeling.getAgentModel().getTorsoAngularVelocity().x
    # modeling.getAgentModel().getTorsoAngularVelocity().y
    torsoAngularVelocity_y = o[52]
    o[52] = -1.0 * torsoAngularVelocity_y

    # modeling.getAgentModel().getTorsoAngularVelocity().z
    torsoAngularVelocity_z = o[53]
    o[53] = -1.0 * torsoAngularVelocity_z

    # torsoVelocityDerivative.x
    # torsoVelocityDerivative.y
    torsoVelocityDerivative_y = o[55]
    o[55] = -1.0 * torsoVelocityDerivative_y

    # torsoVelocityDerivative.z
    torsoVelocityDerivative_z = o[56]
    o[56] = -1.0 * torsoVelocityDerivative_z

    # modeling.getAgentModel().getTorsoAcceleration().x
    torsoAcceleration_x = o[57]
    o[57] = -1.0 * torsoAcceleration_x

    # modeling.getAgentModel().getTorsoAcceleration().y
    # modeling.getAgentModel().getTorsoAcceleration().z
    # torsoAccelDerivative.x
    torsoAccelDerivative_x = o[60]
    o[60] = -1.0 * torsoAccelDerivative_x

    # torsoAccelDerivative.y
    # torsoAccelDerivative.z
    # perception.getAgentPerception().getLeftFootForceResistanceData().getOriginCoordinates().x
    left_ForceResistanceData_Origin_x = o[63] 

    # perception.getAgentPerception().getLeftFootForceResistanceData().getOriginCoordinates().y
    left_ForceResistanceData_Origin_y = o[64] 

    # perception.getAgentPerception().getLeftFootForceResistanceData().getOriginCoordinates().z
    left_ForceResistanceData_Origin_z = o[65] 

    # leftFootForceCoordDerivatives.x
    left_ForceResistanceData_Origin_x_derivative = o[66]

    # leftFootForceCoordDerivatives.y
    left_ForceResistanceData_Origin_y_derivative = o[67]

    # leftFootForceCoordDerivatives.z
    left_ForceResistanceData_Origin_z_derivative = o[68]

    # perception.getAgentPerception().getLeftFootForceResistanceData().getForce().x
    leftFoot_Force_x = o[69]

    # perception.getAgentPerception().getLeftFootForceResistanceData().getForce().y
    leftFoot_Force_y = o[70]

    # perception.getAgentPerception().getLeftFootForceResistanceData().getForce().z
    leftFoot_Force_z = o[71]

    # leftFootForceDerivatives.x
    leftFootForceDerivatives_x = o[72]

    # leftFootForceDerivatives.y
    leftFootForceDerivatives_y = o[73]

    # leftFootForceDerivatives.z
    leftFootForceDerivatives_z = o[74]

    # perception.getAgentPerception().getRightFootForceResistanceData().getOriginCoordinates().x
    rightFoot_OriginCoordinates_x = o[75]

    # perception.getAgentPerception().getRightFootForceResistanceData().getOriginCoordinates().y
    rightFoot_OriginCoordinates_y = o[76]

    # perception.getAgentPerception().getRightFootForceResistanceData().getOriginCoordinates().z
    rightFoot_OriginCoordinates_z = o[77]

    # rightFootForceCoordDerivatives.x
    rightFootForceCoordDerivatives_x = o[78]

    # rightFootForceCoordDerivatives.y
    rightFootForceCoordDerivatives_y = o[79]

    # rightFootForceCoordDerivatives.z
    rightFootForceCoordDerivatives_z = o[80]

    # perception.getAgentPerception().getRightFootForceResistanceData().getForce().x
    rightFoot_Force_x = o[81]

    # perception.getAgentPerception().getRightFootForceResistanceData().getForce().y
    rightFoot_Force_y = o[82]

    # perception.getAgentPerception().getRightFootForceResistanceData().getForce().z
    rightFoot_Force_z = o[83]

    # rightFootForceDerivatives.x
    rightFootForceDerivatives_x = o[84]

    # rightFootForceDerivatives.y
    rightFootForceDerivatives_y = o[85]

    # rightFootForceDerivatives.z
    rightFootForceDerivatives_z = o[86]



    ### Apply symmetry
    o[63] = -1.0 * rightFoot_OriginCoordinates_x
    o[64] = rightFoot_OriginCoordinates_y
    o[65] = rightFoot_OriginCoordinates_z
    o[66] = -1.0 * rightFootForceCoordDerivatives_x
    o[67] = rightFootForceCoordDerivatives_y
    o[68] = rightFootForceCoordDerivatives_z
    o[69] = -1.0 * rightFoot_Force_x
    o[70] = rightFoot_Force_y 
    o[71] = rightFoot_Force_z
    o[72] = -1.0 * rightFootForceDerivatives_x
    o[73] = rightFootForceDerivatives_y 
    o[74] = rightFootForceDerivatives_z

    o[75] = -1.0 * left_ForceResistanceData_Origin_x
    o[76] = left_ForceResistanceData_Origin_y 
    o[77] = left_ForceResistanceData_Origin_z
    o[78] = -1.0 * left_ForceResistanceData_Origin_x_derivative
    o[79] = left_ForceResistanceData_Origin_y_derivative
    o[80] = left_ForceResistanceData_Origin_z_derivative
    o[81] = -1.0 * leftFoot_Force_x
    o[82] = leftFoot_Force_y
    o[83] = leftFoot_Force_z
    o[84] = -1.0 * leftFootForceDerivatives_x
    o[85] = leftFootForceDerivatives_y
    o[86] = leftFootForceDerivatives_z

    # rightFootCounter
    rightFootCounter = o[87]

    # leftFootCounter
    leftFootCounter = o[88]

    o[87] = leftFootCounter
    o[88] = rightFootCounter

    return o

def apply_symmetry_observation(ob):
    initial_size = ob.shape[0]
    initial_cols = ob.shape[1]
    
    for o in ob:
        ob = np.vstack( (ob, symmetric_ob(o) ) )

    assert ob.shape[0] == 2*initial_size 
    assert ob.shape[1] == initial_cols

    return ob

def apply_symmetry_action(ac):
    initial_size = ac.shape[0]
    initial_cols = ac.shape[1]

    for a in ac:
        ac = np.vstack( (ac, symmetric_ac(a) ) )

    assert ac.shape[0] == 2*initial_size 
    assert ac.shape[1] == initial_cols

    return ac
