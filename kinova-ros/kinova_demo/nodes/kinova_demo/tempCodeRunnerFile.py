    position = [0.4,-0.5,0]
    quaternion = tf.transformations.quaternion_from_euler(90*3.1415/180, 0, 0,'rxyz')  
    result = cartesian_pose_client(position, quaternion, prefix)  

    #open gripper
    result = gripper_client([0,0,0], prefix)