


extern int gpCompute_stable_poses(p3d_obj *object, p3d_vector3 cmass, std::list<gpPose> &poseList);

extern int gpFind_poses_on_object(p3d_obj *object, p3d_obj *support, std::list<gpPose> &poseListIn, std::list<gpPose> &poseListOut);

extern int gpSample_horizontal_faces(p3d_obj *object, double step, std::list<gpVector3D> &sampleList);

