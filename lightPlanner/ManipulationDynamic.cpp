#include "ManipulationDynamic.hpp"
#include "p3d_chanEnv_proto.h"


ManipulationDynamic::ManipulationDynamic(p3d_rob* robot): _robot(robot){
}

//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int ManipulationDynamic::checkCollisionOnTraj() {
    //   configPt currentPos = p3d_get_robot_config(robotPt);
    //   double armPos[6] = {currentPos[5], currentPos[6], currentPos[7], currentPos[8], currentPos[9], currentPos[10]};
    if (checkCollisionOnTraj(0)) {
        printf("There is collision\n");
        return 1;
    } else {
        printf("There is no collision\n");
        return 0;
    }
}

//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int  ManipulationDynamic::checkCollisionOnTraj(int currentLpId) {
    p3d_traj *traj = NULL;

    XYZ_ENV->cur_robot = _robot;
    //initialize and get the current linear traj
    if (!traj) {
        if (_robot->nt < _robot->tcur->num - 2) {
            printf("BioMove3D: checkCollisionOnTraj unvalid traj number : nbTraj = %d, robot tcur = %d\n", _robot->nt, _robot->tcur->num);
            return 1;
        } else {
            traj = _robot->t[_robot->tcur->num - 2];
        }
    }
    checkConfigForCartesianMode(NULL, NULL);
    if (currentLpId > _robot->tcur->nlp) {
        printf("BioMove3D: checkCollisionOnTraj given lpId  = %d > tcur nlp = %d\n", currentLpId, _robot->tcur->nlp);
        currentLpId = 0;
    }
    p3d_localpath* currentLp = traj->courbePt;
    for (int i = 0; i < currentLpId / 2; i++) {
        currentLp = currentLp->next_lp;
    }
    return checkForCollidingPath(_robot, traj, currentLp);
}

/** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
 \return MANIPULATION_TASK_OK for success */
MANIPULATION_TASK_MESSAGE ManipulationDynamic::replanCollidingTraj(int currentLpId, std::vector <p3d_traj*> &trajs) {
    p3d_traj* traj = NULL;
    std::vector<double>  objStart, objGoto;
    XYZ_ENV->cur_robot = _robot;
    //initialize and get the current linear traj
    if (!traj) {
        if (_robot->nt < _robot->tcur->num - 2) {
            return MANIPULATION_TASK_INVALID_TRAJ_ID;
        } else {
            traj = _robot->t[_robot->tcur->num - 2];
        }
    }
    checkConfigForCartesianMode(NULL, NULL);
    if (currentLpId > _robot->tcur->nlp) {
        printf("BioMove3D: checkCollisionOnTraj given lpId  = %d > tcur nlp = %d\n", currentLpId, _robot->tcur->nlp);
        currentLpId = 0;
    }
    p3d_localpath* currentLp = traj->courbePt;
    for (int i = 0; i < currentLpId / 2; i++) {
        currentLp = currentLp->next_lp;
    }
    configPt currentConfig = p3d_get_robot_config(_robot);
    int j = 0, returnValue = 0, optimized = traj->isOptimized;
    if (optimized) {
        p3dAddTrajToGraph(_robot, _robot->GRAPH, traj);
    }
    //   printf("nbTraj before : %d\n", _robot->nt);
    do {
        printf("Test %d\n", j);
        j++;
        returnValue = replanForCollidingPath(_robot, traj, _robot->GRAPH, currentConfig, currentLp, optimized);
        traj = _robot->tcur;
        currentLp = traj->courbePt;
    } while (returnValue != 1 && returnValue != 0 && returnValue != -2 && j < 10);

    printf("nbTraj after : %d, returnValue = %d\n", _robot->nt, returnValue);

    if (returnValue == 1 && j == 0) { //no collision on traj
        return armPlanTask(ARM_FREE, 0, currentConfig, _robot->ROBOT_GOTO, objStart, objGoto, (char *) "",(char *) "", trajs);
    }
    p3d_traj* replanTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
    if (optimized || j > 1) {
        optimiseTrajectory(_robot, replanTraj, _optimizeSteps, _optimizeTime);
    }
    trajs.push_back(replanTraj);
    return MANIPULATION_TASK_OK;
}

#ifdef MULTILOCALPATH
/** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
 \return MANIPULATION_TASK_OK for success */
MANIPULATION_TASK_MESSAGE  ManipulationDynamic::replanCollidingTraj(int currentLpId, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
    p3d_traj *traj = NULL;
    MANIPULATION_TASK_MESSAGE returnMessage = MANIPULATION_TASK_OK;
    std::vector <p3d_traj*> trajs;

    p3d_multiLocalPath_disable_all_groupToPlan(_robot);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);

    returnMessage = replanCollidingTraj(currentLpId, trajs);
    if (returnMessage == MANIPULATION_TASK_OK ) {//There is a new traj
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
        if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
            /* COMPUTE THE SOFTMOTION TRAJECTORY */
            MANPIPULATION_TRAJECTORY_CONF_STR conf;
            SM_TRAJ smTraj;
            computeSoftMotion(traj, conf, smTraj);
            confs.push_back(conf);
            smTrajs.push_back(smTraj);
        } else {
            returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
        }
        //peut etre ajouter un return specific pour savoir qu'il y'a une nouvelle traj
    }
    if(traj){
      p3d_destroy_traj(_robot, traj);
    }
    return returnMessage;
}
#endif
