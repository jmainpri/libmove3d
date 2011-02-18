#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "gbM/Proto_gbModeles.h"
#include <list>
#include <string>
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/ManipulationStruct.h"
#include "../lightPlanner/proto/Manipulation.h"
#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include "../lightPlanner/proto/ManipulationUtils.hpp"



/* --------- FORM VARIABLES ------- */
FL_FORM  * SOFT_MOTION_FORM = NULL;
static FL_OBJECT * BT_COMP_TRAJ_OBJ = NULL;
static FL_OBJECT * BT_PLOT_Q_PLOT_CURVE_OBJ = NULL;
static FL_OBJECT * MOTIONGROUP = NULL;
static FL_OBJECT  *BT_LOAD_TRAJ_OBJ = NULL;
static FL_OBJECT  *BT_LOAD_TRAJ_APPROX_OBJ = NULL;
static FL_OBJECT  *BT_PLAY_TRAJ_OBJ = NULL;
static FL_OBJECT  *BT_TEST_OBJ = NULL;
static FL_OBJECT  *BT_LOAD_TRAJCONFIG_OBJ = NULL;

static char fileTraj[128];
configPt configTraj[10000];
static int nbConfigTraj = 0;
static char file_directory[512];
configPt configTrajConfig[10000];
static int nbConfigTrajConfig = 0;

/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_soft_motion_group(void);


static void CB_softMotion_compute_traj_obj(FL_OBJECT *obj, long arg);
static void CB_softMotion_plot_curve_obj(FL_OBJECT *obj, long arg);
static void CB_load_traj_obj(FL_OBJECT *obj, long arg);
static void CB_play_traj_obj(FL_OBJECT *obj, long arg);
static void CB_test_obj(FL_OBJECT *obj, long arg);
static void CB_load_trajconfig_obj(FL_OBJECT *obj, long arg);
static void CB_load_traj_approx_obj(FL_OBJECT *obj, long arg);


#ifdef MULTILOCALPATH
/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_soft_motion_form(void) {
	SOFT_MOTION_FORM = fl_bgn_form(FL_UP_BOX, 150, 380);
	g3d_create_soft_motion_group();
	fl_end_form();
}

void g3d_show_soft_motion_form(void) {
	fl_show_form(SOFT_MOTION_FORM, FL_PLACE_SIZE, TRUE, "Soft Motion");
}

void g3d_hide_soft_motion_form(void) {
	fl_hide_form(SOFT_MOTION_FORM);
}

void g3d_delete_soft_motion_form(void) {
	fl_free_form(SOFT_MOTION_FORM);
}

/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_soft_motion_group(void) {
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 370, "Soft Motion");

	MOTIONGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;
	BT_COMP_TRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Compute softMotion traj");
	BT_PLOT_Q_PLOT_CURVE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Plot qi");
	BT_LOAD_TRAJ_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Load qi Trajectory");
	BT_PLAY_TRAJ_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "Play qi Trajectory");
	BT_LOAD_TRAJCONFIG_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 4*dy, w, h, "Load TrajConfig");
	BT_TEST_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 5*dy, w, h, "Test");
	BT_LOAD_TRAJ_APPROX_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 6*dy, w, h, "Load Traj. Approx.");
	
	fl_set_call_back(BT_COMP_TRAJ_OBJ, CB_softMotion_compute_traj_obj, 1);

	fl_set_call_back(BT_PLOT_Q_PLOT_CURVE_OBJ, CB_softMotion_plot_curve_obj, 1);
	fl_set_object_color(BT_PLOT_Q_PLOT_CURVE_OBJ,FL_MCOL,FL_GREEN);
	fl_set_button(BT_PLOT_Q_PLOT_CURVE_OBJ, FALSE);

	fl_set_call_back(BT_LOAD_TRAJ_OBJ, CB_load_traj_obj, 1);
	fl_set_call_back(BT_PLAY_TRAJ_OBJ, CB_play_traj_obj, 1);
	fl_set_call_back(BT_LOAD_TRAJCONFIG_OBJ, CB_load_trajconfig_obj, 1);
	fl_set_call_back(BT_TEST_OBJ, CB_test_obj, 1);
	fl_set_call_back(BT_LOAD_TRAJ_APPROX_OBJ, CB_load_traj_approx_obj, 1);
	
	fl_end_group();
}

static void CB_softMotion_compute_traj_obj(FL_OBJECT *ob, long arg) {
	void (*fct_draw)(void);
	p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	int ir = p3d_get_desc_curnum(P3D_ROBOT);

// 	double gain,gaintot=1.;
	//int lp[10000];
	// double positions[7][10000];

	std::vector <int> lp;
	std::vector < std::vector <double> > positions;
	SM_TRAJ smTraj;

	if(!traj) {
		printf("Soft Motion : ERREUR : no current traj\n");
		return ;
	}

	fct_draw = &(g3d_draw_allwin_active);


	if(!traj || traj->nlp < 1) {
		if(ob){//if is not a automatic call
			printf("Optimization not possible: current trajectory\
					contains one or zero local path\n");
		}
		fl_set_button(BT_COMP_TRAJ_OBJ,0);
		return;
	}
	if(ob){fl_set_cursor(FL_ObjWin(ob), XC_watch);}

	fct_draw = &(g3d_draw_allwin_active);

	if(p3d_convert_traj_to_softMotion(traj, ENV.getBool(Env::writeSoftMotionFiles), lp, positions, smTraj)){
		/* position the robot at the beginning of the optimized trajectory */
		position_robot_at_beginning(ir, traj);
	}
	if (fct_draw){(*fct_draw)();}

	g3d_win *win= NULL;
	win= g3d_get_cur_win();
	win->fct_draw2 = &(draw_trajectory_ptp);

	g3d_draw_allwin_active();
	if(ob){fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);fl_set_button(ob,0);}
	return;

}

static void CB_softMotion_plot_curve_obj(FL_OBJECT *obj, long arg) {
	
        ENV.setBool(Env::plotSoftMotionCurve, !ENV.getBool(Env::plotSoftMotionCurve));
	
	if(ENV.getBool(Env::plotSoftMotionCurve))
	{  fl_set_button(BT_PLOT_Q_PLOT_CURVE_OBJ, TRUE);
	}
	else
	{  fl_set_button(BT_PLOT_Q_PLOT_CURVE_OBJ, FALSE);
	}
}

static void CB_test_obj(FL_OBJECT *ob, long arg) {
	if(ob){fl_set_cursor(FL_ObjWin(ob), XC_watch);}
	p3d_rob* rob=NULL;



	 SM_TRAJ smTest;
	 smTest.load((char*)"./softMotionTraj.traj", NULL);
         // smTest.print();
         smTest.save((char*)"./softMotionTest.traj");
         return;

	rob = XYZ_ENV->cur_robot;
	double x, y, z, rx, ry, rz;
p3d_mat4ExtractPosReverseOrder(rob->joints[9]->abs_pos, &x, &y, &z, &rx, &ry , &rz);
// RADTODEG does not exist watch out include files
//printf("x %f %f %f %f %f %f\n",x, y, z, RADTODEG*rx, RADTODEG*ry, RADTODEG*rz);
print_config_one_line_degrees(rob, rob->ROBOT_POS);
	if(ob){fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);fl_set_button(ob,0);}
}

void sm_set_PLOT_Q_ARM(int value) {
	if(value == 1) {
	  ENV.setBool(Env::plotSoftMotionCurve, true);
	} else {
	   ENV.setBool(Env::plotSoftMotionCurve, false);
	}
	return;
}


void draw_trajectory_approx() {
	 int i = 0;
         int index_dof = 0;
	p3d_jnt* jnt = NULL;
        p3d_rob* robotPt= (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);;
	jnt = p3d_get_robot_jnt_by_name(robotPt, (char*)"virtual_object");
	index_dof = jnt->index_dof;
	p3d_vector3 p1;
	p3d_vector3 p2;
	if(configTrajConfig[0]==NULL || nbConfigTrajConfig<=0)
	{  return;  }

	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glColor3f(0,1,0);
	for(i=1; i<nbConfigTrajConfig; i++) {
		for(int j=0; j<3; j++) {
			p1[j] = configTrajConfig[i-1][index_dof +j];
			p2[j] = configTrajConfig[i][index_dof +j];
		}
		g3d_draw_cylinder(p1, p2, 0.0005, 16);
	}
	glPopAttrib();
}


static int read_trajectory(p3d_rob* robotPt, FILE *fileptr) {
	nbConfigTraj = 0;
	int index_dof = 11;

	if(nbConfigTraj>0) {
		for(int v=nbConfigTraj-1; v>= 0; v--) {
			p3d_destroy_config(robotPt,configTraj[v]);
		}
		nbConfigTraj = 0;
	}
	/* Read File Variables */
	while(!feof(fileptr)) {
		configTraj[nbConfigTraj] = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
		fscanf(fileptr, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					 &configTraj[nbConfigTraj][index_dof],&configTraj[nbConfigTraj][index_dof+1],&configTraj[nbConfigTraj][index_dof+2],
			&configTraj[nbConfigTraj][index_dof+3],&configTraj[nbConfigTraj][index_dof+4],&configTraj[nbConfigTraj][index_dof+5]);
		nbConfigTraj ++;
		if (nbConfigTraj>= 10000) {
			printf("File too long\n");

			return TRUE;
		}
	}
	return FALSE;
}

static int read_trajectory_config(p3d_rob* robotPt, FILE *fileptr) {
	nbConfigTrajConfig = 0;
	int index_dof = 21;
	int       i, ir;
	char name[64];
	ir = p3d_get_desc_curnum(P3D_ROBOT);
	configPt configRef = NULL;
	for(i=0; i<robotPt->nconf; i++) {
	if (robotPt->conf[i] != NULL) {
		p3d_del_config(robotPt, robotPt->conf[i]);
	}
}
				configRef = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
	if(nbConfigTrajConfig>0) {
		for(int v=nbConfigTrajConfig-1; v>= 0; v--) {
			p3d_destroy_config(robotPt,configTrajConfig[v]);
		}
		nbConfigTrajConfig = 0;
	}
	/* Read File Variables */
	while(!feof(fileptr)) {
		configTrajConfig[nbConfigTrajConfig] = p3d_copy_config(robotPt, configRef);
		fscanf(fileptr, "%lf\t%lf\t%lf %lf\t%lf\t%lf\n",
					 &configTrajConfig[nbConfigTrajConfig][index_dof],&configTrajConfig[nbConfigTrajConfig][index_dof+1],&configTrajConfig[nbConfigTrajConfig][index_dof+2],
			&configTrajConfig[nbConfigTrajConfig][index_dof+3],&configTrajConfig[nbConfigTrajConfig][index_dof+4],&configTrajConfig[nbConfigTrajConfig][index_dof+5]);
		configTrajConfig[nbConfigTrajConfig][8] = 0.0 ; // set yaw to 0.0
		configTrajConfig[nbConfigTrajConfig][index_dof] += robotPt->ROBOT_POS[6];
		configTrajConfig[nbConfigTrajConfig][index_dof+1] += robotPt->ROBOT_POS[7];
		p3d_set_and_update_this_robot_conf(robotPt, configTrajConfig[nbConfigTrajConfig]);
		sprintf(name, "configTraj_%i", nbConfigTrajConfig);
		p3d_set_new_robot_config(name, configTrajConfig[nbConfigTrajConfig], robotPt->ikSol, robotPt->confcur);
		robotPt->confcur = robotPt->conf[nbConfigTrajConfig];
		FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
		nbConfigTrajConfig ++;
		if (nbConfigTrajConfig>= 50) {
			printf("File too long\n");

			return TRUE;
		}
	}
	p3d_destroy_config(robotPt, configRef);
	return FALSE;
}

static int read_trajectory_approx(p3d_rob* robotPt, FILE *fileptr) {

	std::vector < std::vector <double> > positions;
	FILE *fileptr2 = NULL;
std::vector <double>  q_arm;
	configPt q = NULL;
int j=0;

	nbConfigTrajConfig = 0;
	int index_dof = 0, index_dof_arm = 0;
	p3d_jnt* jnt = NULL;
	int       i, ir;

//         p3d_vector3 p1;
// 	p3d_vector3 p2;
	jnt = p3d_get_robot_jnt_by_name(robotPt, (char*)"virtual_object");
	index_dof = jnt->index_dof;
	
	ir = p3d_get_desc_curnum(P3D_ROBOT);
	configPt configRef = NULL;
	for(i=0; i<robotPt->nconf; i++) {
	  if (robotPt->conf[i] != NULL) {
		  p3d_del_config(robotPt, robotPt->conf[i]);
	  }
	}
	configRef = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
	if(nbConfigTrajConfig>0) {
		for(int v=nbConfigTrajConfig-1; v>= 0; v--) {
			p3d_destroy_config(robotPt,configTrajConfig[v]);
		}
		nbConfigTrajConfig = 0;
	}


        g3d_win *win= NULL;
	win= g3d_get_cur_win();
	win->fct_draw2 = &(draw_trajectory_approx);

	if ((fileptr2 = fopen("qarm.traj","w+"))==NULL) {
		printf("cannot open File qarm.traj");
	}
	q_arm.clear();
	positions.clear();

	int arm_mlpID = -1;
int v=0;
int nb_armDof= 0;
	for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
// 		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-arm_lin") == 0) {
// 			arm_mlpID = iGraph;
// 		}
// 		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-rarm_lin") == 0) {
// 			arm_mlpID = iGraph;
// 		}
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBody") == 0) {
			arm_mlpID = iGraph;
		}
	}
	if(robotPt->mlp->mlpJoints[arm_mlpID]->nbJoints !=7){
	  printf("ERROR p3d_softMotion_write_curve_for_bltplot unknow robot type 02 \n");
	  return 1;
	}
	if(arm_mlpID == -1) {
	  printf("ERROR p3d_softMotion_write_curve_for_bltplot unknow robot type  03\n");
	  return 1;
	}

	index_dof_arm = robotPt->joints[robotPt->mlp->mlpJoints[arm_mlpID]->joints[0]]->index_dof;
	
	/* Read File Variables */	
	while(!feof(fileptr)) {
		configTrajConfig[nbConfigTrajConfig] = p3d_copy_config(robotPt, configRef);
		fscanf(fileptr, "%lf\t%lf\t%lf\n",
					 &configTrajConfig[nbConfigTrajConfig][index_dof],&configTrajConfig[nbConfigTrajConfig][index_dof+1],
					 &configTrajConfig[nbConfigTrajConfig][index_dof+2]);
					 //,&configTrajConfig[nbConfigTrajConfig][index_dof+3],&configTrajConfig[nbConfigTrajConfig][index_dof+4],&configTrajConfig[nbConfigTrajConfig][index_dof+5]);

		configTrajConfig[nbConfigTrajConfig][11] = 0.0 ; // set yaw to 0.0
		configTrajConfig[nbConfigTrajConfig][index_dof] += robotPt->ROBOT_POS[index_dof];
		configTrajConfig[nbConfigTrajConfig][index_dof+1] += robotPt->ROBOT_POS[index_dof+1];
		configTrajConfig[nbConfigTrajConfig][index_dof+2] += robotPt->ROBOT_POS[index_dof+2];
		configTrajConfig[nbConfigTrajConfig][index_dof+3] = robotPt->ROBOT_POS[index_dof+3];
		configTrajConfig[nbConfigTrajConfig][index_dof+4] = robotPt->ROBOT_POS[index_dof+4];
		configTrajConfig[nbConfigTrajConfig][index_dof+5] = robotPt->ROBOT_POS[index_dof+5];
		
		p3d_set_and_update_this_robot_conf(robotPt, configTrajConfig[nbConfigTrajConfig]);
 	        g3d_draw_allwin_active();

	        q = p3d_get_robot_config(robotPt);
		j=0;
		nb_armDof = 7;
		
		v=index_dof_arm;
		if(fileptr2 != NULL) {
			
		  for(int w=0; w<7;w++){
			    fprintf(fileptr2,"%f ",q[v]);
			    v++;

		  }
		  fprintf(fileptr2,"\n");
		}

		nbConfigTrajConfig ++;
		if (nbConfigTrajConfig>= 10000) {
			printf("File too long\n");

			return TRUE;
		}
		p3d_destroy_config(robotPt, q);
		q = NULL;
	}
	if(fileptr != NULL) {
	  fclose(fileptr2);
	  printf("File qarm.traj created\n");
	}
        g3d_draw_allwin_active();
	p3d_destroy_config(robotPt, configRef);
	return FALSE;
}

static void read_trajectory_by_name(p3d_rob* robotPt, const char *file) {
	FILE *fdc = NULL;
	int ret = 0;
	/* on lit la trajectoire */
		if (fileTraj){
				if(!(fdc=fopen(fileTraj,"r"))) {
					PrintError(("p3d_rw_scenario_read: can't open %s\n",file));
					return;
				}
				ret = read_trajectory(robotPt, fdc);
				fclose(fdc);
		}
	return;
}

static void read_trajectory_config_by_name(p3d_rob* robotPt, const char *file) {
	FILE *fdc;
	int ret;
	/* on lit la trajectoire */
	if (fileTraj){
		if(!(fdc=fopen(fileTraj,"r"))) {
			PrintError(("p3d_rw_trajConfig_read: can't open %s\n",file));
			return;
		}
		ret = read_trajectory_config(robotPt, fdc);
		fclose(fdc);
	}
	return;
}

static void read_trajectory_approx_by_name(p3d_rob* robotPt, const char *file) {
	FILE *fdc;
	int ret;
	/* on lit la trajectoire */
	if (fileTraj){
		if(!(fdc=fopen(fileTraj,"r"))) {
			PrintError(("p3d_rw_trajApprox_read: can't open %s\n",file));
			return;
		}
		ret = read_trajectory_approx(robotPt, fdc);
		fclose(fdc);
	}
	return;
}

static void CB_load_traj_obj(FL_OBJECT *ob, long arg) {
	double qplot[6][10000];
	gnuplot_ctrl * h = NULL;
	int index = 0, v=0, j=0, i=0;
	const char*file = NULL;
	FILE * f = NULL;
	p3d_rob* robotPt= NULL;
	/* lecture du fichier environnement */

	int r, nr;

	r = p3d_get_desc_curnum(P3D_ROBOT);
	nr= p3d_get_desc_number(P3D_ROBOT);

	for(i=0; i<nr; i++){
		robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
		if(strcmp("ROBOT", robotPt->name)==0){
			break;
		}
	}




	p3d_set_directory(file_directory);
	file	= fl_show_fselector("Trajectory filename", file_directory,	"*.traj", "");
	if(file == NULL) {
		printf("no file to load\n");
		return;
	}
	strcpy(fileTraj, file);
	read_trajectory_by_name(robotPt, fileTraj);
	p3d_desactivateAllCntrts(robotPt);
	index = 0;
	for(i=0; i<nbConfigTraj; i++) {
		p3d_set_and_update_this_robot_conf(robotPt, configTraj[i]);
		g3d_draw_allwin_active();
		j = 0;
		for(v=11; v<(11+6); v++) {
			qplot[j][index] = configTraj[i][v];
			j++;
		}
		index = index + 1;
	}
	f = fopen("temp.dat","w");
	for(i=0; i<index; i++){
		fprintf(f,"%d %f %f %f %f %f %f\n",	i, qplot[0][i],qplot[1][i],qplot[2][i],qplot[3][i],qplot[4][i],qplot[5][i]);
	}
	fclose(f);
	h = gnuplot_init();
	if(h == NULL){
		printf("Gnuplot Init problem");
	}
	gnuplot_cmd(h,(char*)"set term wxt");
	gnuplot_cmd(h,(char*)"set xrange [%d:%d]",0,index-1);
	gnuplot_cmd(h,(char*)"set yrange [-pi:pi]");
	gnuplot_cmd(h, (char*)"plot '%s' using 1:2 with lines lt 1 ti \"q1\", '%s' using 1:3 with lines lt 2 ti \"q2\" , '%s' using 1:4 with lines lt 3 ti \"q3\",'%s' using 1:5 with lines lt 4 ti \"q4\", '%s' using 1:6 with lines lt 5 ti \"q5\", '%s' using 1:7 with lines lt 6 ti \"q6\" " , "temp.dat", "temp.dat", "temp.dat", "temp.dat", "temp.dat", "temp.dat");

	fl_set_button(BT_LOAD_TRAJ_OBJ,0);
}

static void CB_play_traj_obj(FL_OBJECT *ob, long arg) {
	int i, r, nr;
	p3d_rob *robotPt = NULL;
	r = p3d_get_desc_curnum(P3D_ROBOT);
	nr= p3d_get_desc_number(P3D_ROBOT);

	for(i=0; i<nr; i++){
		robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
		if(strcmp("ROBOT", robotPt->name)==0){
			break;
		}
	}

	p3d_desactivateAllCntrts(robotPt);
	for(int i=0; i<nbConfigTraj; i++) {
		p3d_set_and_update_this_robot_conf(robotPt, configTraj[i]);
		g3d_draw_allwin_active();
	}
	fl_set_button(BT_PLAY_TRAJ_OBJ,0);
}



static void CB_load_trajconfig_obj(FL_OBJECT *ob, long arg) {
	int i=0;
	const char*file = NULL;
// 	FILE * f = NULL;
	p3d_rob* robotPt= NULL;
	/* lecture du fichier environnement */

	int r, nr;

	r = p3d_get_desc_curnum(P3D_ROBOT);
	nr= p3d_get_desc_number(P3D_ROBOT);

	for(i=0; i<nr; i++){
		robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
		if(strcmp("ROBOT", robotPt->name)==0){
			break;
		}
	}

	p3d_set_directory(file_directory);
	file	= fl_show_fselector("Trajectory filename", file_directory,	"*.traj", "");
	if(file == NULL) {
		printf("no file to load\n");
		return;
	}
	strcpy(fileTraj, file);
	read_trajectory_config_by_name(robotPt, fileTraj);

	fl_set_button(BT_LOAD_TRAJCONFIG_OBJ,0);
}

static void CB_load_traj_approx_obj(FL_OBJECT *obj, long arg){

	const char*file = NULL;
// 	FILE * f = NULL;
	p3d_rob* robotPt= (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);;
	/* lecture du fichier environnement */


	p3d_set_directory(file_directory);
	file	= fl_show_fselector("Trajectory filename", file_directory,	"*.traj", "");
	if(file == NULL) {
		printf("no file to load\n");
		return;
	}
	strcpy(fileTraj, file);
	read_trajectory_approx_by_name(robotPt, fileTraj);

	fl_set_button(BT_LOAD_TRAJCONFIG_OBJ,0);

}
#endif

