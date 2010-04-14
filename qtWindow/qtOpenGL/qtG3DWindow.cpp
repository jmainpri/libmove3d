#include "qtG3DWindow.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"

#include <iostream>
#include <string>

using namespace std;

extern void* GroundCostObj;

G3D_Window *G3D_WIN;
G3D_Window *G3D_WINDOW_LST = NULL;

p3d_matrix4 Id = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

qtG3DWindow::qtG3DWindow()
{
    newG3dWindow();
}

const string
        XformError("Error: Xform is not compiled, check your flags or make changes to the code");

int fct_stop(void)
{
//            cout << XformError << endl;
//	cout << "In Function : " <<  __func__ << endl;
    static double ti = 0;
    double ts, tu;
#ifdef P3D_PLANNER
    double tmax = p3d_get_tmax();
#endif

    ChronoTimes(&tu, &ts);
//    if (tu > ti) {
//      fl_check_forms();
//      ti = tu + 0.5;
//    }
#ifdef P3D_PLANNER
    if ((p3d_GetStopValue()) || ((tu > tmax) && (tmax != 0))) {
      // STOP = FALSE;
      //  p3d_SetStopValue(FALSE);
      ti = 0;
      return false;
    }
#endif
    return true;
}

void fct_draw(void)
{
    g3d_draw_allwin_active();
}

FILE* DifficultyFile=NULL;

int G3D_MODIF_VIEW  = 0x00; /* flag true durant modif viewing */
int G3D_SAVE_MULT = 0x00;
int G3D_ACTIVE_CC = 1;

/**
 * Gets the current window states
 */
g3d_states& g3d_get_cur_states(){
    return(G3D_WIN->vs);
}

/**
 * Gets the current window states by window name
 */
g3d_states& g3d_get_states_by_name(char *s){
    return (G3D_WIN->vs);
}

G3D_Window *g3d_get_cur_win ( void )
{
    return G3D_WIN;
}

G3D_Window * qt_get_cur_g3d_win()
{
    return G3D_WIN;
}

G3D_Window *g3d_get_win_by_name(char *s)
{
//	G3D_Window *w = G3D_WINDOW_LST;
//	while(w)
//	{
//		if (strcmp(s,w->name)==0)
//			return w;
//		w = w->next;
//	}
//	return NULL;
	
	return G3D_WIN;
}

void g3d_draw_allwin_active(void)
{
    if(pipe2openGl)
    {
          pipe2openGl->update();
    }
}

void g3d_add_traj ( char *name, int i )
{
    cout << XformError << endl;
	cout << "In Function : " <<  __func__ << endl;
}

void g3d_set_picking(unsigned int enabled)
{
    cout << XformError << endl;
	cout << "In Function : " <<  __func__ << endl;
}

int g3d_get_KCD_CHOICE_IS_ACTIVE()
{
//            cout << XformError << endl;
	//cout << "In Function : " <<  __func__ << endl;
    return 0;
}

void g3d_refresh_allwin_active ( void )
{
    cout << XformError << endl;
	cout << "In Function : " <<  __func__ << endl;
}


/***********************************************************/
// allows to choice the jnt which frame is drawn in the graph

static int user_drawnjnt = -1;

int p3d_get_user_drawnjnt(void) {
	return(user_drawnjnt);
}
void p3d_set_user_drawnjnt(int jnt) {
	user_drawnjnt = jnt;
}

void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) {
    static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    p3d_matrix4 m_aux;
    p3d_vector4 Xx;

#ifdef HRI_PLANNER
    if (win->point_of_view==0){
#endif
        get_lookat_vector(win->vs, Xx);
        get_pos_cam_matrix(win->vs, Txc);

        p3d_mat4Mult(*win->cam_frame,Txc,m_aux);
        p3d_matvec4Mult(m_aux,Xx,Xc);
        p3d_matvec4Mult(*win->cam_frame,Xx,Xw);
#ifdef HRI_PLANNER
    }
    else {
        //Modified Luis
        if (PSP_ROBOT){
            p3d_rob *r = PSP_ROBOT;
            p3d_obj *objPt = r->o[r->cam_body_index];
            p3d_jnt *jntPt = objPt->jnt;

            //Robot cam
            //Setting up camera position
            Xx[0]=r->cam_pos[0];
            Xx[1]=r->cam_pos[1];
            Xx[2]=r->cam_pos[2];
            Xx[3]=1;
            p3d_matvec4Mult(jntPt->abs_pos,Xx,Xc);
            //Setting up camera orientation
            Xx[0]=r->cam_dir[0];
            Xx[1]=r->cam_dir[1];
            Xx[2]=r->cam_dir[2];
            p3d_matvec4Mult(jntPt->abs_pos,Xx,Xw);

        }
    }
#endif
}


void qt_calc_cam_param()
{
    p3d_vector4 Xc, Xw;
    p3d_vector4 up;

    //	  std::cout << "G3D_WINDOW_CUR->zo = "<< G3D_WINDOW_CUR->zo << std::endl;

    calc_cam_param(G3D_WIN, Xc, Xw);
//
//    std::cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << std::endl;
//    std::cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << std::endl;
//    std::cout << up[0] << " " << up[1] << " " << up[2] << std::endl;

//    Xc[0] = 5365.67;    Xc[1] = 1659.8;     Xc[2] = 4318.32;
//    Xw[0] = 0;          Xw[1] = 0;          Xw[2] = 1250;
//    up[0] = 6.95e-310;  Xw[1] = 6.95e-310;  up[2] = 6.95e-310;

    JimXc[0] = Xc[0];
    JimXc[1] = Xc[1];
    JimXc[2] = Xc[2];

    JimXw[0] = Xw[0];
    JimXw[1] = Xw[1];
    JimXw[2] = Xw[2];

    if (G3D_WIN)
    {
        p3d_matvec4Mult(*G3D_WIN->cam_frame, G3D_WIN->vs.up, up);
    }
    else
    {
        up[0] = 0;
        up[1] = 0;
        up[2] = 1;
    }

    Jimup[0] = up[0];
    Jimup[1] = up[1];
    Jimup[2] = up[2];
}

void qt_canvas_viewing(int mouse_press, int button)
{
    G3D_Window *g3dwin = G3D_WIN;
    int w = G3D_WINSIZE_WIDTH;
    int h = G3D_WINSIZE_HEIGHT;
    unsigned int key;

    static int i0, j0;//,idr=-1;
    int i, j;

    static double x, y, z, zo, el, az;
    double x_aux, y_aux, az_aux, rotinc, incinc;

    G3D_MODIF_VIEW = TRUE;

    if (mouse_press)
    {
        qt_get_win_mouse(&i0, &j0);
        /*printf("----------------------------------\n", (double) i0);
                 printf("i0 = %d\n", i0);
                 printf("j0 = %d\n", j0);*/
        x = g3dwin->vs.x;
        y = g3dwin->vs.y;
        z = g3dwin->vs.z;
        zo = g3dwin->vs.zo;
        el = g3dwin->vs.el;
        az = g3dwin->vs.az;
    }
    else
    {
        qt_get_win_mouse(&i, &j);

        if (button == 0)
        {
            key = 256;
        }
        else if (button == 1)
        {
            key = 512;
        }
        else if (button == 2)
        {
            key = 1024;
        }

        /*printf("i = %d\n", i);
                 printf("j = %d\n", j);*/
        //			printf("%d\n",key);
        i -= i0;
        j -= j0;
        switch (key)
        {
        case 260: /* Select Object and Displace in X */
            //	move_robot(idr,0,j,g3dwin->size/h);
            break;
        case 516: /* Select Object and Displace in Y */
            //	move_robot(idr,1,i,g3dwin->size/w);
            break;
        case 1028:/* Select Object and Displace in Z */
            //	move_robot(idr,2,j,g3dwin->size/h);
            break;
        case 256: /* zoom */
        case 272:
        case 258:
        case 274:
        case 8464:
        case 16656:
            g3dwin->vs.zo = (zo * i) / w + zo;
            if (g3dwin->vs.zo < .0)
            {

                g3dwin->vs.zo = .0;
            }
            //			std::cout << "g3dwin->zo = "<< g3dwin->zo << std::endl;
            break;
                case 512: /* angle */
                case 514:
                case 528:
                case 530:
                case 8720:
                case 16912:
                    g3dwin->vs.az = (-2 * GAIN_AZ * i) / w + az;
                    if (g3dwin->vs.az < .0)
                        g3dwin->vs.az = 2 * M_PI;
                    if (g3dwin->vs.az > 2 * M_PI)
                        g3dwin->vs.az = .0;
                    g3dwin->vs.el = (GAIN_EL * j) / w + el;
                    if (g3dwin->vs.el < -M_PI / 2.0)
                        g3dwin->vs.el = -M_PI / 2.0;
                    if (g3dwin->vs.el > M_PI / 2.0)
                        g3dwin->vs.el = M_PI / 2.0;
                    break;
                case 1024: /* origin esf*/
                case 1026:
                case 1040:
                case 1042:
                case 9232:
                case 17424:
                    rotinc = (-GAIN_AZ * i) / w;
                    incinc = (-GAIN_EL * j) / w;
                    /* rotation effects */
                    g3dwin->vs.x = x + 2.0 * zo * cos(el) * sin(rotinc / 2.0) * sin(az
                                                                                 + rotinc / 2.0);
                    g3dwin->vs.y = y - 2.0 * zo * cos(el) * sin(rotinc / 2.0) * cos(az
                                                                                 + rotinc / 2.0);
                    g3dwin->vs.az = az + rotinc;

                    if (g3dwin->vs.az < .0)
                        g3dwin->vs.az = 2 * M_PI;
                    if (g3dwin->vs.az > 2 * M_PI)
                        g3dwin->vs.az = .0;
                    x_aux = g3dwin->vs.x;
                    y_aux = g3dwin->vs.y;
                    az_aux = g3dwin->vs.az;
                    /* incline effects */
                    g3dwin->vs.z = z - 2.0 * zo * sin(incinc / 2.0) * cos(el + incinc
                                                                       / 2.0);
                    g3dwin->vs.x = x_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
                                                                           / 2.0) * cos(az_aux);
                    g3dwin->vs.y = y_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
                                                                           / 2.0) * sin(az_aux);
                    g3dwin->vs.el = el + incinc;
                    if (g3dwin->vs.el < -M_PI / 2)
                        g3dwin->vs.el = -M_PI / 2;
                    if (g3dwin->vs.el > M_PI / 2)
                        g3dwin->vs.el = M_PI / 2;
                    break;
                    /* deplacement du  point central de l'image */
                    /* - bouton gauche et central -> x          */
                    /* - bouton droit et central  -> y          */
                    /* - bouton gauche et droit   -> z          */
                case 768: /* origin x */
                case 770:
                case 784:
                case 8976:
                case 17168:
                    g3dwin->vs.x = x + j * g3dwin->vs.size / w;
                    /*       g3dwin->x = x + i*g3dwin->size/h; */
                    break;
                case 1536: /* origin y */
                case 1552:
                case 1538:
                case 9744:
                case 17936:
                    g3dwin->vs.y = y + j * g3dwin->vs.size / w;
                    /*       g3dwin->y = y + i*g3dwin->size/h; */
                    break;
                case 1280: /* origin z */
                case 1282:
                case 1296:
                case 9488:
                case 17680:
                    g3dwin->vs.z = z + j * g3dwin->vs.size / w;
                    /*       g3dwin->z = z + i*g3dwin->size/h; */
                    break;
                case 1792: /* reset up */
                    g3dwin->vs.up[0] = .0;
                    g3dwin->vs.up[1] = .0;
                    g3dwin->vs.up[2] = 1.0;
                    break;
                }
    }
}


void
g3d_resize_win(G3D_Window *win, float w, float h, float size) {
    win->vs.size = size;

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    g3d_set_win_camera(win->vs, .0,.0,.0,5*size, INIT_AZ, INIT_EL,.0,.0,1.0);
    g3d_save_win_camera(win->vs);
}




void
        g3d_set_win_fct_mobcam(G3D_Window *win, pp3d_matrix4 (*fct)(void)) {
    win->fct_mobcam = fct;
}


void
        g3d_set_win_drawer(G3D_Window *win, void (*fct)(void)) {
    win->fct_draw = fct;
}


void
        g3d_init_allwin_booleans(void) {
    G3D_Window *w = G3D_WIN;
    while (w) {
        w->vs.FILAIRE = 0;
        w->vs.CONTOUR = 0;
        w->vs.GOURAUD = 0;
        w = w->next;
    }
}

void g3d_set_light ( void )
{
    GLfloat light_position[] = { 20.0, -60.0, 100.0, 1.0 };
    GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat light_specular[] = { 0.1, 0.1, 0.1, 1.0 };
    double x1,y1,x2,y2,z1,z2,xmil=0.,ymil=0.,zmil=0.,ampl=0.,xampl=0.,yampl=0.,zampl=0.;
    p3d_vector4 Xc,Xw;

    G3D_Window *win = g3d_get_cur_win();

    calc_cam_param(win,Xc,Xw);

#ifdef HRI_PLANNER
    if(win->win_perspective){
        p3d_jnt *jntPt =  PSP_ROBOT->o[PSP_ROBOT->cam_body_index]->jnt;
        Xw[0]=PSP_ROBOT->cam_pos[0];
        Xw[1]=PSP_ROBOT->cam_pos[1];
        Xw[2]=PSP_ROBOT->cam_pos[2];
        Xw[3]=1;
        p3d_matvec4Mult(jntPt->abs_pos,Xw,Xc);
    }

#endif

    if(p3d_get_desc_number(P3D_ENV)) {
        p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
        xmil = (x2 + x1) / 2.;
        ymil = (y2 + y1) / 2.;
        zmil = (z2 + z1) / 2.;
        xampl = (x2 - x1) / 2.;
        yampl = (y2 - y1) / 2.;
        zampl = (z2 - z1) / 2.;
        ampl = 1.5 * sqrt(xampl * xampl + yampl * yampl + zampl * zampl);
        /*   light_position[0]=xmil; light_position[1]=ymil; light_position[2]=zmil+0.5*zampl;*/
        light_position[0] = Xc[0];
        light_position[1] = Xc[1];
        light_position[2] = Xc[2];

    }
#ifdef HRI_PLANNER
    if(win->win_perspective){// && (win->draw_mode != NORMAL)){
        glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT1);
        return;
    }
#endif
#ifdef PLANAR_SHADOWS
    light_position[0]= win->vs.lightPosition[0];
    light_position[1]= win->vs.lightPosition[1];
    light_position[2]= win->vs.lightPosition[2];
    light_position[3]= win->vs.lightPosition[3];
#endif

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 2./ampl);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

void qtG3DWindow::newG3dWindow()
{
	G3D_WIN = new G3D_Window;
	
	double x1,x2,y1,y2,z1,z2,ampl=0.;
	
	if(p3d_get_desc_number(P3D_ENV)) {
		p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
		ampl = MAX(MAX(x2-x1,y2-y1),z2-z1);
		x1 = .5*(x1+x2); y1 = .5*(y1+y2); z1 = .5*(z1+z2);
	}
	
	G3D_WIN->vs = g3d_init_viewer_state(ampl);

	/* Les parametres de la fenetre */
	G3D_WIN->fct_draw   = NULL;
	G3D_WIN->next       = NULL;
	G3D_WIN->fct_mobcam   = NULL;
	G3D_WIN->cam_frame  = &Id;
//	G3D_WIN->mcamera_but  = (void *)mcamera;
//	sprintf(G3D_WIN->name,"%s",name);
	printf("Window Size = %d\n",ampl);
	g3d_set_win_camera(G3D_WIN->vs, .0,.0,.0,2*ampl, INIT_AZ, INIT_EL,.0,.0,1.0);
	g3d_save_win_camera(G3D_WIN->vs);
	g3d_set_win_bgcolor(G3D_WIN->vs,1.0,1.0,1.0);
	G3D_WIN->next = G3D_WINDOW_LST;
	G3D_WINDOW_LST = G3D_WIN;
	G3D_WIN->vs.projection_mode = G3D_PERSPECTIVE;
	G3D_WIN->vs.transparency_mode = G3D_TRANSPARENT_AND_OPAQUE;
	
#ifdef PLANAR_SHADOWS
	if(ENV.getBool(Env::isCostSpace) && (GroundCostObj != NULL)){
		g3d_set_win_bgcolor(G3D_WIN->vs, 0, 0, 0);
	}
	else
	{
		g3d_set_win_bgcolor(G3D_WIN->vs, 1.0, 1.0, 0.8);
	}
	G3D_WIN->fct_draw2= NULL;
	G3D_WIN->fct_key1= NULL;
	G3D_WIN->fct_key2= NULL;
#endif
#ifdef HRI_PLANNER
	G3D_WIN->win_perspective = 0;
	G3D_WIN->point_of_view = 0;
	G3D_WIN->draw_mode = NORMAL;
#endif
	
	
//	G3D_WINDOW_CUR = G3D_WIN;
//	return(G3D_WIN);
	
}
