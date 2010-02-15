#include "qtG3DWindow.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Move3d-pkg.h"

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
//    cout << XformError << endl;
    static double ti = 0;
    double ts, tu;
    double tmax = p3d_get_tmax();

    ChronoTimes(&tu, &ts);
//    if (tu > ti) {
//      fl_check_forms();
//      ti = tu + 0.5;
//    }
    if ((p3d_GetStopValue()) || ((tu > tmax) && (tmax != 0))) {
      // STOP = FALSE;
      //  p3d_SetStopValue(FALSE);
      ti = 0;
      return false;
    }
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

G3D_Window *g3d_get_cur_win ( void )
{
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
}

void g3d_draw_frame(void)
{
    cout << XformError << endl;
}

void g3d_set_picking(unsigned int enabled)
{
    cout << XformError << endl;
}

int g3d_get_KCD_CHOICE_IS_ACTIVE()
{
//    cout << XformError << endl;
    return 0;
}

void g3d_refresh_allwin_active ( void )
{
    cout << XformError << endl;
}


int p3d_get_user_drawnjnt ( void )
{
//    cout << XformError << endl;
    return -1;
}

/* fonctions pour copier les donnees relies a la camera de     */
/* la structure G3D_Window de facon utilisable dans operations */
/* avec transformations homogenes                              */
static void
        get_lookat_vector(G3D_Window *win, p3d_vector4 Vec) {
    Vec[0] = win->x;
    Vec[1] = win->y;
    Vec[2] = win->z;
    Vec[3] = 1.0;
}

static void
        get_pos_cam_matrix(G3D_Window *win, p3d_matrix4 Transf) {
    /* Caution: ici on change les parametres de translation de la */
    /* matrix, les rest des elementes doivent etre initialises    */
    /* dans la fonction qu'appel                                  */
    Transf[0][3] = win->zo * (cos(win->az)*cos(win->el));
    Transf[1][3] = win->zo * (sin(win->az)*cos(win->el));
    Transf[2][3] = win->zo * sin(win->el);
}

void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw) {
    static p3d_matrix4 Txc = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    p3d_matrix4 m_aux;
    p3d_vector4 Xx;

#ifdef HRI_PLANNER
    if (win->point_of_view==0){
#endif
        get_lookat_vector(win, Xx);
        get_pos_cam_matrix(win, Txc);

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

/* fonction pour recalculer le vector 'up' de la camera */
/* quand on change la reference                         */
static void
        recalc_cam_up(G3D_Window *win, p3d_matrix4 transf) {
    p3d_vector4 v_up;
    int i;

    p3d_matvec4Mult(transf,win->up,v_up);
    for(i=0;i<4;i++) win->up[i] = v_up[i];
}

G3D_Window * qt_get_cur_g3d_win()
{
    return G3D_WIN;
}


void qt_calc_cam_param()
{
    p3d_vector4 Xc, Xw;
    p3d_vector4 up;

    //	  std::cout << "G3D_WINDOW_CUR->zo = "<< G3D_WINDOW_CUR->zo << std::endl;

//    calc_cam_param(G3D_WIN, Xc, Xw);
//
//    std::cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << std::endl;
//    std::cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << std::endl;
//    std::cout << up[0] << " " << up[1] << " " << up[2] << std::endl;

    Xc[0] = 5365.67;    Xc[1] = 1659.8;     Xc[2] = 4318.32;
    Xw[0] = 0;          Xw[1] = 0;          Xw[2] = 1250;
    up[0] = 6.95e-310;  Xw[1] = 6.95e-310;  up[2] = 6.95e-310;

    JimXc[0] = Xc[0];
    JimXc[1] = Xc[1];
    JimXc[2] = Xc[2];

    JimXw[0] = Xw[0];
    JimXw[1] = Xw[1];
    JimXw[2] = Xw[2];

    if (G3D_WIN)
    {
        p3d_matvec4Mult(*G3D_WIN->cam_frame, G3D_WIN->up, up);
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
        x = g3dwin->x;
        y = g3dwin->y;
        z = g3dwin->z;
        zo = g3dwin->zo;
        el = g3dwin->el;
        az = g3dwin->az;
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
            g3dwin->zo = (zo * i) / w + zo;
            if (g3dwin->zo < .0)
            {

                g3dwin->zo = .0;
            }
            //			std::cout << "g3dwin->zo = "<< g3dwin->zo << std::endl;
            break;
                case 512: /* angle */
                case 514:
                case 528:
                case 530:
                case 8720:
                case 16912:
                    g3dwin->az = (-2 * GAIN_AZ * i) / w + az;
                    if (g3dwin->az < .0)
                        g3dwin->az = 2 * M_PI;
                    if (g3dwin->az > 2 * M_PI)
                        g3dwin->az = .0;
                    g3dwin->el = (GAIN_EL * j) / w + el;
                    if (g3dwin->el < -M_PI / 2.0)
                        g3dwin->el = -M_PI / 2.0;
                    if (g3dwin->el > M_PI / 2.0)
                        g3dwin->el = M_PI / 2.0;
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
                    g3dwin->x = x + 2.0 * zo * cos(el) * sin(rotinc / 2.0) * sin(az
                                                                                 + rotinc / 2.0);
                    g3dwin->y = y - 2.0 * zo * cos(el) * sin(rotinc / 2.0) * cos(az
                                                                                 + rotinc / 2.0);
                    g3dwin->az = az + rotinc;

                    if (g3dwin->az < .0)
                        g3dwin->az = 2 * M_PI;
                    if (g3dwin->az > 2 * M_PI)
                        g3dwin->az = .0;
                    x_aux = g3dwin->x;
                    y_aux = g3dwin->y;
                    az_aux = g3dwin->az;
                    /* incline effects */
                    g3dwin->z = z - 2.0 * zo * sin(incinc / 2.0) * cos(el + incinc
                                                                       / 2.0);
                    g3dwin->x = x_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
                                                                           / 2.0) * cos(az_aux);
                    g3dwin->y = y_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
                                                                           / 2.0) * sin(az_aux);
                    g3dwin->el = el + incinc;
                    if (g3dwin->el < -M_PI / 2)
                        g3dwin->el = -M_PI / 2;
                    if (g3dwin->el > M_PI / 2)
                        g3dwin->el = M_PI / 2;
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
                    g3dwin->x = x + j * g3dwin->size / w;
                    /*       g3dwin->x = x + i*g3dwin->size/h; */
                    break;
                case 1536: /* origin y */
                case 1552:
                case 1538:
                case 9744:
                case 17936:
                    g3dwin->y = y + j * g3dwin->size / w;
                    /*       g3dwin->y = y + i*g3dwin->size/h; */
                    break;
                case 1280: /* origin z */
                case 1282:
                case 1296:
                case 9488:
                case 17680:
                    g3dwin->z = z + j * g3dwin->size / w;
                    /*       g3dwin->z = z + i*g3dwin->size/h; */
                    break;
                case 1792: /* reset up */
                    g3dwin->up[0] = .0;
                    g3dwin->up[1] = .0;
                    g3dwin->up[2] = 1.0;
                    break;
                }
    }
}

#ifdef PLANAR_SHADOWS
//-----------------------------------------------------------------------------
// Name: findPlane()
// Desc: find the plane equation given 3 points
//-----------------------------------------------------------------------------
void g3d_findPlane( GLdouble plane[4], GLdouble v0[3], GLdouble v1[3], GLdouble v2[3] )
{
    GLdouble vec0[3], vec1[3];

    // Need 2 vectors to find cross product
    vec0[0] = v1[0] - v0[0];
    vec0[1] = v1[1] - v0[1];
    vec0[2] = v1[2] - v0[2];

    vec1[0] = v2[0] - v0[0];
    vec1[1] = v2[1] - v0[1];
    vec1[2] = v2[2] - v0[2];

    // Find cross product to get A, B, and C of plane equation
    plane[0] =   vec0[1] * vec1[2] - vec0[2] * vec1[1];
    plane[1] = -(vec0[0] * vec1[2] - vec0[2] * vec1[0]);
    plane[2] =   vec0[0] * vec1[1] - vec0[1] * vec1[0];

    plane[3] = -(plane[0] * v0[0] + plane[1] * v0[1] + plane[2] * v0[2]);
}

// Construit les matrices de projection des ombres sur les plans du sol et des murs.
// Cette fonction doit être appelée chaque fois que la position de la lumière change.
void g3d_build_shadow_matrices(G3D_Window *win)
{
    buildShadowMatrix( win->floorShadowMatrix, win->lightPosition, win->floorPlane );
    buildShadowMatrix( win->wallShadowMatrix[0], win->lightPosition, win->wallPlanes[0] );
    buildShadowMatrix( win->wallShadowMatrix[1], win->lightPosition, win->wallPlanes[1] );
    buildShadowMatrix( win->wallShadowMatrix[2], win->lightPosition, win->wallPlanes[2] );
    buildShadowMatrix( win->wallShadowMatrix[3], win->lightPosition, win->wallPlanes[3] );
}
#endif


void
        g3d_resize_win(G3D_Window *win, float w, float h, float size) {
    win->size = size;

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    g3d_set_win_camera(win, .0,.0,.0,5*size, INIT_AZ, INIT_EL,.0,.0,1.0);
    g3d_save_win_camera(win);
}


void
        g3d_set_win_bgcolor(G3D_Window *win, float r, float v, float b) {
    win->bg[0] = r;
    win->bg[1] = v;
    win->bg[2] = b;
}

void
        g3d_set_win_floor_color(G3D_Window *win, float r, float v, float b) {
    win->floorColor[0] = r;
    win->floorColor[1] = v;
    win->floorColor[2] = b;
}


void
        g3d_set_win_camera(G3D_Window *win, float ox,float oy, float oz,
                           float dist, float az, float el,
                           float up0, float up1, float up2) {
    win->x = ox;
    win->y = oy;
    win->z = oz;
    win->zo = dist;
    win->az = az, win->el = el;
    win->up[0] = up0;
    win->up[1] = up1, win->up[2] = up2;
    win->up[3]=0.0;
}

void
        g3d_set_win_center(G3D_Window *win, float ox,float oy, float oz) {
    win->x = ox;
    win->y = oy;
    win->z = oz;
}


void
        g3d_save_win_camera(G3D_Window *win) {
    int i;

    win->sx = win->x;
    win->sy = win->y;
    win->sz = win->z;
    win->szo = win->zo;
    win->saz = win->az, win->sel = win->el;
    for(i=0;i<4;i++) win->sup[i] = win->up[i];
}


void
        g3d_restore_win_camera(G3D_Window *win) {
    int i;

    win->x = win->sx;
    win->y = win->sy;
    win->z = win->sz;
    win->zo = win->szo;
    win->az = win->saz, win->el = win->sel;
    for(i=0;i<4;i++) win->up[i] = win->sup[i];
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
        w->FILAIRE = 0;
        w->CONTOUR = 0;
        w->GOURAUD = 0;
        w = w->next;
    }
}

void g3d_set_light ( void )
{
    GLdouble light_position[] = { 20.0, -60.0, 100.0, 1.0 };
    GLdouble light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
    GLdouble light_specular[] = { 0.1, 0.1, 0.1, 1.0 };
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
    light_position[0]= win->lightPosition[0];
    light_position[1]= win->lightPosition[1];
    light_position[2]= win->lightPosition[2];
    light_position[3]= win->lightPosition[3];
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
    // g3d_new_win("Move3D",(int) w, (int) h,ampl);
    G3D_WIN = new G3D_Window;

    /* Les parametres de la fenetre */
//    G3D_WIN->form       = (void *)form;
//    G3D_WIN->canvas     = (void *)can;
    G3D_WIN->size       = size;
    G3D_WIN->FILAIRE = 0;
    G3D_WIN->CONTOUR = 0;
    G3D_WIN->GHOST = 0;
    G3D_WIN->BB = 0;
    G3D_WIN->GOURAUD = 0;
    G3D_WIN->ACTIVE = 1;
    G3D_WIN->list = -1;
    G3D_WIN->fct_draw   = NULL;
    G3D_WIN->next       = NULL;
    G3D_WIN->fct_mobcam   = NULL;
    G3D_WIN->cam_frame  = &Id;
//    G3D_WIN->mcamera_but  = (void *)mcamera;
//    sprintf(G3D_WIN->name,"%s",name);
    g3d_set_win_camera(G3D_WIN, .0,.0,.0,2*size, INIT_AZ, INIT_EL,.0,.0,1.0);
    g3d_save_win_camera(G3D_WIN);
    g3d_set_win_bgcolor(G3D_WIN,1.0,1.0,1.0);
    G3D_WIN->next = G3D_WINDOW_LST;
    G3D_WINDOW_LST = G3D_WIN;
#ifdef PLANAR_SHADOWS
    if(ENV.getBool(Env::isCostSpace) && (GroundCostObj != NULL)){
        g3d_set_win_bgcolor(G3D_WIN, 0, 0, 0);
    }
    else
    {
        g3d_set_win_bgcolor(G3D_WIN, 1.0, 1.0, 0.8);
    }
    G3D_WIN->fct_draw2= NULL;
    G3D_WIN->fct_key1= NULL;
    G3D_WIN->fct_key2= NULL;
    G3D_WIN->floorColor[0]= 0.5;
    G3D_WIN->floorColor[1]= 0.9;
    G3D_WIN->floorColor[2]= 0.9;
    G3D_WIN->displayJoints = 0;
    G3D_WIN->displayShadows = 0;
    G3D_WIN->displayWalls = 0;
    G3D_WIN->displayFloor = 0;
    G3D_WIN->displayTiles = 0;
#endif
#ifdef HRI_PLANNER
    G3D_WIN->win_perspective = 0;
    G3D_WIN->point_of_view = 0;
    G3D_WIN->draw_mode = NORMAL;
#endif

#ifdef PLANAR_SHADOWS
    //Les plans du sol et des murs vont être ajustés sur les coordonnées de
    //l'environment_box.
    double xmin, xmax, ymin, ymax, zmin, zmax;
    p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

    if( xmin>=xmax || ymin>=ymax || zmin>=zmax)
    {
        printf("%s: %d: g3d_new_win(): mauvais paramètres pour la commande p3d_set_env_box.\n\t", __FILE__, __LINE__);
        printf("Il faut les donner sous la forme xmin ymin zmin xmax ymax zmax.\n");
    }


    GLdouble v0[3], v1[3], v2[3];

    //plan du sol (normale selon Z):
    v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
    v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmin;
    g3d_findPlane(G3D_WIN->floorPlane, v0, v1, v2);

    //plan du premier mur (normale selon y):
    v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmax;
    v0[1]= ymin;      v1[1]= ymin;      v2[1]= ymin;
    v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
    g3d_findPlane(G3D_WIN->wallPlanes[0], v0, v1, v2);

    //plan du deuxième mur (normale selon -y):
    v0[0]= xmin;      v1[0]= xmax;      v2[0]= xmin;
    v0[1]= ymax;      v1[1]= ymax;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
    g3d_findPlane(G3D_WIN->wallPlanes[1], v0, v1, v2);

    //plan du troisième mur (normale selon x):
    v0[0]= xmin;      v1[0]= xmin;      v2[0]= xmin;
    v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymin;
    v0[2]= zmin;      v1[2]= zmin;      v2[2]= zmax;
    g3d_findPlane(G3D_WIN->wallPlanes[2], v0, v1, v2);


    //plan du quatrième mur (normale selon -x):
    v0[0]= xmax;      v1[0]= xmax;      v2[0]= xmax;
    v0[1]= ymin;      v1[1]= ymax;      v2[1]= ymax;
    v0[2]= zmin;      v1[2]= zmax;      v2[2]= zmin;
    g3d_findPlane(G3D_WIN->wallPlanes[3], v0, v1, v2);

    //positionnement de la lumière:
    G3D_WIN->lightPosition[0]= 0.5*(xmin+xmax);
    G3D_WIN->lightPosition[1]= 0.5*(ymin+ymax);
    G3D_WIN->lightPosition[2]= 0.9*(zmin+zmax);
    G3D_WIN->lightPosition[3]= 1.0;

    //Remplissage des matrices de projection sur les plans dans la direction de la lumière.
    //Si la position de la lumière est modifiée, il faudra mettre à jour les matrices.
    g3d_build_shadow_matrices(G3D_WIN);
#endif
}
