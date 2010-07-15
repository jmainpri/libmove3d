/*g3d_draw_ui
  This file contains function that permit to
  draw easily user interfaces using Xforms.
  author: Mokhtar GHARBI*/

/*Variables des fl_objects modifiees:
    u_ldata : définit quels dimentions l'utilisateur a définit.
              0 aucune
              1 la largeure
              2 la hauteur
              3 les largeure et la hauteur
    u_vdata : tableau de deux fl_objects representant l'fl_object précedents
              et l'fl_object suivant.
    child : premier fils de l'fl_object
    parent : fl_object pere de l'fl_object. Si l'fl_object se trouve directement
             sur le form cette variable est mise a null.
*/
#include "Graphic-pkg.h"
//function definition:

static int g3d_create_fl_object_on_form(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, FL_FORM** parent);
static int g3d_create_fl_object_on_frame(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, FL_OBJECT** parent);
static int g3d_place_fl_object_in_form(FL_OBJECT** obj, const char *label, int* x, int* y, FL_Coord w, FL_Coord h, FL_FORM** parent);
static int g3d_place_fl_object_in_frame(FL_OBJECT** obj, const char *label, int* x, int* y, FL_Coord w, FL_Coord h, FL_OBJECT** parent);
static int g3d_shift_childs(FL_OBJECT** obj);
static int g3d_shift_parents(FL_OBJECT** obj);
static int g3d_fl_object_max_width(FL_OBJECT** obj);
static int g3d_fl_object_max_height(FL_OBJECT** obj);
static int g3d_fl_object_has_good_width_height(FL_OBJECT** obj, const char *label, int x, int y, FL_Coord w, FL_Coord h, void* parent, int onRootFrame);
static void g3d_get_higher_fl_object_line_height(FL_OBJECT** obj, FL_OBJECT** previous);
static int g3d_label_has_return(const char *label);
// static int g3d_is_it_in_the_first_line(FL_OBJECT** obj, int y);
// static void g3d_shift_the_first_line(FL_OBJECT** obj, int offset, int *y);

#define FORM_SHIFT_X_DEFAULT 10
#define FORM_SHIFT_Y 10
#define FORM_NEXT_DIST_X_DEFAULT 5
#define FORM_NEXT_DIST_Y_DEFAULT 10
#define FRAME_SHIFT_X_DEFAULT 2
#define FRAME_SHIFT_Y_DEFAULT 2
#define FRAME_NEXT_DIST_X_DEFAULT 2
#define FRAME_NEXT_DIST_Y_DEFAULT 2

static int FORM_SHIFT_X = FORM_SHIFT_X_DEFAULT;
static int FORM_NEXT_DIST_X = FORM_NEXT_DIST_X_DEFAULT;
static int FORM_NEXT_DIST_Y = FORM_NEXT_DIST_Y_DEFAULT;
static int FRAME_SHIFT_X = FRAME_SHIFT_X_DEFAULT;
static int FRAME_SHIFT_Y = FRAME_SHIFT_Y_DEFAULT;
static int FRAME_NEXT_DIST_X = FRAME_NEXT_DIST_X_DEFAULT;
static int FRAME_NEXT_DIST_Y = FRAME_NEXT_DIST_Y_DEFAULT;
//Creating FL_OBJECT

/****************************************************************************/
/** \brief Create a from from given parameters.
 \param **form a pointer on the FL_FORM
 \param w the width of the form
 \param h the height of the form
 \param type the form type
 */
/****************************************************************************/

void g3d_create_form(FL_FORM** form, int w, int h, int type){
  if (w < 0 || h < 0){//si l'utilisateur ne veut pas spécifier les dimensions de la frame.
    return;
  }
  *form = fl_bgn_form(type,w,h);
  (*form)->u_vdata = (void*)malloc(sizeof(FL_OBJECT*));
  (*form)->u_vdata = NULL;
}

/****************************************************************************/
/** \brief Create a labled frame from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the frame.
 \param **obj pointer to the *FL_OBJECT
 \param type labelframe type
 \param w the labelframe Width
 \param h the labelframe Height
 \param label the labelframe label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the labelframe has a form as parent or not.
 \return 1 if the labelframe is created 0 if not.
 */
/****************************************************************************/
int g3d_create_labelframe(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_labelframe(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a button from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width and height of or button.
 \param **obj pointer to the *FL_OBJECT
 \param type button type
 \param w the button Width
 \param h the button_copy(  Height
 \param label the button label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the button has a form as parent or not.
 \return 1 if the button is created 0 if not.
 */
/****************************************************************************/
int g3d_create_button(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_button(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a choice from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width and height of or button.
 \param **obj pointer to the *FL_OBJECT
 \param type button type
 \param w the button Width
 \param h the button_copy(  Height
 \param label the button label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the button has a form as parent or not.
 \return 1 if the button is created 0 if not.
 */
/****************************************************************************/
int g3d_create_choice(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_choice(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}


/****************************************************************************/
/** \brief Create a checkbutton from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the checkbutton.
 \param **obj pointer to the *FL_OBJECT
 \param type checkbutton type
 \param w the checkbutton Width
 \param h the checkbutton Height
 \param label the checkbutton label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the checkbutton has a form as parent or not.
 \return 1 if the checkbutton is created 0 if not.
 */
/****************************************************************************/
int g3d_create_checkbutton(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_checkbutton(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a frame from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the frame.
 \param **obj pointer to the *FL_OBJECT
 \param type frame type
 \param w the frame Width
 \param h the frame Height
 \param label the frame label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the frame has a form as parent or not.
 \return 1 if the frame is created 0 if not.
 */
/****************************************************************************/
int g3d_create_frame(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_frame(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create an input from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the input.
 \param **obj pointer to the *FL_OBJECT
 \param type input type
 \param w the input Width
 \param h the input Height
 \param label the input label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the input has a form as parent or not.
 \return 1 if the input is created 0 if not.
 */
/****************************************************************************/
int g3d_create_input(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_input(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a valslider from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the valslider.
 \param **obj pointer to the *FL_OBJECT
 \param type valslider type
 \param w the valslider Width
 \param h the valslider Height
 \param label the valslider label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the valslider has a form as parent or not.
 \return 1 if the valslider is created 0 if not.
 */
/****************************************************************************/
int g3d_create_valslider(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_valslider(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a box from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the box.
 \param **obj pointer to the *FL_OBJECT
 \param type box type
 \param w the box Width
 \param h the box Height
 \param label the box label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the box has a form as parent or not.
 \return 1 if the box is created 0 if not.
 */
/****************************************************************************/
int g3d_create_box(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_box(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a counter from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the counter.
 \param **obj pointer to the *FL_OBJECT
 \param type counter type
 \param w the counter Width
 \param h the counter Height
 \param label the counter label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the counter has a form as parent or not.
 \return 1 if the counter is created 0 if not.
 */
/****************************************************************************/
int g3d_create_counter(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_counter(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}

/****************************************************************************/
/** \brief Create a xyplot from given parameters. Put w = -1 or h = -1
 if you don't want to specify the width or height of the xyplot.
 \param **obj pointer to the *FL_OBJECT
 \param type xyplot type
 \param w the xyplot Width
 \param h the xyplot Height
 \param label the xyplot label
 \param *parent pointer to the parent form if onRootFrame == 1 or on the 
 parent frame if onRootFrame == 0.
 \param onRootFrame specify if the xyplot has a form as parent or not.
 \return 1 if the xyplot is created 0 if not.
 */
/****************************************************************************/
int g3d_create_xyplot(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame){
  *obj = fl_add_xyplot(type,0,0,0,0,label);
  (*obj)->u_vdata = (void*)malloc(2*sizeof(FL_OBJECT*));
  ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
  ((FL_OBJECT**)(*obj)->u_vdata)[1] = NULL;
  if (onRootFrame){//si le parent est un form
    return g3d_create_fl_object_on_form(obj, type, w, h, label, (FL_FORM**) parent);
  }else{//if (onRootFrame)
    return g3d_create_fl_object_on_frame(obj, type, w, h, label, (FL_OBJECT**)parent);
  }//if (onRootFrame)
}
//Configuring primitivers Width, height, x, y

/****************************************************************************/
/** \brief Create an FL_OBJECT from given parameters on a form. Put w = -1 or h = -1
 if you don't want to specify the width or height of the FL_OBJECT.
 \param **obj pointer to the *FL_OBJECT
 \param type FL_OBJECT type
 \param w the FL_OBJECT Width
 \param h the FL_OBJECT Height
 \param label the FL_OBJECT label
 \param **parent pointer to the parent form.
 \return 1 if the FL_OBJECT is created 0 if not.
 */
/****************************************************************************/
static int g3d_create_fl_object_on_form(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, FL_FORM** parent){
  FL_OBJECT* previous = NULL;
  int x=0,y=0,defaultWidth = FORM_SHIFT_X, defaultHeight = FORM_SHIFT_Y;
  //si le fl_object a un label on cherche la taille du label pour que l'objet ne soit pas plus petit.
  if (strcmp(label,"") && !g3d_label_has_return(label) && (*obj)->label != NULL){
    fl_get_string_dimension((*obj)->lstyle, (*obj)->lsize, (*obj)->label, strlen((*obj)->label), &defaultWidth, &defaultHeight);
  }
  //tailles par défaut spéciales
  switch ((*obj)->objclass){
    case (FL_CHECKBUTTON):{
      if(h < 0){//si l'utilisateur ne veut pas specifier la hauteur.
        defaultHeight = 30;
      }
      if(w < 0){//si l'utilisateur ne veut pas specifier la largeure.
        defaultWidth = defaultWidth + 30;
      }
      break;
    }
    case(FL_INPUT):{
      defaultWidth = 20;
      break;
    }
  }
  (*obj)->u_ldata = 0;
  if (w < 0){//si l'utilisateur ne veut pas spécifier la largeure de l'objet.
    w = defaultWidth;
  }else{//if (w < 0)
    (*obj)->u_ldata += 1;
    w = w > defaultWidth?w:defaultWidth;
  }//if (w < 0)
  if (h < 0){//si l'utilisateur ne veut pas spécifier la hauteur de l'objet.
    h = defaultHeight;
  }else{//if (h < 0)
    (*obj)->u_ldata += 2;
    h = h > defaultHeight?h:defaultHeight;
  }//if (h < 0)
  if ((*parent)->u_vdata == NULL){//si c'est le premier élément du form.
    if (!g3d_place_fl_object_in_form(obj,label,&x,&y,w,h,parent)) return 0;
    //l'fl_objet ne depasse pas du form.
    if(!g3d_fl_object_has_good_width_height(obj,label,x,y,w,h,(*parent),1)){
      printf("Error : Can't create the frame %s : out of form bounds\n", label);
      g3d_fl_free_object(*obj);
      return 0;
    }
    ((FL_OBJECT**)(*obj)->u_vdata)[0] = NULL;
    (*parent)->u_vdata = (*obj);
  }else{//if (parent->u_vdata == NULL)
    previous = ((FL_OBJECT*)(*parent)->u_vdata);
    //aller au dernier élément ajouté dans le form.
    while(((FL_OBJECT**)previous->u_vdata)[1] != NULL){
      previous = ((FL_OBJECT**)previous->u_vdata)[1];
    }
    ((FL_OBJECT**)previous->u_vdata)[1] = (*obj);
    ((FL_OBJECT**)(*obj)->u_vdata)[0] = previous;
    if (!g3d_place_fl_object_in_form(obj,label,&x,&y,w,h,parent)) return 0;
  }//if (parent->u_vdata == NULL)
  (*obj)->x = x;
  (*obj)->y = y;
  (*obj)->w = w;
  (*obj)->h = h;
  return 1;
}

/****************************************************************************/
/** \brief Create an fl_object from given parameters on a frame. Put w = -1 or h = -1
 if you don't want to specify the width or height of the FL_OBJECT.
 \param **obj pointer to the *FL_OBJECT
 \param type FL_OBJECT type
 \param w the FL_OBJECT Width
 \param h the FL_OBJECT Height
 \param label the FL_OBJECT label
 \param **parent pointer to the parent frame.
 \return 1 if the FL_OBJECT is created 0 if not.
 */
/****************************************************************************/
static int g3d_create_fl_object_on_frame(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, FL_OBJECT** parent){
  FL_OBJECT* previous = NULL;
  FL_OBJECT* hierarchy = NULL;
  FL_OBJECT* hierarchyTmp = NULL;
  int newW = 0;
  int x=0,y=0,defaultWidth = FRAME_SHIFT_X*2, defaultHeight = FL_DEFAULT_SIZE+FRAME_SHIFT_Y*2;

  FRAME_SHIFT_Y = FRAME_SHIFT_Y_DEFAULT;

  //si le fl_object a un label on cherche la taille du label pour que l'objet ne soit pas plus petit.
  if (strcmp(label,"") && !g3d_label_has_return(label) && (*obj)->label != NULL){
    fl_get_string_dimension((*obj)->lstyle, (*obj)->lsize, (*obj)->label, strlen((*obj)->label), &defaultWidth, &defaultHeight);
  }
  //tailles par defaut speciales
  switch ((*obj)->objclass){
    case (FL_CHECKBUTTON):{
      if(h < 0 && defaultHeight < 30){//si l'utilisateur ne veux pas specifier la hauteur.
        defaultHeight = 30;
      }
      if(w < 0){//si l'utilisateur ne veux pas specifier la largeure.
        defaultWidth = (defaultWidth == FRAME_SHIFT_X*2)? 30: defaultWidth + 30;
      }
      break;
    }
    case(FL_INPUT):{
      if(defaultWidth  < 20){
        defaultWidth = 20;
      }
      break;
    }
  }

  (*obj)->parent = (*parent);
  (*obj)->u_ldata = 0;

  if (w < 0){//si l'utilisateur ne veut pas specifier la largeure de l'objet.
    w = defaultWidth;
  }else{//if (w < 0)
    (*obj)->u_ldata += 1;
    w = MAX(w, defaultWidth);
  }//if (w < 0)
  if (h < 0){//si l'utilisateur ne veut pas specifier la hauteur de l'objet.
    h = defaultHeight;
  }else{//if (h < 0)
    (*obj)->u_ldata += 2;
    h = MAX(h, defaultHeight);
  }//if (h < 0)
 
  if ((*parent)->child == NULL){//si c'est le premier element de la frame.
    (*parent)->child = (*obj);
  }else{
    previous = (*parent)->child;
    //aller au dernier element ajoute dans le form.
    while(((FL_OBJECT**)previous->u_vdata)[1] != NULL){
      previous = ((FL_OBJECT**)previous->u_vdata)[1];
    }
    ((FL_OBJECT**)previous->u_vdata)[1] = (*obj);
    ((FL_OBJECT**)(*obj)->u_vdata)[0] = previous;
  }
  g3d_place_fl_object_in_frame(obj,label,&x,&y,w,h,parent); //positionnement de l'object
  if(!g3d_fl_object_has_good_width_height(obj,label,x,y,w,h,(*parent),0)){
    if ((*parent)->u_ldata == 3){//si la taille de la frame parent a ete specifie.
      printf("Error : Can't create the frame %s at (x=%d, y=%d, w=%d, h=%d): out of parent frame bounds\n", label, x,y,w,h);
      g3d_fl_free_object(*obj);
    }else{//if ((*parent)->u_ldata == 3)
      //agrandir la frame parent si elle n'est pas assez grande.
      (*obj)->x = x;
      (*obj)->y = y;
      (*obj)->w = w;
      (*obj)->h = h;
      hierarchy = (*parent);
      hierarchyTmp = (*obj);
      do{
        if(hierarchy->u_ldata == 3){
          break;
        }
        if((hierarchy->u_ldata == 0 || hierarchy->u_ldata == 2)){//si la largeur n'est pas specifiee
             newW = (hierarchyTmp->x - hierarchy->x + hierarchyTmp->w + FRAME_SHIFT_X);
             hierarchy->w = newW > hierarchy->w ? newW: hierarchy->w;
        }
        if(hierarchy->u_ldata == 0 || hierarchy->u_ldata == 1){//si la hauteur n'est pas specifiee
          switch (hierarchyTmp->objclass){
            case (FL_SLIDER):{
              FRAME_SHIFT_Y = strcmp(hierarchyTmp->label,"")?14:FRAME_SHIFT_Y_DEFAULT;
              break;
            }
            case (FL_VALSLIDER):{
              FRAME_SHIFT_Y = strcmp(hierarchyTmp->label,"")?14:FRAME_SHIFT_Y_DEFAULT;
              break;
            }
            case (FL_COUNTER):{
              FRAME_SHIFT_Y = strcmp(hierarchyTmp->label,"")?14:FRAME_SHIFT_Y_DEFAULT;
              break;
            }
            case (FL_CHECKBUTTON):{
              FRAME_SHIFT_Y -= h/4 - FRAME_SHIFT_Y_DEFAULT;
              break;
            }
          }
          hierarchy->h = (hierarchyTmp->y - hierarchy->y + hierarchyTmp->h + FRAME_SHIFT_Y) > hierarchy->h? (hierarchyTmp->y - hierarchy->y + hierarchyTmp->h + FRAME_SHIFT_Y):hierarchy->h;
          FRAME_SHIFT_Y = FRAME_SHIFT_Y_DEFAULT;
        }
        hierarchyTmp = hierarchy;
      }while((hierarchy = hierarchy->parent));

      if(!g3d_shift_parents(obj)){
        printf("Error : Can't create the frame %s : can't shift parent\n", label);
        g3d_fl_free_object(*obj);
        return 0;
      }
      x = (*obj)->x;
      y = (*obj)->y;
    }//if ((*parent)->u_ldata == 1)
  }
  (*obj)->x = x;
  (*obj)->y = y;
  (*obj)->w = w;
  (*obj)->h = h;
  return 1;
}

//placing FL_OBJECT

/****************************************************************************/
/** \brief Place an FL_OBJECT on a form.
 \param **obj pointer to the current *FL_OBJECT
 \param *label pointer to the current FL_OBJECT's label
 \param *x pointer to X-coordinate
 \param *y pointer to Y-coordinate
 \param w the width
 \param h the height
 \param *parent pointer to the parent FL_OBJECT
 \return 1 if the FL_OBJECT is correctly placed 0 if not.
 */
/****************************************************************************/
static int g3d_place_fl_object_in_form(FL_OBJECT** obj, const char *label, int* x, int* y, FL_Coord w, FL_Coord h, FL_FORM** parent){
  FL_OBJECT* prev = ((FL_OBJECT**)(*obj)->u_vdata)[0];
  FL_OBJECT* tmp = ((FL_OBJECT*)(*parent)->u_vdata);
  int nbPrev = 0, newWidth = 0, newHeight = 0;
  //décalage spécifique a certaines classes
  FORM_SHIFT_X = FORM_SHIFT_X_DEFAULT;
  FORM_NEXT_DIST_X = FORM_NEXT_DIST_X_DEFAULT;
  switch ((*obj)->objclass){
    case (FL_INPUT):{
      if ((*obj)->label != NULL) {
        fl_get_string_dimension((*obj)->lstyle, (*obj)->lsize, label, strlen((*obj)->label), &newWidth, &newHeight);
      }
      FORM_SHIFT_X = strcmp(label,"")?newWidth:FORM_SHIFT_X_DEFAULT;
      FORM_NEXT_DIST_X = strcmp(label,"")?newWidth+14:FORM_NEXT_DIST_X_DEFAULT;
      break;
    }
     case (FL_CHECKBUTTON):{
      FORM_NEXT_DIST_Y -= h/4 - FORM_NEXT_DIST_X_DEFAULT;
      break;
    }
  }
  if(!prev){//c'est le premier element du form
    (*x) = FORM_SHIFT_X;
    (*y) = FORM_SHIFT_Y;
  }else{//if(!prev)
    //décalage spécifique a certaines classes
    FORM_NEXT_DIST_X = FORM_NEXT_DIST_X_DEFAULT;
    FORM_NEXT_DIST_Y = FORM_NEXT_DIST_Y_DEFAULT;
    switch (prev->objclass){
      case (FL_FRAME):{
        FORM_NEXT_DIST_X = FORM_NEXT_DIST_X_DEFAULT*2;
        break;
      }
      case (FL_LABELFRAME):{
        FORM_NEXT_DIST_X = FORM_NEXT_DIST_X_DEFAULT*2;
        break;
      }
      case (FL_COUNTER):{
        FORM_NEXT_DIST_Y = strcmp(prev->label,"")?14:FORM_NEXT_DIST_Y_DEFAULT;
        break;
      }
      case (FL_CHECKBUTTON):{
        FORM_NEXT_DIST_Y -= h/4 - FORM_NEXT_DIST_X_DEFAULT;
        break;
      }
    }
    (*x) = prev->x + prev->w + FORM_NEXT_DIST_X;
    (*y) = prev->y;
  }//if(!prev)
  if(!g3d_fl_object_has_good_width_height(obj,label,(*x),(*y),w,h,(*parent),1)){
    printf("Error : Can't create the frame %s : out of form bounds\n", label);
    g3d_fl_free_object(*obj);
    return 0;
  }
  //compter le nombre d'espace entre les fl_objects se trouvant sur la meme ligne.
  for(nbPrev = 0; tmp && tmp != (*obj); tmp = ((FL_OBJECT**)tmp->u_vdata)[1]){
    if(tmp->y == (*y)){nbPrev++;}
  }
  //revenir à la ligne
  if (((*x)-(*parent)->x-nbPrev*FORM_NEXT_DIST_X) + w > (*parent)->w - FORM_SHIFT_X*2){
    (*x) = FORM_SHIFT_X;
    if(!prev){//c'est le premier element du form
      (*y) = FORM_SHIFT_Y;
    }else{//if(!prev)
      g3d_get_higher_fl_object_line_height(obj, &prev);
      (*y) = prev->y + prev->h + FORM_NEXT_DIST_Y;
    }//if(!prev)
  }//if ((*x) + w > (*parent)->w - 20)
  //decalage de tous les enfants.
  if((*obj)->child){
    g3d_shift_childs(obj);
  }
  if (((*y) + h) > ((*parent)->h - FORM_SHIFT_Y)){
    printf("Error : Can't create the frame : %s .Increase form height\n", label);
    g3d_fl_free_object(*obj);
    return 0;
  }
  return 1;
}

/****************************************************************************/
/** \brief Place the given object in the given frame (parent).
 \param **obj pointer to the current *FL_OBJECT
 \param *label pointer to the current FL_OBJECT's label
 \param *x pointer to X-coordinate
 \param *y pointer to Y-coordinate
 \param w the width
 \param h the height
 \param *parent pointer to the parent FL_OBJECT
 \return 1 if the FL_OBJECT is good 0 if not.
 */
/****************************************************************************/
static int g3d_place_fl_object_in_frame(FL_OBJECT** obj, const char *label, int* x, int* y, FL_Coord w, FL_Coord h, FL_OBJECT** parent){
  FL_OBJECT* prev = ((FL_OBJECT**)(*obj)->u_vdata)[0];
  int newWidth = 0, newHeight = 0;
  //décalage spécifique a certaines classes
  FRAME_SHIFT_X = FRAME_SHIFT_X_DEFAULT;
  FRAME_SHIFT_Y = FRAME_SHIFT_Y_DEFAULT;
  FRAME_NEXT_DIST_X = FRAME_NEXT_DIST_X_DEFAULT;
  FRAME_NEXT_DIST_Y = FRAME_NEXT_DIST_Y_DEFAULT;
  switch ((*obj)->objclass){
    case (FL_INPUT):{
      if ((*obj)->label != NULL) {
        fl_get_string_dimension((*obj)->lstyle, (*obj)->lsize, label, strlen((*obj)->label), &newWidth, &newHeight);
      }
      FRAME_SHIFT_X = strcmp(label,"")?newWidth+10:FRAME_SHIFT_X_DEFAULT;
      FRAME_NEXT_DIST_X = strcmp(label,"")?newWidth+14:FRAME_NEXT_DIST_X_DEFAULT;
      break;
    }
    case (FL_CHECKBUTTON):{
      FRAME_NEXT_DIST_Y -= h/4 /*- FRAME_NEXT_DIST_Y_DEFAULT*/;
      break;
    }
    case (FL_LABELFRAME):{
      FRAME_SHIFT_X = 2*FRAME_SHIFT_X_DEFAULT;
      break;
    }
  }

  //c'est le premier element du form
  if(!prev){
    (*x) = FRAME_SHIFT_X + (*parent)->x;
    (*y) = FRAME_SHIFT_Y + (*parent)->y;
  }else{//if(!prev)
    (*x) = prev->x + prev->w + FRAME_NEXT_DIST_X;
    (*y) = prev->y;
    switch (prev->objclass){
      case (FL_CHECKBUTTON):{
        if ((*obj)->objclass != FL_CHECKBUTTON){
        (*y) += (prev->h)/4 - FRAME_NEXT_DIST_Y_DEFAULT;
        }
        break;
      }
    }
  }//if(!prev)
  //revenir à la ligne
  if ((prev) && ((*x)-(*parent)->x + w >= g3d_fl_object_max_width(obj))){
    (*x) = FRAME_SHIFT_X + (*parent)->x;
    g3d_get_higher_fl_object_line_height(obj, &prev);
    //décalage spécifique a certaines classes
    switch (prev->objclass){
      case (FL_SLIDER):{
        FRAME_NEXT_DIST_Y = strcmp(prev->label,"")?14:FRAME_NEXT_DIST_Y_DEFAULT;
        break;
      }
      case (FL_VALSLIDER):{
        FRAME_NEXT_DIST_Y = strcmp(prev->label,"")?14:FRAME_NEXT_DIST_Y_DEFAULT;
        break;
      }
      case (FL_COUNTER):{
        FRAME_NEXT_DIST_Y = strcmp(prev->label,"")?14:FRAME_NEXT_DIST_Y_DEFAULT;
        break;
      }
      case (FL_CHECKBUTTON):{
        FRAME_NEXT_DIST_Y -= prev->h/4 - FRAME_NEXT_DIST_Y_DEFAULT;
        break;
      }
    }
    switch ((*obj)->objclass){
      case(FL_LABELFRAME):{
          FRAME_NEXT_DIST_Y += strcmp(prev->label,"")?10:FRAME_NEXT_DIST_Y_DEFAULT;
        break;
      }
    }
    (*y) = prev->y + prev->h + FRAME_NEXT_DIST_Y;
  }//if ((*x) + w > (*parent)->w - 20)
  
  if((*obj)->child){
    g3d_shift_childs(obj);
  }
  FRAME_SHIFT_X = FRAME_SHIFT_X_DEFAULT;
  FRAME_SHIFT_Y = FRAME_SHIFT_Y_DEFAULT;
  FRAME_NEXT_DIST_X = FRAME_NEXT_DIST_X_DEFAULT;
  FRAME_NEXT_DIST_Y = FRAME_NEXT_DIST_Y_DEFAULT;
  return 1;
}

//Tree shifting

/****************************************************************************/
/** \brief Shift childs of the object given as parameter (recursive function).
 * \param **obj pointer to the current *FL_OBJECT
 * \return 1 if childs' of the FL_OBJECT are correctly shifted 0 if not.
 */
/****************************************************************************/
static int g3d_shift_childs(FL_OBJECT** obj){
  FL_OBJECT * next = (*obj);

  //si l'objet a un fils : continuer a déscendre.
  if ((*obj)->child){
    g3d_shift_childs(&((*obj)->child));
  }else{//if (next)
    //on place le premier élément de la liste.
    g3d_place_fl_object_in_frame(obj,(*obj)->label,&(*obj)->x,&(*obj)->y,(*obj)->w,(*obj)->h,&(*obj)->parent);
    //pour tous les freres de l'fl_object on fait de meme
    while ((next = ((FL_OBJECT**) next->u_vdata)[1])){
      if(next->child){
        g3d_shift_childs(&next);
      }
      g3d_place_fl_object_in_frame(&next,next->label,&(next->x),&(next->y),next->w,next->h,&(next->parent));
    }
  }//if (next) décaler les freres tout en regardant si ils n'ont pas de fils. 
  return 1;
}

/****************************************************************************/
/** \brief Shift parent of the object given as paramter (recurtsive function).
 \param **obj pointer to the current *FL_OBJECT
 \return 1 if parents' of the FL_OBJECT are correctly shifted 0 if not.
 */
/****************************************************************************/
static int g3d_shift_parents(FL_OBJECT** obj){
  FL_OBJECT *parent = ((*obj)->parent);
  FL_OBJECT *next = parent;
  if (parent->parent){//si le parent de l'object n'est pas sur le form.
    do{//décalage de tous les freres puis appel recursif pour les parents.
        if (!g3d_place_fl_object_in_frame(&next, next->label, &next->x, &next->y, next->w, next->h, &(next->parent))) return 0;
    } while((next = ((FL_OBJECT**)(next->u_vdata))[1]));
    if (!g3d_shift_parents(&parent)) return 0; //appel recursif pour les parents
  }else{//if (next->parent) egalement condition d'arret de la recursion.
    do{
        if (!g3d_place_fl_object_in_form(&next, next->label, &next->x, &next->y, next->w, next->h, &(next->form))) return 0;
    } while((next = ((FL_OBJECT**)(next->u_vdata))[1]));
  }//if (next->parent)
  return 1;
}

//TOOLS

/****************************************************************************/
/** \brief Get if there is a return '\n' in the label or not.
 \param label the string.
 \return 1 if the label contain a '\n', 0 if not.
 */
/****************************************************************************/
static int g3d_label_has_return(const char *label){
  int i = 0;
  char c = label[i];
  while(c!='\0'){
    c = label[i];
    if(c == '\n')return 1;
    i++;
  }
  return 0;
}

/****************************************************************************/
/** \brief Compute the max width that an fl_object can take
 \param **obj pointer to the *FL_OBJECT
 \return the max width or -1 if infinite.
 */
/****************************************************************************/
static int g3d_fl_object_max_width(FL_OBJECT** obj){
  if ((*obj)->parent){//l'objet n'est pas sur le form
     if ((*obj)->parent->u_ldata == 3 || (*obj)->parent->u_ldata == 1){//si l'utilisateur a spécifié la largeure
       return (*obj)->parent->w - (2*FRAME_SHIFT_X);
     }else{//if ((*obj)->parent->u_ldata == 3 || (*obj)->parent->u_ldata == 1)
      return g3d_fl_object_max_width(&(*obj)->parent) - (2*FRAME_SHIFT_X);
     }//if ((*obj)->parent->u_ldata == 3 || (*obj)->parent->u_ldata == 1)
  }else{//if ((*obj)->parent)
      return (*obj)->form->w - (2*FORM_SHIFT_X);
  }//if ((*obj)->parent)
}

/****************************************************************************/
/** \brief Compute the max height that an FL_OBJECT can take
 \param **obj pointer to the *FL_OBJECT
 \return the max height or -1 if infinite.
 */
/****************************************************************************/
static int g3d_fl_object_max_height(FL_OBJECT** obj){
  if ((*obj)->parent){//l'objet n'est pas sur le form
    if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2){//si l'utilisateur a spécifié la hauteur
      return (*obj)->h -(2*FRAME_SHIFT_Y);
    }else{//if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2)
      return g3d_fl_object_max_height(&(*obj)->parent) - (2*FRAME_SHIFT_Y);
    }//if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2)
  }else{//if ((*obj)->parent)
    if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2){//si l'utilisateur a spécifié les dimensions de l'object
       return (*obj)->h -(2*FORM_SHIFT_Y);
    }else{//if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2)
      return (*obj)->form->h - (2*FORM_SHIFT_Y);
    }//if ((*obj)->u_ldata == 3 || (*obj)->u_ldata == 2)
  }//if ((*obj)->parent)
}

/****************************************************************************/
/** \brief check if an FL_OBJECT has a good width and height or not.
 \param **obj pointer to the *FL_OBJECT
 \param *label pointer to the FL_OBJECT's label
 \param w the width
 \param h the height
 \param *parent pointer to the parent FL_OBJECT
 \param onRootFrame has a FL_FORM as parent
 \return 1 if the FL_OBJECT is good 0 if not.
 */
/****************************************************************************/
static int g3d_fl_object_has_good_width_height(FL_OBJECT** obj, const char *label, int x, int y, FL_Coord w, FL_Coord h, void* parent, int onRootFrame){
  FL_FORM* parentForm = NULL;
  FL_OBJECT* parentFrame = NULL;
  if (onRootFrame){//si le parent est un form
    parentForm = (FL_FORM*) parent;
    if (w > parentForm->w - FORM_SHIFT_X*2 || h > parentForm->h - FORM_SHIFT_Y*2){
      return 0;// les dimensions de frame sont plus grandes que celles de form.
    }/*else if(x + w > parentForm->w - 20 || y + h > parentForm->h - 20){
      return 0;// les dimensions de frame sont plus grandes que celles de form.
    }*/
  }else{
    parentFrame = (FL_OBJECT*) parent;
    if (w > parentFrame->w - FRAME_SHIFT_X*2 || h > parentFrame->h - FRAME_SHIFT_Y*2){
      return 0;// les dimensions de frame sont plus grandes que celles de la frame parent.
    }else if((x - parentFrame->x) + w > parentFrame->w - FRAME_SHIFT_X || (y - parentFrame->y) + h > parentFrame->h - FRAME_SHIFT_Y){
      return 0;// les dimensions de frame sont plus grandes que celles de form.
    }
  }
  return 1;
}

/****************************************************************************/
/** \brief Find the higer FL_OBJECT in the uper ligne.
 \param **obj pointer to the *FL_OBJECT
 \param **previous pointer to the previous *FL_OBJECT of obj
 */
/****************************************************************************/
static void g3d_get_higher_fl_object_line_height(FL_OBJECT** obj, FL_OBJECT** previous){
  int y = (*previous)->y, h = (*previous)->h;
  FL_OBJECT* tmp = (*previous);
  //on part du principe que tout les objets ayant la meme ordonnée sont dans un meme niveau hierarchique
  while (tmp && (tmp->y == y || tmp->y == 0) ){
    if(tmp->h > h){
      (*previous) = tmp;
      h = tmp->h;
    }
    tmp = ((FL_OBJECT**)tmp->u_vdata)[0];
  }
}

/****************************************************************************/
/** \brief Is this object in the first line ?
 \param **obj pointer to the *FL_OBJECT
 */
/****************************************************************************/
// static int g3d_is_it_in_the_first_line(FL_OBJECT** obj, int y){
//   FL_OBJECT* tmp = ((FL_OBJECT**)(*obj)->u_vdata)[0];
//   //on part du principe que tout les objets ayant la meme ordonnée sont dans un meme niveau hierarchique
//   while (tmp && (tmp->y == y || tmp->y == 0)){
//     tmp = ((FL_OBJECT**)tmp->u_vdata)[0];
//   }
//   if (!tmp){//premier element de la liste
//     return 1;
//   }
//   return 0;
// }

/****************************************************************************/
/** \brief Shift the first line
 \param **obj pointer to the *FL_OBJECT containing the first line to shift
 \param offset the size to add on Y axis
 \param *y the value to change
 */
/****************************************************************************/
// static void g3d_shift_the_first_line(FL_OBJECT** obj, int offset, int *y){
//   FL_OBJECT* child = (*obj)->child, *tmp = child;
//   int yTmp = (*y) + offset;
//   //on part du principe que tout les objets ayant la meme ordonnée sont dans un meme niveau hierarchique
//   /*while (tmp && (tmp->y == child->y || tmp->y == 0)){
//     tmp->y = yTmp;
//     tmp = ((FL_OBJECT**)tmp->u_vdata)[1];
//   }*/
//   (*y) = yTmp;
// }

//free

/****************************************************************************/
/** \brief Free the fl object and u_vdata and all his childs (recursive function)
 \param *obj pointer to the *FL_OBJECT
 */
/****************************************************************************/
void g3d_fl_free_object(FL_OBJECT* obj){
  FL_OBJECT * suiv = NULL;
  FL_OBJECT * tmp = NULL;
  //relinkage de la liste
  if(((FL_OBJECT**)obj->u_vdata)[0])
    ((FL_OBJECT**)(((FL_OBJECT**)obj->u_vdata)[0])->u_vdata)[1] = ((FL_OBJECT**)obj->u_vdata)[1];
  if(((FL_OBJECT**)obj->u_vdata)[1])
    ((FL_OBJECT**)(((FL_OBJECT**)obj->u_vdata)[1])->u_vdata)[0] = ((FL_OBJECT**)obj->u_vdata)[0];
  if (obj->parent && obj->parent->child == obj){//si l'objet n'est pas sur le form, et que c'est le premier element
    obj->parent->child = ((FL_OBJECT**)obj->u_vdata)[1]; //on donne le suivant de l'objet comme premier fils du parent
  }
  //suppression de tous les enfants
  if (obj->child != NULL){//si l'objet a des enfants.
    suiv = obj->child;
    while (suiv){//passer tous les fils en revue
      tmp = ((FL_OBJECT**)suiv->u_vdata)[1];
      g3d_fl_free_object(suiv);
      suiv = tmp;
    }
  }//si pas de fils supprimer l'objet
  free(obj->u_vdata);
  fl_free_object(obj);
}

/****************************************************************************/
/** \brief Destroy a from from
 \param **form a pointer on the FL_FORM
 */
/****************************************************************************/

void g3d_fl_free_form(FL_FORM* form){
  free(form->u_vdata);
  fl_free_form(form);
}
