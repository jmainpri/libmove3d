#include "Util-pkg.h"
#include "P3d-pkg.h"

#include "p3d/env.hpp"

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#include "Bio-pkg.h"

#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif

#include "GroundHeight-pkg.h"

#if defined( CXX_PLANNER )
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "cost_space.hpp"
#endif

#if defined(HRI_COSTSPACE) && defined(HRI_PLANNER_GUI)
#include "HRI_costspace/HRICS_HAMP.hpp"
#endif

#if defined(LIGHT_PLANNER)
#include "lightPlanner/proto/lightPlannerApi.h"
#include "lightPlanner/proto/ManipulationUtils.hpp"
#endif

extern void* GroundCostObj;
extern p3d_matrix4 Transfo;
static char DATA_FILE[200];
static char DATA_DIR[200] = "./";
int GOTO_READ = FALSE;
int START_READ = FALSE;

/***************************************************************/
int num_data_error ( char *msg );

int p3d_read_desc ( char *file )
{
	FILE *fdc;
	int ret;

	if ( ! ( fdc = fopen ( file, "r" ) ) )
	{
		PrintError ( ( "MP: p3d_read_desc: can't open %s\n", file ) );
		return ( FALSE );
	}
	fseek ( fdc, 0, 0 );

	ret = ( int ) strlen ( file );
	strcpy ( DATA_DIR, file );
	while ( file[--ret] != '/' && ret > 0 )
		;
	if ( ret == 0 )
		DATA_DIR[ret] = '\0';
	else
		DATA_DIR[++ret] = '\0';

//   ret = read_desc(fdc);
	ret = read_desc ( fdc, ( char* ) "", 1.0, 0 );

	strcpy ( DATA_FILE, file );
	fclose ( fdc );
	p3d_BB_init_BB0();
	return ( TRUE );
}

int p3d_read_macro ( char *namemac,char *nameobj,double scale )
{
	FILE *fd;
	char macro[200], macro2[200];
	char c_dir_name[200];
	char newNameAc[] = ".ground";
	int  c_sz=0;

	strcpy ( c_dir_name, DATA_DIR );
	c_sz = ( int ) strlen ( DATA_DIR );
	if ( DATA_DIR[c_sz-1] != '/' ) {strcat ( c_dir_name,"/" );}
	sprintf ( macro,"%sMACROS/",c_dir_name );

	strcat ( macro,namemac );
	if ( ENV.getBool ( Env::isCostSpace ) )
	{
		strcpy ( macro2,macro );
		strcat ( macro2,newNameAc );
		if ( ! ( fd=fopen ( macro2,"r" ) ) )
		{
			PrintError ( ( "MP: p3d_read_macro_ground: can't open %s\n",macro ) );
			return ( FALSE );
		}
		fd=fopen ( macro2,"r" );
		read_macro_ground ( fd,nameobj,scale );
		fclose ( fd );
#ifdef CXX_PLANNER		
		if(GroundCostObj)
		{
			std::cout << "Initializing the 2d costmap cost function" << std::endl;
			global_costSpace->addCost("costMap2D",boost::bind(computeIntersectionWithGround, _1));
			global_costSpace->setCost("costMap2D");
		}
#endif
	}
	if ( ! ( fd=fopen ( macro,"r" ) ) )
	{
		PrintError ( ( "MP: p3d_read_macro: can't open %s\n",macro ) );
		return ( FALSE );
	}
	read_desc ( fd, nameobj, scale, 1 );
//   read_macro(fd,nameobj,scale);

	fclose ( fd );
	return ( TRUE );
}

void p3d_set_directory ( char *dir )
{
	strcpy ( DATA_DIR, dir );
}

void p3d_get_directory ( char *dir )
{
	strcpy ( dir, DATA_DIR );
}

void p3d_get_filename ( char *fullname )
{
	strcpy ( fullname, DATA_FILE );
}

void p3d_set_filename ( char *fullname )
{
	strcpy ( DATA_FILE, fullname );
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Compute the position matrix with the position and the axe
 *        of a joint.
 *
 * Note: \a tabd[0], \a tabd[1], \a tabd[2] are respectively x, y, z.
 *       \a tabd[3], \a tabd[4], \a tabd[5] are the coordinate of the axis.
 *
 * \param tabd: an array of 6 doubles.
 *
 * \retval M: The position matrix.
 *
 * \internal
 */
void p3d_convert_axe_to_mat ( p3d_matrix4 M, double * tabd )
{
	p3d_vector3 axe1, axe2;
	double t2;

	/* axe1 the reference axis */
	axe1[0] = axe1[1] = 0.;
	axe1[2] = 1;
	axe2[0] = tabd[3];
	axe2[1] = tabd[4];
	axe2[2] = tabd[5];

	t2 = p3d_vectNorm ( axe2 );
	if ( t2 < EPS6 )
	{
		p3d_mat4Copy ( p3d_mat4IDENTITY, M );
	}
	else
	{
		p3d_vectScale ( axe2, axe2, 1 / t2 );
		p3d_mat4DeltaRot ( M, axe1, 0., axe2, 0. );
	}
	M[0][3] = tabd[0];
	M[1][3] = tabd[1];
	M[2][3] = tabd[2];
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Compute the position matrix with the previous joint position
        matrix and the D-H parameters of a joint.
 *
 * Note: \a tabd[0], \a tabd[1], \a tabd[2] \a tabd[3]
        are respectively a, alpha, r, theta
 *
 * \param tabd: an array of 4 doubles.
 *
 * \retval M: The position matrix.
 *
 * \internal
 */
void p3d_convert_dh_to_mat ( p3d_matrix4 M, double * tabd, p3d_jnt * prevJnt )
{
	p3d_matrix4 prevJntPos, passage;
	double a = tabd[0], alpha = tabd[1], r = tabd[2], theta = tabd[3];

	p3d_mat4Copy ( prevJnt->pos0, prevJntPos );
	passage[0][0] = cos ( theta );
	passage[0][1] = -sin ( theta );
	passage[0][2] = 0;
	passage[0][3] = a;
	passage[1][0] = cos ( alpha ) * sin ( theta );
	passage[1][1] = cos ( alpha ) * cos ( theta );
	passage[1][2] = -sin ( alpha );
	passage[1][3] = -r * sin ( alpha );
	passage[2][0] = sin ( alpha ) * sin ( theta );
	passage[2][1] = sin ( alpha ) * cos ( theta );
	passage[2][2] = cos ( alpha );
	passage[2][3] = r * cos ( alpha );
	passage[3][0] = 0.0;
	passage[3][1] = 0.0;
	passage[3][2] = 0.0;
	passage[3][3] = 1.0;

	p3d_mat4Mult ( prevJntPos, passage, M );
	return;
}

int read_desc_type ( FILE *fd, int *type )
{
	char name[256];

	if ( fscanf ( fd, "%s", name ) != 1 ) return ( FALSE );

	if ( ( strcmp ( name, "P3D_ENV" ) == 0 ) ||
	        ( strcmp ( name, "M3D_ENV" ) == 0 ) )   *type = P3D_ENV;
	else if ( ( strcmp ( name, "P3D_OBSTACLE" ) == 0 ) ||
	          ( strcmp ( name, "M3D_OBSTACLE" ) == 0 ) )    *type = P3D_OBSTACLE;
	else if ( ( strcmp ( name, "P3D_ROBOT" ) == 0 ) ||
	          ( strcmp ( name, "M3D_ROBOT" ) == 0 ) )    *type = P3D_ROBOT;
	else if ( ( strcmp ( name, "P3D_BODY" ) == 0 ) ||
	          ( strcmp ( name, "M3D_BODY" ) == 0 ) )     *type = P3D_BODY;

	else if ( p3d_rw_jnt_get_type_by_name ( name, ( p3d_type_joint * ) type ) ) {}
	else if ( ( strcmp ( name, "P3D_PPIVOT" ) == 0 ) ||
	          ( strcmp ( name, "M3D_PPIVOT" ) == 0 ) )    *type = P3D_PPIVOT;

	else if ( strcmp ( name, "Black" ) == 0 ) *type = Black;
	else if ( strcmp ( name, "tBlack" ) == 0 ) *type = tBlack;
	else if ( strcmp ( name, "Blue" ) == 0 ) *type = Blue;
	else if ( strcmp ( name, "tBlue" ) == 0 ) *type = tBlue;
	else if ( strcmp ( name, "Yellow" ) == 0 ) *type = Yellow;
	else if ( strcmp ( name, "tYellow" ) == 0 ) *type = tYellow;
	else if ( strcmp ( name, "Red" ) == 0 ) *type = Red;
	else if ( strcmp ( name, "tRed" ) == 0 ) *type = tRed;
	else if ( strcmp ( name, "Orange" ) == 0 ) *type = Orange;
	else if ( strcmp ( name, "tOrange" ) == 0 ) *type = tOrange;
	else if ( strcmp ( name, "Green" ) == 0 ) *type = Green;
	else if ( strcmp ( name, "tGreen" ) == 0 ) *type = tGreen;
	else if ( strcmp ( name, "White" ) == 0 ) *type = White;
	else if ( strcmp ( name, "tWhite" ) == 0 ) *type = tWhite;
	else if ( strcmp ( name, "Grey" ) == 0 ) *type = Grey;
	else if ( strcmp ( name, "tGrey" ) == 0 ) *type = tGrey;
	else if ( strcmp ( name, "Brown" ) == 0 ) *type = Brown;
	else if ( strcmp ( name, "tBrown" ) == 0 ) *type = tBrown;
	else if ( strcmp ( name, "Filaire" ) == 0 ) *type = Filaire;

	else if ( strcmp ( name, "Skin" ) == 0 ) *type = Skin;
	else if ( strcmp ( name, "tSkin" ) == 0 ) *type = tSkin;
	else if ( strcmp ( name, "Blue2" ) == 0 ) *type = Blue2;
	else if ( strcmp ( name, "tBlue2" ) == 0 ) *type = tBlue2;
	else if ( strcmp ( name, "DGrey" ) == 0 ) *type = DGrey;
	else if ( strcmp ( name, "tDGrey" ) == 0 ) *type = tDGrey;
	else if ( strcmp ( name, "DSkin" ) == 0 ) *type = DSkin;
	else if ( strcmp ( name, "tDSkin" ) == 0 ) *type = tDSkin;
	else if ( strcmp ( name, "tDBrown" ) == 0 ) *type = DBrown;
	else if ( strcmp ( name, "DBrown" ) == 0 ) *type = tDBrown;
	else if ( strcmp ( name, "Blue2" ) == 0 ) *type = Blue2;
	else if ( strcmp ( name, "tBlue2" ) == 0 ) *type = tBlue2;
	else if ( strcmp ( name, "DGreen" ) == 0 ) *type = DGreen;
	else if ( strcmp ( name, "tDGreen" ) == 0 ) *type = tDGreen;
	else if ( strcmp ( name, "Any" ) == 0 ) *type = Any;
	else if ( strcmp ( name, "Violet" ) == 0 ) *type = Violet;
	else if ( strcmp ( name, "tViolet" ) == 0 ) *type = tViolet;
	else if ( ( strcmp ( name, "P3D_GRAPHIC" ) == 0 ) ||
	          ( strcmp ( name, "M3D_GRAPHIC" ) == 0 ) )    *type = P3D_GRAPHIC;
	else if ( ( strcmp ( name, "P3D_REAL" ) == 0 ) ||
	          ( strcmp ( name, "M3D_REAL" ) == 0 ) )    *type = P3D_REAL;
	else if ( ( strcmp ( name, "P3D_GHOST" ) == 0 ) ||
	          ( strcmp ( name, "M3D_GHOST" ) == 0 ) )    *type = P3D_GHOST;
	else return ( FALSE );
	return ( TRUE );
}

int read_desc_hyp_type ( FILE *fd, int *n, int *type )
{
	char name[256];
	int EOL = 0, i = 1;
	char c;

	do
	{
		c = ( ( char ) ( getc ( fd ) ) );
		if ( c == ' ' )
		{
			continue;
		}
		if ( c == '\n' )
		{
			EOL = 1;
		}
		else
		{
			ungetc ( c, fd );
			if ( fscanf ( fd, "%s", name ) != 1 ) return ( FALSE );

			if ( ( strcmp ( name, "P3D_ENV" ) == 0 ) ||
			        ( strcmp ( name, "M3D_ENV" ) == 0 ) )   *type = P3D_ENV;
			else if ( ( strcmp ( name, "P3D_OBSTACLE" ) == 0 ) ||
			          ( strcmp ( name, "M3D_OBSTACLE" ) == 0 ) )    *type = P3D_OBSTACLE;
			else if ( ( strcmp ( name, "P3D_ROBOT" ) == 0 ) ||
			          ( strcmp ( name, "M3D_ROBOT" ) == 0 ) )    *type = P3D_ROBOT;
			else if ( ( strcmp ( name, "P3D_BODY" ) == 0 ) ||
			          ( strcmp ( name, "M3D_BODY" ) == 0 ) )     *type = P3D_BODY;

			else if ( p3d_rw_jnt_get_type_by_name ( name, ( p3d_type_joint * ) type ) ) {}
			else if ( ( strcmp ( name, "P3D_PPIVOT" ) == 0 ) ||
			          ( strcmp ( name, "M3D_PPIVOT" ) == 0 ) )    *type = P3D_PPIVOT;

			else if ( strcmp ( name, "Black" ) == 0 ) *type = Black;
			else if ( strcmp ( name, "tBlack" ) == 0 ) *type = Black;
			else if ( strcmp ( name, "Blue" ) == 0 ) *type = Blue;
			else if ( strcmp ( name, "tBlue" ) == 0 ) *type = Blue;
			else if ( strcmp ( name, "Yellow" ) == 0 ) *type = Yellow;
			else if ( strcmp ( name, "tYellow" ) == 0 ) *type = Yellow;
			else if ( strcmp ( name, "Red" ) == 0 ) *type = Red;
			else if ( strcmp ( name, "tRed" ) == 0 ) *type = Red;
			else if ( strcmp ( name, "Green" ) == 0 ) *type = Green;
			else if ( strcmp ( name, "tGreen" ) == 0 ) *type = Green;
			else if ( strcmp ( name, "White" ) == 0 ) *type = White;
			else if ( strcmp ( name, "tWhite" ) == 0 ) *type = White;
			else if ( strcmp ( name, "Grey" ) == 0 ) *type = Grey;
			else if ( strcmp ( name, "tGrey" ) == 0 ) *type = Grey;
			else if ( strcmp ( name, "Brown" ) == 0 ) *type = Brown;
			else if ( strcmp ( name, "tBrown" ) == 0 ) *type = Brown;
			else if ( strcmp ( name, "Filaire" ) == 0 ) *type = Filaire;
			else if ( strcmp ( name, "Skin" ) == 0 ) *type = Skin;
			else if ( strcmp ( name, "tSkin" ) == 0 ) *type = Skin;
			else if ( strcmp ( name, "Blue2" ) == 0 ) *type = Blue2;
			else if ( strcmp ( name, "tBlue2" ) == 0 ) *type = Blue2;
			else if ( strcmp ( name, "DGrey" ) == 0 ) *type = DGrey;
			else if ( strcmp ( name, "tDGrey" ) == 0 ) *type = DGrey;
			else if ( strcmp ( name, "DSkin" ) == 0 ) *type = DSkin;
			else if ( strcmp ( name, "tDSkin" ) == 0 ) *type = DSkin;
			else if ( strcmp ( name, "DBrown" ) == 0 ) *type = DBrown;
			else if ( strcmp ( name, "tDBrown" ) == 0 ) *type = DBrown;
			else if ( strcmp ( name, "Blue2" ) == 0 ) *type = Blue2;
			else if ( strcmp ( name, "tBlue2" ) == 0 ) *type = Blue2;
			else if ( strcmp ( name, "DGreen" ) == 0 ) *type = DGreen;
			else if ( strcmp ( name, "tDGreen" ) == 0 ) *type = DGreen;
			else if ( strcmp ( name, "Any" ) == 0 ) *type = Any;
			else if ( strcmp ( name, "Violet" ) == 0 ) *type = Violet;
			else if ( strcmp ( name, "tViolet" ) == 0 ) *type = Violet;
			else if ( ( strcmp ( name, "P3D_GRAPHIC" ) == 0 ) ||
			          ( strcmp ( name, "M3D_GRAPHIC" ) == 0 ) )    *type = P3D_GRAPHIC;
			else if ( ( strcmp ( name, "P3D_REAL" ) == 0 ) ||
			          ( strcmp ( name, "M3D_REAL" ) == 0 ) )    *type = P3D_REAL;
			else if ( ( strcmp ( name, "P3D_GHOST" ) == 0 ) ||
			          ( strcmp ( name, "M3D_GHOST" ) == 0 ) )    *type = P3D_GHOST;
			else if ( ( strcmp ( name, "P3D_GHOST_OBJECT" ) == 0 ) ) *type = P3D_GHOST_OBJECT;
			else if ( ( strcmp ( name, "P3D_REAL_OBJECT" ) == 0 ) ) *type = P3D_REAL_OBJECT;
			else if ( ( strcmp ( name, "P3D_GRAPHIC_OBJECT" ) == 0 ) ) *type = P3D_GRAPHIC_OBJECT;
			else if ( ( strcmp ( name, "P3D_DEACTIVATED_OBSTACLE" ) == 0 ) ) *type = P3D_DEACTIVATED_OBSTACLE; /* does not make sense => generate error message */
			else if ( ( strcmp ( name, "P3D_ACTIVATED_OBSTACLE" ) == 0 ) ) *type = P3D_ACTIVATED_OBSTACLE; /* does not make sense => generate error message */
			else if ( ( strcmp ( name, "P3D_ADDABLE_OBSTACLE" ) == 0 ) ) *type = P3D_ADDABLE_OBSTACLE;
			else return ( FALSE );
			i = i + 1;
		}
	}
	while ( !EOL );
	*n = i - 1;
	return ( TRUE );
}


/***************************************************************/

int read_desc_name ( FILE *fd, char *name )
{
	if ( fscanf ( fd, "%s", name ) != 1 )
		return ( FALSE );
	return ( TRUE );
}

/*--------------------------------------------------------------------------*/
/*! \brief Read a string between "" or a word
 *
 *  \param  fd: the file description
 *
 *  \retval name: the string read (must have allocated enough memory)
 *
 *  \return TRUE if a string can be read
 */
int read_desc_string ( FILE *fd, char *name )
{
	char c;
	int i, stop_s;

	i = 0;
	c = ' ';
	while ( ( fscanf ( fd, "%c", &c ) == 1 ) && ( ( c == ' ' ) || ( c == '\t' ) ) )
		;
	if ( c == ' ' )
	{
		return FALSE;
	}
	stop_s = ( c == '"' );
	if ( !stop_s )
	{
		name[i++] = c;
	}
	while ( ( fscanf ( fd, "%c", &c ) == 1 ) &&
	        ( ( stop_s && ( c != '"' ) ) || ( !stop_s && ( c != ' ' ) && ( c != '\t' ) ) ) &&
	        ( c != '\n' ) )
	{
		name[i++] = c;
	}
	name[i] = '\0';

	return ( i > 0 );
}
/***************************************************************/

int read_desc_double ( FILE *fd, int n, double *f )
{
	int i;

	for ( i = 0;i < n;i++ )
	{
		if ( fscanf ( fd, "%lf", &f[i] ) != 1 )
			return ( FALSE );
	}
	return ( TRUE );
}

int read_desc_line_double ( FILE *fd, int *n, double *dtab )
{
	int EOL = 0, i = 1;
	char c;

	do
	{
		c = ( ( char ) ( getc ( fd ) ) );
		if ( c == ' ' )
		{
			continue;
		}
		if ( c == '\n' )
		{
			EOL = 1;
		}
		else
		{
			ungetc ( c, fd );
			if ( fscanf ( fd, "%lf", &dtab[i-1] ) != 1 )
				return ( FALSE );
			i = i + 1;
		}
	}
	while ( !EOL );
	*n = i - 1;
	return ( TRUE );
}

/***************************************************************/

int read_desc_int ( FILE *fd, int n, int *itab )
{
	int i;

	for ( i = 0;i < n;i++ )
		if ( fscanf ( fd, "%d", &itab[i] ) != 1 )
			return ( FALSE );
	return ( TRUE );
}

int read_desc_line_int ( FILE *fd, int *n, int *itab )
{
	int EOL = 0, i = 1;
	char c;

	do
	{
		c = ( ( char ) ( getc ( fd ) ) );
		if ( c == ' ' )
		{
			continue;
		}
		if ( c == '\n' )
		{
			EOL = 1;
		}
		else
		{
			ungetc ( c, fd );
			if ( fscanf ( fd, "%d", &itab[i-1] ) != 1 )
				return ( FALSE );
			i = i + 1;
		}
	}
	while ( !EOL );
	*n = i - 1;
	return ( TRUE );
}

int read_desc_mat ( FILE *fd, p3d_matrix4 mat )
{
	int i, j;

	for ( i = 0 ; i <= 3 ; i++ )
	{
		for ( j = 0 ; j <= 3 ; j++ )
		{
			if ( fscanf ( fd, "%lf", &mat[i][j] ) != 1 )
				return ( FALSE );
		}
	}
	return ( TRUE );
}

int read_desc_mat_scaled ( FILE *fd, p3d_matrix4 mat, double scale )
{
	int i, j;

	for ( i = 0 ; i <= 3 ; i++ )
	{
		for ( j = 0 ; j <= 3 ; j++ )
		{
			if ( ( i < 3 ) && ( j == 3 ) )
			{
				if ( fscanf ( fd, "%lf", &mat[i][j] ) != 1 )
					return ( FALSE );
				mat[i][j] = scale * mat[i][j];
			}
			else
			{
				if ( fscanf ( fd, "%lf", &mat[i][j] ) != 1 )
					return ( FALSE );
			}
		}
	}
	return ( TRUE );
}
/***************************************************************/

int read_desc_error ( char *msg )
{
	PrintError ( ( "MP: p3d_read_desc: unknown function %s\n", msg ) );
	while ( p3d_inside_desc() )
		p3d_end_desc();
	return ( FALSE );
}

/***************************************************************/
/***************************************************************/
/***************************************************************/

/*--------------------------------------------------------------------------*/
/*!
 * \brief Read from a stream the description of a Move3D environment
 *
 *  This parser checks lines to reconize the description of Move3D objects.
 *  The instructions reconized are reumed in \ref file_description_page.
 *
 *  \param fd:  the file.
 *  \param nameobj the object name in case of macro
 *  \param scale the scale in case of macro
 *  \param fileType 0 p3d file, 1 macro file
 *  \return True if the file is correctly parsed
 */
int read_desc ( FILE *fd, char* nameobj, double scale, int fileType )
{
	char  fct[256];
	double dtab[1000], dtab2[1000], dtab3[1000], vtemp, *color_vect;
	configPt q;
	int   itab[200]; /* max nombre de sommets d'une face ==
            longueur liste itab */
	int   itab2[20], itab3[20], itab4[20], itab5[20];
	int   argnum[5];
	char  namefunct[256];
	int   type;
	char  child[256], parent[256];
	char  name[256], name2[256], name3[256], namemac[256], namecompl[256], gc;
	int n, i, nb_dof, nb_user_dof, nb_param, activated;
	p3d_poly *p_pos = NULL;
	p3d_matrix4 pos;
	pp3d_rob robotPt = NULL;
	int old_style = FALSE;
	double vel_max[JNT_NB_DOF_MAX], acc_max[JNT_NB_DOF_MAX], jerk_max[JNT_NB_DOF_MAX];

	while ( fscanf ( fd, "%s", fct ) != EOF )
	{
		if ( fct[0] == '#' )
		{
			/* comment in file: ignore the line ! */
			do
			{
				gc=getc ( fd );
			}
			while ( gc!='\n' && gc!='\r' && gc!=EOF );
			continue;
		}

		if ( strcmp ( fct, "p3d_begin_liste_poly" ) == 0 )
		{
			PrintWarning ( ( "p3d_read_desc : ERREUR : la fonction p3d_begin_liste_poly ne doit plus etre dans un fichier de donnees !!!\n" ) );
			return ( read_desc_error ( fct ) );
		}

		//##################### PRIMITIVES ######################

		/** \brief Start a description of an Environement, a Robot, an Obstacle, a Body or a Path. If we are in p3d file we have to always specify a name for the descrip started. Otherwise if is a macro file, we have to specify only a name for a Body description. We can't start an Environement description in a macro file.
		*/
		if ( ( strcmp ( fct, "p3d_beg_desc" ) == 0 ) || ( strcmp ( fct, "M3D_beg_desc" ) == 0 ) )
		{
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //if macro
			{
				if ( type == P3D_ENV )
				{
					PrintInfo ( ( "read_macro : ERREUR : on ne peut pas definir d'environnement par macro\n" ) );
					return ( read_desc_error ( fct ) );
				}
				strcpy ( namecompl, nameobj );
				if ( type == P3D_BODY )
				{
					if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
					strcat ( namecompl, "." );
					strcat ( namecompl, name );
				}
			}
			else
			{
				if ( !read_desc_name ( fd, namecompl ) )  return ( read_desc_error ( fct ) );
			}
			p3d_beg_desc ( type, namecompl );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_end_desc" ) == 0 ) || ( strcmp ( fct, "M3D_end_desc" ) == 0 ) )
		{
			p3d_end_desc();
			continue;
		}

		if ( strcmp ( fct,"p3d_CostEnvironment" ) == 0 )
		{
// TODO callback OOMOVE3D
#if defined( CXX_PLANNER )
			// initialize the cost function object.
			global_costSpace = new CostSpace();
			
			std::cout << "Initializing the dist to obst costmap cost function" << std::endl;
			global_costSpace->addCost("costDistToObst",boost::bind(computeDistanceToObstacles, _1));
			global_costSpace->setCost("costDistToObst");
#endif
			
			ENV.setBool ( Env::isCostSpace,true );
			continue;
		}

		if ( strcmp ( fct,"p3d_set_hri_zone_size" ) == 0 )
		{
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			ENV.setDouble ( Env::zone_size, dtab[0] );
			ENV.setBool ( Env::enableHri,true );
			continue;
		}
#if defined(LIGHT_PLANNER) 
		if ( strcmp ( fct, "p3d_set_object_to_carry" ) == 0 )
		{
			if ( !read_desc_int ( fd, 1, itab ) ) {
				printf("Error : The arm id is not specified");
				return ( read_desc_error ( fct ) );
			}
			
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			
			p3d_rob* MyRobot = p3d_get_robot_by_name_containing((const char *) "ROBOT" );
			
			if ( MyRobot == NULL) {
				printf("Error MyRobot = NULL\n");
			}
			
			p3d_set_object_to_carry_to_arm( MyRobot, itab[0] , name );
			//ArmManipulationData* armData = &(MyRobot->armManipulationData[itab[0]]);
			// Set the dist of the object to the radius of the carried object
			//MyRobot->curObjectJnt->dist = MyRobot->carriedObject->joints[1]->dist;
//			robotPt->isCarryingObject = TRUE;
			
			printf ( "Object To Carry = %s\n", name );
			continue;
		}
#endif
#if defined( PQP ) && defined( FK_CNTRT ) && defined( LIGHT_PLANNER ) 
		if ( strcmp ( fct, "p3d_set_fk_constraint" ) == 0 )
		{
			ENV.setBool(Env::startWithFKCntrt,true);
			//p3d_rob* MyRobot = p3d_get_robot_by_name ( ( char* ) "ROBOT" );
//			deactivateCcCntrts(MyRobot,-1);
			continue;
		}
#endif
		
   if (strcmp(fct, "p3d_set_camera_pos") == 0) {
			if ( !read_desc_double ( fd, 10, dtab ) ) return ( read_desc_error ( fct ) );
      //if (!p3d_read_string_n_double(&pos, 10, &dtab, &size_max_dtab)) return(read_desc_error(fct));
      g3d_load_saved_camera_params(dtab);      
      continue;
    }   

		if ( ( strcmp ( fct, "p3d_contact_surface" ) == 0 ) || ( strcmp ( fct, "M3D_contact_surface" ) == 0 ) )
		{
			XYZ_OBSTACLES->contact_surface= TRUE;
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_poly" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_poly" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_poly ( namecompl, P3D_REAL );
			}
			else
			{
				p3d_add_desc_poly ( namecompl, type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_end_desc_poly" ) == 0 ) || ( strcmp ( fct, "M3D_end_desc_poly" ) == 0 ) )
		{
			p3d_end_desc_poly();
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_vert" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_vert" ) == 0 ) )
		{
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			p3d_add_desc_vert ( scale*dtab[0], scale*dtab[1], scale*dtab[2] );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_face" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_face" ) == 0 ) )
		{
			if ( !read_desc_line_int ( fd, &n, &itab[0] ) ) return ( read_desc_error ( fct ) );
			p3d_add_desc_face ( itab, n );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_cube" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_cube" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_cube ( namecompl, scale*dtab[0], P3D_REAL );
			}
			else
			{
				p3d_add_desc_cube ( namecompl, scale*dtab[0], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_srect" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_srect" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 9, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_srect ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], P3D_REAL );
			}
			else
			{
				p3d_add_desc_srect ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_box" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_box" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_box ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_box ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_pyramid" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_pyramid" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 7, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			/* surete pour eviter des polyedres de largeur 0.... */
			if ( dtab[0] < EPS3 )
			{
				dtab[0] = 1.0;
			}
			if ( dtab[1] < EPS3 )
			{
				dtab[1] = 1.0;
			}
			if ( dtab[2] < EPS3 )
			{
				dtab[2] = 1.0;
			}
			if ( dtab[3] < EPS3 )
			{
				dtab[3] = 1.0;
			}
			if ( n == 0 )
			{
				p3d_add_desc_pyramid ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], P3D_REAL );
			}
			else
			{
				p3d_add_desc_pyramid ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_cylindre" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_cylindre" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_cylindre ( namecompl, scale*dtab[0], scale*dtab[1], P3D_REAL );
			}
			else
			{
				p3d_add_desc_cylindre ( namecompl, scale*dtab[0], scale*dtab[1], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_cylindre_oval" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_cylindre_oval" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_cylindre_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_cylindre_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_prisme" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_prisme" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_prisme ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], P3D_REAL );
			}
			else
			{
				p3d_add_desc_prisme ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_cone" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_cone" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			/* if(!read_desc_int(fd,1,itab)) return(read_desc_error(fct)); */
			itab[0] = 12;
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_cone ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_cone ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_snout" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_snout" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			/* if(!read_desc_int(fd,1,itab)) return(read_desc_error(fct)); */
			itab[0] = 12;
			if ( !read_desc_double ( fd, 5, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_snout ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], P3D_REAL );
			}
			else
			{
				p3d_add_desc_snout ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_skew_snout" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_skew_snout" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			/* if(!read_desc_int(fd,1,itab)) return(read_desc_error(fct)); */
			itab[0] = 12;
			if ( !read_desc_double ( fd, 9, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_skew_snout ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], P3D_REAL );
			}
			else
			{
				p3d_add_desc_skew_snout ( namecompl, itab[0], scale*dtab[0], scale*dtab[1], scale*dtab[2], scale*dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_rtorusslice" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_rtorusslice" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 4, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_rtorusslice ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], dtab[3], P3D_REAL );
			}
			else
			{
				p3d_add_desc_rtorusslice ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], dtab[3], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_ctorusslice" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_ctorusslice" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_ctorusslice ( namecompl, scale*dtab[0], scale*dtab[1], dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_ctorusslice ( namecompl, scale*dtab[0], scale*dtab[1], dtab[2], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_sweptrectslice" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_sweptrectslice" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 10, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_sweptrectslice ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], scale*dtab[9], P3D_REAL );
			}
			else
			{
				p3d_add_desc_sweptrectslice ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], dtab[3], scale*dtab[4], scale*dtab[5], scale*dtab[6], scale*dtab[7], scale*dtab[8], scale*dtab[9], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_sphere" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_sphere" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_sphere ( namecompl, scale*dtab[0], P3D_REAL );
			}
			else
			{
				p3d_add_desc_sphere ( namecompl, scale*dtab[0], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_half_sphere" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_half_sphere" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_half_sphere ( namecompl, scale*dtab[0], P3D_REAL );
			}
			else
			{
				p3d_add_desc_half_sphere ( namecompl, scale*dtab[0], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_sphere_shell" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_sphere_shell" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_sphere_shell ( namecompl, scale*dtab[0], scale*dtab[1], P3D_REAL );
			}
			else
			{
				p3d_add_desc_sphere_shell ( namecompl, scale*dtab[0], scale*dtab[1], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_oval" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_oval" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], type );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_half_oval" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_half_oval" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_half_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], P3D_REAL );
			}
			else
			{
				p3d_add_desc_half_oval ( namecompl, scale*dtab[0], scale*dtab[1], scale*dtab[2], type );
			}
			continue;
		}


		if ( ( strcmp ( fct, "p3d_add_desc_tore" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_tore" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_hyp_type ( fd, &n, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_add_desc_tore ( namecompl, scale*dtab[0], scale*dtab[1], P3D_REAL );
			}
			else
			{
				p3d_add_desc_tore ( namecompl, scale*dtab[0], scale*dtab[1], type );
			}
			continue;
		}

		//##################### JOINTS ######################

		if ( strcmp ( fct, "p3d_beg_desc_jnt" ) == 0 )
		{
			if ( !read_desc_type ( fd, &type ) )   return ( read_desc_error ( fct ) );
			if ( type == P3D_BASE )            return ( read_desc_error ( fct ) );
			if ( !p3d_env_beg_jnt_desc ( fd, ( p3d_type_joint ) type, scale ) ) {
			  printf("%s: %d: error in a p3d_env_beg_jnt_desc description.\n",__FILE__,__LINE__);
			  return ( read_desc_error ( fct ) );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_add_desc_jnt" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_add_desc_jnt" ) == 0 ) ||
		        ( strcmp ( fct, "p3d_add_desc_xyz_jnt" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_add_desc_xyz_jnt" ) == 0 ) )
		{
			if ( !read_desc_type ( fd, &type ) )   return ( read_desc_error ( fct ) );
			if ( type == P3D_BASE )            return ( read_desc_error ( fct ) );
			if ( type == P3D_PPIVOT )
			{
				type = P3D_ROTATE;
			}
			p3d_jnt_get_nb_param ( ( p3d_type_joint ) type, &nb_dof, &nb_param );
			if ( !read_desc_double ( fd, 6 + 3*nb_dof + nb_param, dtab ) )
				return ( read_desc_error ( fct ) );
			if ( ( strcmp ( fct, "M3D_add_desc_jnt" ) == 0 ) ||
			        ( strcmp ( fct, "M3D_add_desc_xyz_jnt" ) == 0 ) )
			{
				if ( !read_desc_name ( fd, parent ) )   return ( read_desc_error ( fct ) );
				if ( !read_desc_name ( fd, child ) )   return ( read_desc_error ( fct ) );
				itab[0] = p3d_get_parent_jnt_by_name ( parent );
			}
			else
			{
				if ( !read_desc_int ( fd, 1, itab ) )   return ( read_desc_error ( fct ) );
			}
			if ( !read_desc_double ( fd, 2*nb_dof, dtab2 ) )
			{
				/* no bounds specified for random sampling, take the same
				    as joint bounds */
				for ( i = 0; i < nb_dof; i++ )
				{
					dtab2[2*i]   = dtab[3*i+7];
					dtab2[2*i+1] = dtab[3*i+8];
				}
			}
			if ( fileType )  //is a macro file
			{
				if ( ( strcmp ( fct, "p3d_add_desc_xyz_jnt" ) == 0 ) ||
				        ( strcmp ( fct, "M3D_add_desc_xyz_jnt" ) == 0 ) )
				{
					rot_trans4 ( pos, dtab[0], dtab[1], dtab[2], dtab[3], dtab[4], dtab[5] );
				}
				else
				{
					p3d_convert_axe_to_mat ( pos, dtab );
				}
			}
			else
			{
				if ( ( strcmp ( fct, "p3d_add_desc_xyz_jnt" ) == 0 ) ||
				        ( strcmp ( fct, "M3D_add_desc_xyz_jnt" ) == 0 ) )
				{
					for ( i = 3; i < 6; i++ )
					{
						dtab[i] = DTOR ( dtab[i] );
					}
					p3d_mat4Pos ( pos, dtab[0], dtab[1], dtab[2], dtab[3], dtab[4], dtab[5] );
				}
				else
				{
					p3d_convert_axe_to_mat ( pos, dtab );
				}
			}
			for(int w=0; w<nb_dof;w++) {
			  vel_max[w] = 0.0;
			  acc_max[w] = 0.0;
			  jerk_max[w] = 0.0;
			}
			p3d_add_desc_jnt_deg ( ( p3d_type_joint ) type, pos, dtab + 6, itab[0], dtab2, scale, dtab3, vel_max, acc_max, jerk_max );
			continue;
		}

		//##################### ENV BOXES ######################

		if ( ( strcmp ( fct, "p3d_sel_desc_name" ) == 0 ) || ( strcmp ( fct, "M3D_sel_desc_name" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on ne peut pas selectionner un objet dans une macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( !read_desc_type ( fd, &type ) )   return ( read_desc_error ( fct ) );
				if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
				p3d_sel_desc_name ( type, name );
				continue;
			}
		}

		if ( ( strcmp ( fct, "p3d_set_env_box" ) == 0 ) || ( strcmp ( fct, "M3D_set_env_box" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on ne peut definir la boite d'un environnement dans une macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
				/* check the number of values specified */
				if ( n > 6 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "too many values specified, last ones are ignored\n" ) );
					n = 6;
				}
				if ( n < 6 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "too few values specified, last ones are set to 0\n" ) );
					for ( i = n; i < 6; i++ )
					{
						dtab[i] = 0;
					}
					n = 6;
				}
				if ( dtab[0] > dtab[1] )
				{
					vtemp = dtab[0];
					dtab[0] = dtab[1];
					dtab[1] = vtemp;
				}
				if ( dtab[2] > dtab[3] )
				{
					vtemp = dtab[2];
					dtab[2] = dtab[3];
					dtab[3] = vtemp;
				}
				if ( dtab[4] > dtab[5] )
				{
					vtemp = dtab[4];
					dtab[4] = dtab[5];
					dtab[5] = vtemp;
				}
				p3d_set_env_box ( dtab[0], dtab[1], dtab[2], dtab[3], dtab[4], dtab[5] );
				continue;
			}
		}

		if ( strcmp ( fct, "p3d_set_env_background_color" ) == 0 )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( n != 3 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "p3d_set_background_color requires 3 arguments (RGB values)\n" ) );
				}
				else
				{
					p3d_set_env_background_color ( dtab[0], dtab[1], dtab[2] );
				}
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_box" ) == 0 ) || ( strcmp ( fct, "M3D_set_robot_box" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on ne peut definir la boite d'un robot dans une macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "Don't use the robot box (and the joint0)\n" ) );

				if ( !read_desc_line_double ( fd, &n, dtab ) )
				{
					return ( read_desc_error ( fct ) );
				}
				/* check the number of values specified */
				if ( n == 7 )
					n = 6;
				if ( n > 12 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "too many values specified, last ones are ignored\n" ) );
					n = 12;
				}
				if ( n < 6 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( ": too few values specified, last ones are set to 0\n" ) );
					for ( i = n; i < 6; i++ )
					{
						dtab[i] = 0;
					}
					n = 6;
				}
				if ( n == 12 )
				{
					p3d_set_robot_box_deg ( dtab[0], dtab[1], dtab[2], dtab[3],
					                        dtab[4], dtab[5], dtab[6], dtab[7],
					                        dtab[8], dtab[9], dtab[10], dtab[11] );
					if ( dtab[0] > dtab[1] )
					{
						vtemp = dtab[0];
						dtab[0] = dtab[1];
						dtab[1] = vtemp;
					}
					if ( dtab[2] > dtab[3] )
					{
						vtemp = dtab[2];
						dtab[2] = dtab[3];
						dtab[3] = vtemp;
					}
					if ( dtab[4] > dtab[5] )
					{
						vtemp = dtab[4];
						dtab[4] = dtab[5];
						dtab[5] = vtemp;
					}
					if ( dtab[6] > dtab[7] )
					{
						vtemp = dtab[6];
						dtab[6] = dtab[7];
						dtab[7] = vtemp;
					}
					if ( dtab[8] > dtab[9] )
					{
						vtemp = dtab[8];
						dtab[8] = dtab[9];
						dtab[9] = vtemp;
					}
					if ( dtab[10] > dtab[11] )
					{
						vtemp = dtab[10];
						dtab[10] = dtab[11];
						dtab[11] = vtemp;
					}
				}
				else if ( n >= 8 )
				{
					if ( dtab[0] > dtab[1] )
					{
						vtemp = dtab[0];
						dtab[0] = dtab[1];
						dtab[1] = vtemp;
					}
					if ( dtab[2] > dtab[3] )
					{
						vtemp = dtab[2];
						dtab[2] = dtab[3];
						dtab[3] = vtemp;
					}
					if ( dtab[4] > dtab[5] )
					{
						vtemp = dtab[4];
						dtab[4] = dtab[5];
						dtab[5] = vtemp;
					}
					if ( dtab[6] > dtab[7] )
					{
						vtemp = dtab[6];
						dtab[6] = dtab[7];
						dtab[7] = vtemp;
					}
					dtab[10] = dtab[6];
					dtab[11] = dtab[7];
					dtab[6] = 0;
					dtab[7] = 0;
					dtab[8] = 0;
					dtab[9] = 0;

					p3d_set_robot_box_deg ( dtab[0], dtab[1], dtab[2], dtab[3],
					                        dtab[4], dtab[5], dtab[6], dtab[7],
					                        dtab[8], dtab[9], dtab[10], dtab[11] );
				}
				else if ( n == 6 )
				{
					if ( dtab[0] > dtab[1] )
					{
						vtemp = dtab[0];
						dtab[0] = dtab[1];
						dtab[1] = vtemp;
					}
					if ( dtab[2] > dtab[3] )
					{
						vtemp = dtab[2];
						dtab[2] = dtab[3];
						dtab[3] = vtemp;
					}
					if ( dtab[4] > dtab[5] )
					{
						vtemp = dtab[4];
						dtab[4] = dtab[5];
						dtab[5] = vtemp;
					}

					/* if 6 or less values specified, set Rz between -180 and 180,
					    fix Rx and Ry */
					dtab[6] = 0;
					dtab[7] = 0;
					dtab[8] = 0;
					dtab[9] = 0;
					dtab[10] = -180;
					dtab[11] = 180;

					p3d_set_robot_box_deg ( dtab[0], dtab[1], dtab[2], dtab[3],
					                        dtab[4], dtab[5], dtab[6], dtab[7],
					                        dtab[8], dtab[9], dtab[10], dtab[11] );
				}
				else
				{
					return ( read_desc_error ( fct ) );
				}
				continue;
			}
		}

		if ( ( strcmp ( fct, "p3d_set_min_bounds" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_min_bounds" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n <= 0 ) || ( n > nb_user_dof ) )
			{
				return ( read_desc_error ( fct ) );
			}

			p3d_set_user_config_min_bounds_deg ( robotPt, dtab, n );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_max_bounds" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_max_bounds" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n <= 0 ) || ( n > nb_user_dof ) )
			{
				return ( read_desc_error ( fct ) );
			}

			p3d_set_user_config_max_bounds_deg ( robotPt, dtab, n );
			continue;
		}

		// Usage : p3d_set_jnt_dofs_min_max <jntId> dof1min dof1max dof2min dof2max ...
		if ( strcmp ( fct, "p3d_set_jnt_dofs_min_max" ) == 0 )
		{
			if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( itab[0] <= 0 || itab[0] > robotPt->njoints ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, robotPt->joints[itab[0]]->dof_equiv_nbr*2, dtab ) ) return ( read_desc_error ( fct ) );
			for ( int j = 0; j < robotPt->joints[itab[0]]->dof_equiv_nbr*2; j+=2 )
			{
				p3d_jnt_set_dof_bounds_deg ( robotPt->joints[itab[0]], j/2, dtab[j], dtab[j+1] );
				p3d_jnt_set_dof_rand_bounds_deg ( robotPt->joints[itab[0]], j/2, dtab[j], dtab[j+1] );
			}
			continue;
		}
		//##################### STEERING AND LOCAL PATHS ######################

		if ( ( strcmp ( fct, "p3d_set_robot_radius" ) == 0 ) || ( strcmp ( fct, "M3D_set_robot_radius" ) == 0 ) )
		{
			PrintWarning ( ( "!!! WARNING %s: ", fct ) );
			PrintWarning ( ( "don't use this function anymore, use p3d_create_reeds_shepp_local_method\n" ) );
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				p3d_sel_desc_name ( P3D_ROBOT, nameobj );
			}
			dtab[0] *= scale;
			itab[0] = 0;  /* joint 0 */
#ifdef P3D_LOCALPATH
			p3d_create_reeds_shepp_local_method ( dtab, itab );
#endif
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_steering_method" ) == 0 ) || ( strcmp ( fct, "M3D_set_robot_steering_method" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on doit affecter la steering methode en dehors de la macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
				p3d_set_robot_steering_method ( name );
				continue;
			}
		}

		if ( strcmp ( fct, "p3d_create_trailer_local_method" ) == 0 )
		{
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, NB_JNT_TRAILER, itab ) )
			{
				return ( read_desc_error ( fct ) );
			}
			if ( read_desc_int ( fd, NB_COORD_TRAILER - NB_JNT_TRAILER, itab ) )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "Old format, now we only need the number of the base joint the trailer joint and the dc_ds joint\n" ) );
				robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
				itab[0] = p3d_robot_dof_to_jnt ( robotPt, itab[2], & ( itab[5] ) )->num;
				itab[1] = p3d_robot_dof_to_jnt ( robotPt, itab[3], & ( itab[5] ) )->num;
				itab[2] = p3d_robot_dof_to_jnt ( robotPt, itab[4], & ( itab[5] ) )->num;
			}
			for ( i = 0; i < 2; i++ )
			{
				dtab[i] *= scale;
			}
			if ( fileType )  //is a macro file
			{
				p3d_sel_desc_name ( P3D_ROBOT, nameobj );
			}
#ifdef P3D_LOCALPATH
			p3d_create_trailer_local_method ( dtab, itab );
#endif
			continue;
		}
    if (strcmp(fct, "p3d_create_trailer_local_method") == 0) {
      if (!read_desc_double(fd, 2, dtab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, NB_JNT_TRAILER, itab)) {
        return(read_desc_error(fct));
      }
      if (read_desc_int(fd, NB_COORD_TRAILER - NB_JNT_TRAILER, itab)) {
        PrintWarning(("!!! WARNING %s: ", fct));
        PrintWarning(("Old format, now we only need the number of the base joint the trailer joint and the dc_ds joint\n"));
        robotPt = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
        itab[0] = p3d_robot_dof_to_jnt(robotPt, itab[2], &(itab[5]))->num;
        itab[1] = p3d_robot_dof_to_jnt(robotPt, itab[3], &(itab[5]))->num;
        itab[2] = p3d_robot_dof_to_jnt(robotPt, itab[4], &(itab[5]))->num;
      }
      for (i = 0; i < 2; i++) {
        dtab[i] *= scale;
      }
      if (fileType) {//is a macro file
        p3d_sel_desc_name(P3D_ROBOT, nameobj);
      }
#ifdef P3D_LOCALPATH
      p3d_create_trailer_local_method(dtab, itab);
#endif
      continue;
    }

		if ( strcmp ( fct, "p3d_create_hilflat_local_method" ) == 0 )
		{
			if ( !read_desc_int ( fd, NB_JNT_HILFLAT, itab ) )
			{
				return ( read_desc_error ( fct ) );
			}
#ifdef P3D_LOCALPATH
			p3d_create_hilflat_local_method ( itab );
#endif
			continue;
		}
    if (strcmp(fct, "p3d_create_hilflat_local_method") == 0) {
      if (!read_desc_int(fd, NB_JNT_HILFLAT, itab)) {
        return(read_desc_error(fct));
      }
#ifdef P3D_LOCALPATH
      p3d_create_hilflat_local_method(itab);
#endif
      continue;
    }

		if ( strcmp ( fct, "p3d_create_reeds_shepp_local_method" ) == 0 )
		{
			if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, NB_JNT_REEDS_SHEPP, itab ) )
			{
				return ( read_desc_error ( fct ) );
			}
			if ( read_desc_int ( fd, 3 - NB_JNT_REEDS_SHEPP, itab + NB_JNT_REEDS_SHEPP ) )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "Old format, now we only need the number of the base joint\n" ) );
				robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
				itab[0] = p3d_robot_dof_to_jnt ( robotPt, itab[2], & ( itab[3] ) )->num;
			}
			dtab[0] *= scale;
#ifdef P3D_LOCALPATH
			p3d_create_reeds_shepp_local_method ( dtab, itab );
#endif
			continue;
		}
    if (strcmp(fct, "p3d_create_reeds_shepp_local_method") == 0) {
      if (!read_desc_double(fd, 1, dtab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, NB_JNT_REEDS_SHEPP, itab)) {
        return(read_desc_error(fct));
      }
      if (read_desc_int(fd, 3 - NB_JNT_REEDS_SHEPP, itab + NB_JNT_REEDS_SHEPP)) {
        PrintWarning(("!!! WARNING %s: ", fct));
        PrintWarning(("Old format, now we only need the number of the base joint\n"));
        robotPt = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
        itab[0] = p3d_robot_dof_to_jnt(robotPt, itab[2], &(itab[3]))->num;
      }
      dtab[0] *= scale;
#ifdef P3D_LOCALPATH
      p3d_create_reeds_shepp_local_method(dtab, itab);
#endif
      continue;
    }

//##################### POSITIONS ######################

		if ( ( strcmp ( fct, "p3d_set_robot_pos" ) == 0 ) || ( strcmp ( fct, "M3D_set_robot_pos" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on ne peut pas positionner un robot dans une macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
				/* if only 4 values specifed, old style specification */
				old_style = FALSE;
				if ( n > NDOF_BASE )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "the set pos need 6 dof, there are more than 6 dof, the next parameters are not used\n" ) );
				}
				if ( n < NDOF_BASE )
				{
					old_style = TRUE;
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "the set pos need 6 dof, here the parameters are completed\n" ) );
				}
				robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );

				q = p3d_alloc_config ( robotPt );

				for ( i = 0;i < n;i++ )
				{
					q[i] = dtab[i];
				}

				if ( old_style )
				{
					/* in case not enough configuration coordinates are specified,
					   the remaining ones are set to 0 */
					for ( i = n ; i < NDOF_BASE; i++ )
					{
						q[i] = 0;
					}
					/* in order to make coordinates consistant with 4 dof plateform,
					   set ry and rz to 0 */
					q[5] = q[3];
					q[3] = 0;
					q[4] = 0;
				}

				p3d_set_robot_pos_deg ( q );
				p3d_update_robot_pos();
				p3d_destroy_config ( robotPt, q );
				continue;
			}
		}

		if ( ( strcmp ( fct, "p3d_set_prim_pos" ) == 0 ) || ( strcmp ( fct, "M3D_set_prim_pos" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 6, dtab ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			{
				p3d_poly *poly;
				poly = p3d_poly_get_poly_by_name ( namecompl );
				p3d_set_prim_pos_deg ( poly, scale*dtab[0], scale*dtab[1], scale*dtab[2], dtab[3], dtab[4], dtab[5] );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_prim_pos_by_mat" ) == 0 ) || ( strcmp ( fct, "M3D_set_prim_pos_by_mat" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_mat_scaled ( fd, pos, scale ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			{
				p3d_poly *poly;
				poly = p3d_poly_get_poly_by_name ( namecompl );
				p3d_set_prim_pos_by_mat ( poly, pos );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_polyhedre_set_pos" ) == 0 ) || ( strcmp ( fct, "M3D_polyhedre_set_pos" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			p_pos = p3d_poly_get_poly_by_name ( namecompl );
			if ( !read_desc_mat_scaled ( fd, pos, scale ) ) return ( read_desc_error ( fct ) );
			p3d_set_poly_pos ( p_pos->poly, pos );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_obst_pos" ) == 0 ) || ( strcmp ( fct, "M3D_set_obst_pos" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on doit positionner l'objet macro en dehors de la macro\n" ) );
				return ( read_desc_error ( fct ) );
			}
			else
			{
				if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
				if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
				/* check the number of values specified */
				if ( n > 6 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "too many values specified, last ones are ignored\n" ) );
					n = 6;
				}
				if ( n < 6 )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "too few values specified, last ones are set to 0\n" ) );
					for ( i = n; i < 6; i++ )
					{
						dtab[i] = 0;
					}
					n = 6;
				}
				p3d_set_obst_pos ( name, dtab[0], dtab[1], dtab[2],
				                   DTOR ( dtab[3] ), DTOR ( dtab[4] ), DTOR ( dtab[5] ) );
				continue;
			}
		}

		if ( ( strcmp ( fct, "p3d_set_obst_pos_by_mat" ) == 0 ) || ( strcmp ( fct, "M3D_set_obst_pos_by_mat" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				PrintInfo ( ( "read_macro : ERREUR : on doit positionner l'objet macro en dehors de la macro\n" ) );
				continue;
			}
			else
			{
				if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
				if ( !read_desc_mat ( fd, pos ) ) return ( read_desc_error ( fct ) );
				p3d_set_obst_pos_by_mat ( name, pos );
				continue;
			}
		}

		if ( strcmp ( fct, "p3d_set_body_pos" ) == 0 || strcmp ( fct, "M3D_set_body_pos" ) == 0 )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
			/* check the number of values specified */
			if ( n > 6 )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "too many values specified, last ones are ignored\n" ) );
				n = 6;
			}
			if ( n < 6 )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "too few values specified, last ones are set to 0\n" ) );
				for ( i = n; i < 6; i++ )
				{
					dtab[i] = 0;
				}
				n = 6;
			}
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			p3d_set_body_pos ( namecompl, dtab[0], dtab[1], dtab[2], DTOR ( dtab[3] ), DTOR ( dtab[4] ), DTOR ( dtab[5] ) );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_body_pos_by_mat" ) == 0 ) || ( strcmp ( fct, "M3D_set_body_pos_by_mat" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_mat ( fd, pos ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			p3d_set_body_pos_by_mat ( namecompl, pos );
			continue;
		}

		if ( strcmp ( fct, "p3d_set_body_abs_pos" ) == 0 || strcmp ( fct, "M3D_set_body_abs_pos" ) == 0 )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
			/* check the number of values specified */
			if ( n > 6 )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "too many values specified, last ones are ignored\n" ) );
				n = 6;
			}
			if ( n < 6 )
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "too few values specified, last ones are set to 0\n" ) );
				for ( i = n; i < 6; i++ )
				{
					dtab[i] = 0;
				}
				n = 6;
			}
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			p3d_set_body_abs_pos ( namecompl, dtab[0], dtab[1], dtab[2], DTOR ( dtab[3] ), DTOR ( dtab[4] ), DTOR ( dtab[5] ) );
			continue;
		}

		if ( strcmp ( fct,"info_pos_by_mat" ) ==0 && fileType )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_mat_scaled ( fd,pos,scale ) ) return ( read_desc_error ( fct ) );
			p3d_mat4Copy ( pos, Transfo );
			continue;
		}
    if(strcmp(fct,"info_pos_by_mat")==0 && fileType) {
      if(!read_desc_name(fd,name))  return(read_desc_error(fct));
      if(!read_desc_mat_scaled(fd,pos,scale)) return(read_desc_error(fct));
#ifdef P3D_PLANNER
        p3d_mat4Copy (pos, Transfo);
#endif
      continue;
    }


		//##################### COLORS ######################

		if ( ( strcmp ( fct, "p3d_set_obst_color" ) == 0 ) || ( strcmp ( fct, "M3D_set_obst_color" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
			}
			else
			{
				if ( !read_desc_name ( fd, namecompl ) )  return ( read_desc_error ( fct ) );
			}
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( type == Any )
			{
				if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
				color_vect = MY_ALLOC ( double, 4 );
				color_vect[0] = dtab[0];
				color_vect[1] = dtab[1];
				color_vect[2] = dtab[2];
				if ( read_desc_double ( fd, 1, dtab ) )  //alpha
				{
					color_vect[3] = dtab[0];
				}
				else
				{
					color_vect[3] = 1;
				}
				p3d_set_obst_color ( namecompl, type, color_vect );
				MY_FREE ( color_vect, double, 4 );
			}
			else
			{
				color_vect = NULL;
				p3d_set_obst_color ( namecompl, type, color_vect );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_obst_poly_color" ) == 0 ) || ( strcmp ( fct, "M3D_set_obst_poly_color" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
			}
			else
			{
				if ( !read_desc_name ( fd, namecompl ) )  return ( read_desc_error ( fct ) );
			}
			if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( type == Any )
			{
				if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
				color_vect = MY_ALLOC ( double, 4 );
				color_vect[0] = dtab[0];
				color_vect[1] = dtab[1];
				color_vect[2] = dtab[2];
				if ( read_desc_double ( fd, 1, dtab ) )  //alpha
				{
					color_vect[3] = dtab[0];
				}
				else
				{
					color_vect[3] = 1;
				}
				p3d_set_obst_poly_color ( namecompl, itab[0], type, color_vect );
				MY_FREE ( color_vect, double, 4 );
			}
			else
			{
				color_vect = NULL;
				p3d_set_obst_poly_color ( namecompl, itab[0], type, color_vect );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_body_color" ) == 0 ) || ( strcmp ( fct, "M3D_set_body_color" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( type == Any )
			{
				if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
				color_vect = MY_ALLOC ( double, 4 );
				color_vect[0] = dtab[0];
				color_vect[1] = dtab[1];
				color_vect[2] = dtab[2];
				if ( read_desc_double ( fd, 1, dtab ) )  //alpha
				{
					color_vect[3] = dtab[0];
				}
				else
				{
					color_vect[3] = 1;
				}
				p3d_set_body_color ( namecompl, type, color_vect );
				MY_FREE ( color_vect, double, 4 );
			}
			else
			{
				color_vect = NULL;
				p3d_set_body_color ( namecompl, type, color_vect );
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_body_poly_color" ) == 0 ) || ( strcmp ( fct, "M3D_set_body_poly_color" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( type == Any )
			{
				if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
				color_vect = MY_ALLOC ( double, 4 );
				color_vect[0] = dtab[0];
				color_vect[1] = dtab[1];
				color_vect[2] = dtab[2];
				if ( read_desc_double ( fd, 1, dtab ) )  //alpha
				{
					color_vect[3] = dtab[0];
				}
				else
				{
					color_vect[3] = 1;
				}
				p3d_set_body_poly_color ( namecompl, itab[0], type, color_vect );
				MY_FREE ( color_vect, double, 4 );
			}
			else
			{
				color_vect = NULL;
				p3d_set_body_poly_color ( namecompl, itab[0], type, color_vect );
			}
			continue;
		}

		if (  strcmp ( fct, "p3d_make_body_deformable" ) == 0   )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
                        printf("%s: %d: p3d_make_body_deformable %s\n", __FILE__,__LINE__,namecompl);
                        p3d_adjust_deformable_body(namecompl);
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_prim_color" ) == 0 ) || ( strcmp ( fct, "M3D_set_prim_color" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_type ( fd, &type ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( type == Any )
			{
				if ( !read_desc_double ( fd, 3, dtab ) ) return ( read_desc_error ( fct ) );
				color_vect = MY_ALLOC ( double, 4 );
				color_vect[0] = dtab[0];
				color_vect[1] = dtab[1];
				color_vect[2] = dtab[2];
				if ( read_desc_double ( fd, 1, dtab ) )  //alpha
				{
					color_vect[3] = dtab[0];
				}
				else
				{
					color_vect[3] = 1;
				}
				{
					p3d_poly *poly;
					poly = p3d_poly_get_poly_by_name ( namecompl );
					p3d_poly_set_color ( poly, type, color_vect );
				}
				MY_FREE ( color_vect, double, 4 );
			}
			else
			{
				color_vect = NULL;
				{
					p3d_poly *poly;
					poly = p3d_poly_get_poly_by_name ( namecompl );
					p3d_poly_set_color ( poly, type, color_vect );
				}
			}
			continue;
		}


		if ( strcmp ( fct,"p3d_mark_as_hand_body" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			p3d_mark_body ( XYZ_ENV->cur_robot, name, P3D_HAND_PART );
			continue;
		}
		if ( strcmp ( fct,"p3d_mark_as_arm_body" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			p3d_mark_body ( XYZ_ENV->cur_robot, name, P3D_ARM_PART );
			continue;
		}
		if ( strcmp ( fct,"p3d_mark_as_base_body" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			p3d_mark_body ( XYZ_ENV->cur_robot, name, P3D_BASE_PART );
			continue;
		}
		if ( strcmp ( fct,"p3d_mark_as_finger_body" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			p3d_mark_body ( XYZ_ENV->cur_robot, name, P3D_FINGER_PART );
			continue;
		}
		if ( strcmp ( fct,"p3d_mark_as_fingertip_body" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			p3d_mark_body ( XYZ_ENV->cur_robot, name, P3D_FINGERTIP_PART );
			continue;
		}
		if ( strcmp ( fct,"p3d_set_distance_weight" ) == 0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 1, &dtab[0] ) )  return ( read_desc_error ( fct ) );
			p3d_set_distance_weight ( XYZ_ENV->cur_robot, name, dtab[0] );
			continue;
		}
		if ( strcmp ( fct,"p3d_set_finger_joints" ) == 0 )
		{
			double x;
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 1, &x ) )  return ( read_desc_error ( fct ) );
			p3d_set_distance_weight ( XYZ_ENV->cur_robot, name, x );
			continue;
		}


		//##################### PLANNING ######################

		if ( ( strcmp ( fct, "p3d_set_robot_goto" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_goto" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}

			p3d_set_ROBOT_GOTO_deg_to_rad ( q );
			GOTO_READ = TRUE;
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_goto_rads" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_goto_rads" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}

			p3d_set_ROBOT_GOTO ( q );
			GOTO_READ = TRUE;
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_ikSol_goto" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_ikSol_goto" ) == 0 ) )
		{
			if ( !read_desc_line_int ( fd, &n, itab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );

			if ( ( n != robotPt->cntrt_manager->ncntrts ) )
			{
				return ( read_desc_error ( fct ) );
			}
			robotPt->ikSolGoto = MY_ALLOC ( int, robotPt->cntrt_manager->ncntrts );
			for ( i = 0; i < robotPt->cntrt_manager->ncntrts; i++ )
			{
				robotPt->ikSolGoto[i] = itab[i];
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_current" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_current" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			
			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;
			PrintInfo(("set %s current pos\n",robotPt->name));

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}
			p3d_set_ROBOT_START_deg_to_rad ( q );
			START_READ = TRUE;
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_current_rads" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_current_rads" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}
			p3d_set_ROBOT_START ( q );
			START_READ = TRUE;
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_ikSol_current" ) == 0 ) ||
		        ( strcmp ( fct, "M3D_set_robot_ikSol_current" ) == 0 ) )
		{
			if ( !read_desc_line_int ( fd, &n, itab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );

			if ( ( n != robotPt->cntrt_manager->ncntrts ) )
			{
				PrintError ( ( "Check the number of constraints. There is %d constraint declared!!!\n", robotPt->cntrt_manager->ncntrts ) );
				return ( read_desc_error ( fct ) );
			}
			robotPt->ikSolPos = MY_ALLOC ( int, robotPt->cntrt_manager->ncntrts );
			for ( i = 0; i < robotPt->cntrt_manager->ncntrts; i++ )
			{
				robotPt->ikSolPos[i] = itab[i];
			}
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_robot_transition" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;
			if ( robotPt->nTransition > MAX_TRANSITION )
			{
				return ( read_desc_error ( fct ) );
			}
			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}
			p3d_pushRobotTransitionsDegToRad ( q, robotPt->nTransition );
			robotPt->nTransition++;
			p3d_destroy_config ( robotPt, q );
			continue;
		}
		//##################### COLLISION ######################

		if ( ( strcmp ( fct, "p3d_add_desc_rob_col_init" ) == 0 ) || ( strcmp ( fct, "M3D_add_desc_rob_col_init" ) == 0 ) )
		{
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
			}
			else
			{
				if ( !read_desc_name ( fd, namecompl ) )  return ( read_desc_error ( fct ) );
			}
			p3d_init_rob_col_activ ( namecompl );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_desactivate_col_check" ) == 0 ) || ( strcmp ( fct, "M3D_desactivate_col_check" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( strcmp ( name, "automatic" ) == 0 )
				p3d_desactivate_col_check_automatic();
			else if ( strcmp ( name, "automatic2" ) == 0 )
			{
				p3d_setAutocolDeep2 ( 1 );
				p3d_desactivate_col_check_automatic();
			}
			else if ( strcmp ( name, "all" ) == 0 )
				p3d_desactivate_col_check_all();
			else
			{
				if ( !read_desc_name ( fd, namecompl ) )  return ( read_desc_error ( fct ) );
				p3d_desactivate_col_check ( name, namecompl );
			}
			continue;
		}

#ifdef LIGHT_PLANNER
		if ( strcmp ( fct, "p3d_set_removable_bb_for_grasp" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) ); //number of joints
			if ( !read_desc_int ( fd, 4, itab ) ) return ( read_desc_error ( fct ) );//joints number
			if ( !p3d_set_removable_bb_for_grasp ( robotPt, argnum[0], itab ) ) return ( read_desc_error ( fct ) );//joint already declared
			continue;
		}

		if ( strcmp ( fct, "p3d_set_object_base_and_arm_constraints" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 4, argnum ) ) return ( read_desc_error ( fct ) ); //joints for the object, the base and the closedChain constraints
			robotPt->curObjectJnt = robotPt->joints[argnum[0]];
			robotPt->baseJnt = robotPt->joints[argnum[1]];
			robotPt->relativeZRotationBaseObject = DTOR ( argnum[2] );
			robotPt->nbCcCntrts = argnum[3];
			if ( !read_desc_int ( fd, robotPt->nbCcCntrts, argnum ) ) return ( read_desc_error ( fct ) ); //closedChain contraint ids
			robotPt->ccCntrts = MY_ALLOC ( p3d_cntrt*, robotPt->nbCcCntrts );
			for ( int i = 0; i < robotPt->nbCcCntrts; i++ )
			{ 
				if ( argnum[i] < robotPt->cntrt_manager->ncntrts )
				{
					robotPt->ccCntrts[i] = robotPt->cntrt_manager->cntrts[argnum[i]];
				}
				else
				{   printf("%s: %d: wrong argument to %s\n",__FILE__,__LINE__,fct);
					read_desc_error ( fct );
				}
			}
			continue;
		}
		if ( ( strcmp ( fct, "p3d_set_open_chain_config" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}
			configPt conf = NULL;
			conf = p3d_copy_config_deg_to_rad ( robotPt, q );
			p3d_copy_config_into ( robotPt, conf, & ( robotPt->openChainConf ) );
			p3d_destroy_config ( robotPt, conf );
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( ( strcmp ( fct, "p3d_set_closed_chain_config" ) == 0 ) )
		{
			if ( !read_desc_line_double ( fd, &n, dtab ) )
			{
				return ( read_desc_error ( fct ) );
			}

			nb_dof = p3d_get_robot_ndof();
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			nb_user_dof = robotPt->nb_user_dof;

			if ( ( n != nb_user_dof ) && ( n != nb_dof ) && ( n != nb_dof - 2 ) )
			{
				return ( read_desc_error ( fct ) );
			}

			q = p3d_get_robot_config_deg ( robotPt );

			if ( n == nb_user_dof ) /* User parameters */
			{
				p3d_copy_user_config_into_config ( robotPt, dtab, &q );
			}
			else
			{
				PrintWarning ( ( "!!! WARNING %s: ", fct ) );
				PrintWarning ( ( "old style. Now %s use only user parameters (not the 6 first dof)\n", fct ) );

				if ( n < nb_dof )
				{
					for ( i = 0; i < NDOF_BASE_TRANSLATE; i++ )
					{
						q[i] = dtab[i];
					}
					for ( i = NDOF_BASE - 1; i < nb_dof; i++ )
					{
						q[i] = dtab[i-2];
					}
				}
				else
				{
					for ( i = 0; i < nb_dof; i++ )
					{
						q[i] = dtab[i];
					}
				}
			}
			configPt conf = NULL;
			conf = p3d_copy_config_deg_to_rad ( robotPt, q );
			p3d_copy_config_into ( robotPt, conf, & ( robotPt->closedChainConf ) );
			p3d_destroy_config ( robotPt, conf );
			p3d_destroy_config ( robotPt, q );
			continue;
		}

		if ( strcmp ( fct, "p3d_set_arm_data" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt ) return ( read_desc_error ( fct ) );
			// the data ccntrtId, mlpGroupId, handType, virtualObjJntId
			if ( !read_desc_int ( fd, 7, itab ) ) 
			{
				printf("Error in p3d_set_arm_data argument size\n");
				return ( read_desc_error ( fct ) );
			}
			if ( !p3d_set_arm_data ( robotPt, itab ) ) 
			{
				printf("Error in p3d_set_arm_data()\n");
				return ( read_desc_error ( fct ) );//joint already declared
			}	
			continue;
		}

    if ( strcmp ( fct, "p3d_set_config_cost_threshold" ) == 0 )
    {
      robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
      if ( !robotPt ) return ( read_desc_error ( fct ) );
      // the data ccntrtId, mlpGroupId, handType, virtualObjJntId
      if ( !read_desc_double ( fd, 1, dtab ) ) return ( read_desc_error ( fct ) );
      robotPt->configCostThreshold = dtab[0];
      continue;
    }
		
#endif

#ifdef P3D_CONSTRAINTS
		//##################### CONSTRAINT ######################

		/* les arguments de p3d_constraint dans le ficher .p3d sont: */
		/*   - le nom de la contrainte                               */
		/*   - le numbre d'articulations pasives                     */
		/*   - les index des articulations pasives                   */
		/*   - le numbre d'articulations actives                     */
		/*   - les index des articulations actives                   */
		/*   - le numbre de constants                                */
		/*   - les valeurs constants                                 */
		/*   - le numbre d'autres parametres intieres                */
		/*   - les autres parametres intieres                        */
		if ( strcmp ( fct, "p3d_constraint" ) == 0 )
		{
			if ( !read_desc_name ( fd, namefunct ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[0], itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 1 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[1], itab2 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 2 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, argnum[2], dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 3 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[3], itab3 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 4 ) ) argnum[4] = 1;
			p3d_constraint ( namefunct, argnum[0], itab, argnum[1], itab2, argnum[2], dtab,
			                 argnum[3], itab3, -1, argnum[4] );
			continue;
		}
    /* les arguments de p3d_constraint dans le ficher .p3d sont: */
    /*   - le nom de la contrainte                               */
    /*   - le numbre d'articulations pasives                     */
    /*   - les index des articulations pasives                   */
    /*   - le numbre d'articulations actives                     */
    /*   - les index des articulations actives                   */
    /*   - le numbre de constants                                */
    /*   - les valeurs constants                                 */
    /*   - le numbre d'autres parametres intieres                */
    /*   - les autres parametres intieres                        */
    if (strcmp(fct, "p3d_constraint") == 0) {
      if (!read_desc_name(fd, namefunct)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[0], itab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 1)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[1], itab2)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 2)) return(read_desc_error(fct));
      if (!read_desc_double(fd, argnum[2], dtab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 3)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[3], itab3)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 4)) argnum[4] = 1;
      p3d_constraint(namefunct, argnum[0], itab, argnum[1], itab2, argnum[2], dtab,
                     argnum[3], itab3, -1, argnum[4]);
      continue;
    }

		if ( strcmp ( fct, "p3d_constraint_dof" ) == 0 )
		{
			if ( !read_desc_name ( fd, namefunct ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[0], itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[0], itab4 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 1 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[1], itab2 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[1], itab5 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 2 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, argnum[2], dtab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 3 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[3], itab3 ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum + 4 ) ) argnum[4] = 1;
			p3d_constraint_dof ( namefunct, argnum[0], itab, itab4,
			                     argnum[1], itab2, itab5, argnum[2], dtab,
			                     argnum[3], itab3, -1, argnum[4] );
			continue;
		}
    if (strcmp(fct, "p3d_constraint_dof") == 0) {
      if (!read_desc_name(fd, namefunct)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[0], itab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[0], itab4)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 1)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[1], itab2)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[1], itab5)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 2)) return(read_desc_error(fct));
      if (!read_desc_double(fd, argnum[2], dtab)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 3)) return(read_desc_error(fct));
      if (!read_desc_int(fd, argnum[3], itab3)) return(read_desc_error(fct));
      if (!read_desc_int(fd, 1, argnum + 4)) argnum[4] = 1;

      p3d_constraint_dof(namefunct, argnum[0], itab, itab4,
                         argnum[1], itab2, itab5, argnum[2], dtab,
                         argnum[3], itab3, -1, argnum[4]);
      continue;
    }

		/* les arguments de p3d_set_cntrt_Tatt  sont:                */
		/*   - l'index de la cntrt                                   */
		/*   - les elements des 3 premieres files de Tatt            */
		/*                                                           */
		/* NOTE: l'appel doit etre fait apres de celui a la          */
		/*       fonction de set de la cntrt correspondente          */
		if ( strcmp ( fct, "p3d_set_cntrt_Tatt" ) == 0 )
		{
			if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 12, dtab ) ) return ( read_desc_error ( fct ) );
			p3d_set_cntrt_Tatt ( itab[0], dtab );
			continue;
		}
    /* les arguments de p3d_set_cntrt_Tatt  sont:                */
    /*   - l'index de la cntrt                                   */
    /*   - les elements des 3 premieres files de Tatt            */
    /*                                                           */
    /* NOTE: l'appel doit etre fait apres de celui a la          */
    /*       fonction de set de la cntrt correspondente          */
    if (strcmp(fct, "p3d_set_cntrt_Tatt") == 0) {
      if (!read_desc_int(fd, 1, itab)) return(read_desc_error(fct));
      if (!read_desc_double(fd, 12, dtab)) return(read_desc_error(fct));
      p3d_set_cntrt_Tatt(itab[0], dtab);
      continue;
    }

		/* les arguments de p3d_set_cntrt_Tatt2  sont:                */
		/*   - l'index de la cntrt                                    */
		/*   - les elements des 3 premieres files de Tatt2            */
		/*                                                            */
		/* NOTE: l'appel doit etre fait apres de celui a la           */
		/*       fonction de set de la cntrt correspondente           */
		if ( strcmp ( fct,"p3d_set_cntrt_Tatt2" ) ==0 )
		{
			if ( !read_desc_int ( fd,1,itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd,12,dtab ) ) return ( read_desc_error ( fct ) );
			p3d_set_cntrt_Tatt2 ( itab[0],dtab );
			continue;
		}
    /* les arguments de p3d_set_cntrt_Tatt2  sont:                */
    /*   - l'index de la cntrt                                    */
    /*   - les elements des 3 premieres files de Tatt2            */
    /*                                                            */
    /* NOTE: l'appel doit etre fait apres de celui a la           */
    /*       fonction de set de la cntrt correspondente           */
    if(strcmp(fct,"p3d_set_cntrt_Tatt2")==0) {
      if(!read_desc_int(fd,1,itab)) return(read_desc_error(fct));
      if(!read_desc_double(fd,12,dtab)) return(read_desc_error(fct));
      p3d_set_cntrt_Tatt2(itab[0],dtab);
      continue;
    }
#endif
		/* les arguments de p3d_set_random_loop_generator  sont:     */
		/*   - rlg type                                              */
		/*   - index de la contrainte de la partie passive           */
		/*     IMP: faire d'abord le set des contraintes !!!         */
		/*   + si chain :                                            */
		/*     - la premiere et derniere articulations actives       */
		/*     - rmax de la partie passive  (optionnel)              */
		/*     - rmin de la partie passive  (optionnel)              */
		/*   + si base :                                             */
		/*     - le joint de base                                    */
		/*                                                           */
		/* NOTE: ces parametres sont provisoires                     */
		/*       ils peuvent etre calcules automatiquement           */
		/*       a partir d'autres informations                      */
		if ( strcmp ( fct, "p3d_set_random_loop_generator" ) == 0 )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, itab2 ) ) return ( read_desc_error ( fct ) );
			if ( strcmp ( name, "holonom_base" ) == 0 )
			{
				if ( !read_desc_int ( fd, 1, itab ) ) return ( read_desc_error ( fct ) );
				p3d_set_random_loop_generator ( name, itab2[0], itab[0], 0, 0.0, 0.0 );
			}
			else
			{
				if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
				if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
				if ( n == 2 )
					p3d_set_random_loop_generator ( name, itab2[0], itab[0], itab[1], dtab[0], dtab[1] );
				if ( n == 1 )
					p3d_set_random_loop_generator ( name, itab2[0], itab[0], itab[1], dtab[0], 0.0 );
				if ( n == 0 )
					p3d_set_random_loop_generator ( name, itab2[0], itab[0], itab[1], -1.0, 0.0 );
			}
			continue;
		}

		/* p3d_set_parallel_sys_data  arguments in .p3d file:
		    - type of system (fixed or mobile manipulator bases)
		    - index of the joint corresponding with the platform
		    - index of the joint corresponding with the base
		    - list of index of cntrts associated to manipulators
		*/
		/* NOTE : a new cntrt is generated
		          -> remind it for cntrt index !
		*/
		if ( strcmp ( fct, "p3d_set_parallel_sys" ) == 0 )
		{
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, argnum[0], itab2 ) ) return ( read_desc_error ( fct ) );
			p3d_set_parallel_sys ( name, itab[0], itab[1], argnum[0], itab2 );
			continue;
		}
		
#ifdef P3D_PLANNER
		/* p3d_set_frames_for_metric arguments in .p3d file:
		    - index of the joint associated with the reference frame
		      (set it to -1 if the reference is not mobile)
		    - index of the joint associated with the mobile frame
		*/
		if ( strcmp ( fct, "p3d_set_frames_for_metric" ) == 0 )
		{
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			//p3d_set_frames_for_metric(itab[0], itab[1]);
			p3d_SetRefAndMobFrames ( itab[0],itab[1] );
			continue;
		}
#endif
		
    /* p3d_set_frames_for_metric arguments in .p3d file:
        - index of the joint associated with the reference frame
          (set it to -1 if the reference is not mobile)
        - index of the joint associated with the mobile frame
    */
#ifdef P3D_PLANNER
    if (strcmp(fct, "p3d_set_frames_for_metric") == 0) {
      if (!read_desc_int(fd, 2, itab)) return(read_desc_error(fct));
      //p3d_set_frames_for_metric(itab[0], itab[1]);
      p3d_SetRefAndMobFrames(itab[0],itab[1]);
		continue;
    }
#endif

		/** \brief add a singular value to a specific constraint.
		    How to use :
		    p3d_set_singularity cntrtNum nJnt  jntNum1 Value1 Value2  jntNum2 Value3
		    with cntrtNum : the constraint number
		         nJnt     : the number of jnt for this constraint
		         jntNum1  : the first joint number
		         Value1   : the value for the first dof of the first joint
		         Value2   : the value for the second dof of the first joint
		         jntNum2  : the second joint number
		         Value3   : the value for the first dof of the second joint
		*/
#ifdef P3D_CONSTRAINTS
		if ( strcmp ( fct, "p3d_set_singularity" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt || !robotPt->cntrt_manager ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) ); //number of jnt for this sing
			if ( !robotPt->cntrt_manager->cntrts[itab[0]] ) return ( read_desc_error ( fct ) );
			for ( i = 0, n = 0; i < itab[1]; i++, n += nb_dof )
			{
				if ( !read_desc_int ( fd, 1, itab2+i ) ) return ( read_desc_error ( fct ) ); //number of each jnt
				nb_dof = ( robotPt->joints[itab2[i]] )->dof_equiv_nbr;
				if ( !read_desc_double ( fd, nb_dof, dtab + n ) ) return ( read_desc_error ( fct ) ); //number of each jnt
			}
			p3d_set_singularity ( itab[0], itab[1], itab2, dtab );
			continue;
		}
		/** \brief solution that a singularity connect.
		    How to use :
		    p3d_set_singular_rel cntrtNum singNum nPaires  class1 class2  class3 class 4
		    with cntrtNum : the constraint number
		         singNum  : the singularity number in this constraint
		         nPaires  : the number of paires that this singularity connect
		         class1   : the first solution class
		         class2   : the second solution class
		         class3   : the third solution class
		         class4   : the fourth solution class
		*/
		if ( strcmp ( fct, "p3d_set_singular_rel" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt || !robotPt->cntrt_manager ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 3, itab ) ) return ( read_desc_error ( fct ) ); //The cntrt, the sing and nb paires
			if ( !robotPt->cntrt_manager->cntrts[itab[0]] ) return ( read_desc_error ( fct ) );
			if ( ! ( robotPt->cntrt_manager->cntrts[itab[0]] )->singularities[itab[1]] ) return ( read_desc_error ( fct ) );
			if ( ( ( robotPt->cntrt_manager->cntrts[itab[0]] )->singularities[itab[1]] )->nRel != 0 ) return ( read_desc_error ( fct ) );
			for ( i = 0; i < itab[2]; i++ )
			{
				if ( !read_desc_int ( fd, 2, itab2+2*i ) ) return ( read_desc_error ( fct ) ); //solution class num
			}
			p3d_set_singular_rel ( itab[0], itab[1], itab[2],itab2 );
			continue;
		}
#endif
		//################## MULTILOCALPATH ##############
    /** \brief add a singular value to a specific constraint.
        How to use :
        p3d_set_singularity cntrtNum nJnt  jntNum1 Value1 Value2  jntNum2 Value3
        with cntrtNum : the constraint number
             nJnt     : the number of jnt for this constraint
             jntNum1  : the first joint number
             Value1   : the value for the first dof of the first joint
             Value2   : the value for the second dof of the first joint
             jntNum2  : the second joint number
             Value3   : the value for the first dof of the second joint
    */
    if (strcmp(fct, "p3d_set_singularity") == 0){
      robotPt = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
      if (!robotPt || !robotPt->cntrt_manager) return(read_desc_error(fct));
      if (!read_desc_int(fd, 2, itab)) return(read_desc_error(fct)); //number of jnt for this sing
      if (!robotPt->cntrt_manager->cntrts[itab[0]]) return(read_desc_error(fct));
      for(i = 0, n = 0; i < itab[1]; i++, n += nb_dof){
        if (!read_desc_int(fd, 1, itab2+i)) return(read_desc_error(fct)); //number of each jnt
        nb_dof = (robotPt->joints[itab2[i]])->dof_equiv_nbr;
        if (!read_desc_double(fd, nb_dof, dtab + n)) return(read_desc_error(fct)); //number of each jnt
      }
#ifdef P3D_CONSTRAINTS
      p3d_set_singularity(itab[0], itab[1], itab2, dtab);
#endif
      continue;
    }
    /** \brief solution that a singularity connect.
        How to use :
        p3d_set_singular_rel cntrtNum singNum nPaires  class1 class2  class3 class 4
        with cntrtNum : the constraint number
             singNum  : the singularity number in this constraint
             nPaires  : the number of paires that this singularity connect
             class1   : the first solution class
             class2   : the second solution class
             class3   : the third solution class
             class4   : the fourth solution class
    */
    if (strcmp(fct, "p3d_set_singular_rel") == 0){
      robotPt = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
      if (!robotPt || !robotPt->cntrt_manager) return(read_desc_error(fct));
      if (!read_desc_int(fd, 3, itab)) return(read_desc_error(fct)); //The cntrt, the sing and nb paires
      if (!robotPt->cntrt_manager->cntrts[itab[0]]) return(read_desc_error(fct));
      if (!(robotPt->cntrt_manager->cntrts[itab[0]])->singularities[itab[1]]) return(read_desc_error(fct));
      if (((robotPt->cntrt_manager->cntrts[itab[0]])->singularities[itab[1]])->nRel != 0) return(read_desc_error(fct));
      for(i = 0; i < itab[2]; i++){
        if (!read_desc_int(fd, 2, itab2+2*i)) return(read_desc_error(fct)); //solution class num
      }
#ifdef P3D_CONSTRAINTS
      p3d_set_singular_rel(itab[0], itab[1], itab[2],itab2);
#endif
      continue;
    }
	 //################## MULTILOCALPATH ##############
#if defined(MULTILOCALPATH)

		if ( strcmp ( fct, "p3d_multi_localpath_group" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt || !robotPt->cntrt_manager ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, &activated ) ) return ( read_desc_error ( fct ) ); //
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) ); //number of joints
			if ( !read_desc_int ( fd, argnum[0], itab ) )  //joints number
			{
				char tempChar;
				if ( fscanf ( fd, "%c", &tempChar ) != 1 ) return ( read_desc_error ( fct ) );
				if ( tempChar == '-' ) {fscanf ( fd, "%c", &tempChar );}
				if ( tempChar == '>' ) {
					int lastJnt = 0;
					if ( fscanf ( fd, "%d", &lastJnt ) != 1 ) {
						return ( read_desc_error ( fct ) );
					}
					else {
						for ( i = 1; i < lastJnt; i++ ) {
							itab[i] = itab[0] + i;
						}
					}
				}
				else {
					return ( read_desc_error ( fct ) );
				}
			}
			if ( !p3d_set_multi_localpath_group ( robotPt, argnum[0], itab, activated ) ) return ( read_desc_error ( fct ) );//joint already declared
			continue;
		}

		/* Need to be call immediatly after p3d_multi_localpath_group */
		if ( strcmp ( fct, "p3d_multi_localpath_data" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt || !robotPt->cntrt_manager ) return ( read_desc_error ( fct ) );
			if ( !read_desc_name ( fd, name3 ) )
			{
				return ( read_desc_error ( fct ) ); // the group name
			}
			if ( !read_desc_name ( fd, name ) )   // the group type
			{
				return ( read_desc_error ( fct ) );
			}
			if ( !read_desc_name ( fd, name2 ) )   // localpath type
			{
				return ( read_desc_error ( fct ) );
			}
			if ( strcmp ( name2,"Soft-Motion" ) ==0 ) {
				if ( !p3d_set_multi_localpath_data ( robotPt, name3, name, name2, dtab ) ) return ( read_desc_error ( fct ) );
				continue;
			}
			else if ( strcmp ( name2,"R&S+linear" ) ==0 ) {
				lm_reeds_shepp_str *rsheep;
				if ( !read_desc_double ( fd,1,dtab ) ) return ( read_desc_error ( fct ) );
				if ( !read_desc_int ( fd,NB_JNT_REEDS_SHEPP,itab ) ) { return ( read_desc_error ( fct ) ); }
				if ( read_desc_int ( fd,3-NB_JNT_REEDS_SHEPP,itab+NB_JNT_REEDS_SHEPP ) )
				{
					PrintWarning ( ( "!!! WARNING %s: ", fct ) );
					PrintWarning ( ( "Old format, now we only need the number of the base joint\n" ) );
					robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
					itab[0] = p3d_robot_dof_to_jnt ( robotPt, itab[2], & ( itab[3] ) )->num;
				}
				p3d_create_reeds_shepp_local_method ( dtab, itab );

				rsheep = lm_get_reeds_shepp_lm_param ( ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT ) );
				if ( !p3d_set_multi_localpath_data ( robotPt, name3, name, name2, dtab ) ) return ( read_desc_error ( fct ) );
				continue;
			}
			else if ( strcmp ( name2,"Linear" ) ==0 ) {
				if ( !p3d_set_multi_localpath_data ( robotPt, name3, name, name2, NULL ) ) return ( read_desc_error ( fct ) );
				continue;
			}
			else {
				printf ( "localpath not declared for multiLocalPath\n" );
				return ( num_data_error ( fct ) );
			}
		}
#endif

//##################### BIO ######################

		/* bio_set_loop arguments in .p3d file:
		    - index of the first joint in the loop
		    - index of the last joint in the loop
		*/
#ifdef BIO
		if ( strcmp ( fct, "bio_set_loop" ) == 0 )
		{
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			bio_set_loop ( itab[0], itab[1] );

			continue;
		}

		/* bio_set_bkb_Hbond arguments in .p3d file:
		    - index of the joint associated with the N(O) atom
		    - index of the joint associated with the O(N) atom
		    - index of the main constraint of the loop
		    - minimum value for the distance N-O
		    - maximum value for the distance N-O
		    - minimum Hbond angle
		    - maximum Hbond angle
		*/
		if ( strcmp ( fct, "bio_set_bkb_Hbond" ) == 0 )
		{
			if ( !read_desc_int ( fd, 3, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 4, dtab ) )
			{
				printf ( "ERROR : bio_set_bkb_Hbond : too few arguments\n" );
				return ( read_desc_error ( fct ) );
			}

			bio_set_bkb_Hbond ( itab[0], itab[1], itab[2], dtab[0], dtab[1], dtab[2], dtab[3] );
			continue;
		}

		/* bio_set_diS_bond arguments in .p3d file:
		    - index of the joint associated with the 1st S atom
		    - index of the joint associated with the 2nd S atom
		    - minimum value for the distance S-S
		    - maximum value for the distance S-S
		*/
		if ( strcmp ( fct, "bio_set_diS_bond" ) == 0 )
		{
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 2, dtab ) ) return ( read_desc_error ( fct ) );
			bio_set_diS_bond ( itab[0], itab[1], dtab[0], dtab[1] );
			continue;
		}

		/* bio_track_loop arguments in .p3d file:
		    - index of the first joint in the loop
		    - index of the last joint in the loop
		    - name of file containing trajectory of Fb and Fe
		*/
		if ( strcmp ( fct, "bio_track_loop" ) == 0 )
		{
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			bio_set_track_loop_from_FbFe_sequence ( itab[0], itab[1], name );
			continue;
		}


		// OLD FUNCTIONS //////////////////////////////////////////////////////////////
		/* bio_set_loop_ori arguments in .p3d file:
		    - index of the first joint in the loop
		    - index of the last joint in the loop
		    - index of the reference freeflying joint
		*/
		/* for BIO */
		if ( strcmp ( fct, "bio_set_loop_ori" ) == 0 )
		{
			if ( !read_desc_int ( fd, 3, itab ) ) return ( read_desc_error ( fct ) );
			bio_set_loop_ori ( itab[0], itab[1], itab[2] );
			continue;
		}

		/* bio_set_loop_pos arguments in .p3d file:
		    - index of the first joint in the loop
		    - index of the last joint in the loop
		    - position (x,y,z) of last atom
		    - position error
		*/
		/* for BIO */
		if ( strcmp ( fct, "bio_set_loop_pos" ) == 0 )
		{
			if ( !read_desc_int ( fd, 2, itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_double ( fd, 4, dtab ) ) return ( read_desc_error ( fct ) );
			bio_set_loop_pos ( itab[0], itab[1], dtab[0], dtab[1], dtab[2], dtab[3] );
			continue;
		}
		///////////////////////////////////////////////////////////////////////////////

		if ( strcmp ( fct, "bio_set_cyclooctane" ) == 0 )
		{
			bio_set_cyclooctane();
			continue;
		}

		if ( strcmp ( fct, "bio_set_cycloheptane" ) == 0 )
		{
			bio_set_cycloheptane();
			continue;
		}
#endif
#ifdef ENERGY
		/* bio_set_triade arguments in .p3d file:
		    - index of the 3 joints
		    - flag to indicate type of conformation:
		      - closed  ->  1
		- open    -> -1
		- unknown ->  0
		*/
		if ( strcmp ( fct, "bio_set_triade" ) == 0 )
		{
			if ( !read_desc_int ( fd, 4, itab ) ) return ( read_desc_error ( fct ) );
			bio_set_triade ( itab[0], itab[1], itab[2], itab[3] );
			continue;
		}

		/* bio_set_pairs_for_dist arguments in .p3d file:
		    - number of pairs
		    - indices of the jnts corresponding to each pair
		    - flag to indicate type of conformation:
		      - closed  ->  1
		- open    -> -1
		- unknown ->  0
		*/
		if ( strcmp ( fct, "bio_set_pairs_for_dist" ) == 0 )
		{
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 2*argnum[0], itab ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, itab2 ) ) return ( read_desc_error ( fct ) );
			bio_set_pairs_for_dist ( argnum[0], itab, itab2[0] );
			continue;
		}
#endif

		//##################### MACRO ######################

		if ( ( strcmp ( fct, "p3d_read_macro" ) == 0 ) || ( strcmp ( fct, "M3D_read_macro" ) == 0 ) )
		{
			if ( !read_desc_name ( fd, namemac ) ) return ( read_desc_error ( fct ) );
			if ( !read_desc_name ( fd, name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_line_double ( fd, &n, dtab ) ) return ( read_desc_error ( fct ) );
			if ( fileType )  //is a macro file
			{
				strcpy ( namecompl, nameobj );
				strcat ( namecompl, "." );
				strcat ( namecompl, name );
			}
			else
			{
				strcpy ( namecompl, name );
			}
			if ( n == 0 )
			{
				p3d_read_macro ( namemac, namecompl, 1 );
			}
			else
			{
				p3d_read_macro ( namemac, namecompl, dtab[0] );
			}
			continue;
		}

		//##################### MULTI-GRAPH ##################
#ifdef MULTIGRAPH
		if ( strcmp ( fct, "p3d_multi_graph" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !robotPt || !robotPt->cntrt_manager ) return ( read_desc_error ( fct ) );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) ); //number of joints
			if ( !read_desc_int ( fd, argnum[0], itab ) ) return ( read_desc_error ( fct ) );//joints number
			if ( !p3d_set_multi_graph_data ( robotPt, argnum[0], itab ) ) return ( read_desc_error ( fct ) );//joint already declared
			continue;
		}
		if ( strcmp ( fct, "p3d_involves_common_part" ) == 0 )
		{
			robotPt = ( pp3d_rob ) p3d_get_desc_curid ( P3D_ROBOT );
			if ( !read_desc_int ( fd, 1, argnum ) ) return ( read_desc_error ( fct ) ); //true or false (1/0)
			robotPt->mg->involvesCp = argnum[0];
			continue;
		}
#endif
		return ( read_desc_error ( fct ) );
	}
	while ( !fileType && p3d_inside_desc() )
	{
		p3d_end_desc();
	}
	return ( TRUE );
}

int read_macro_ground ( FILE *fd,char *nameobj, double scale )
{
	char  fct[100];
	char gc;
	double dtab[1000];
	int  n, itab[100];
	char  name[256];
	p3d_matrix4 pos;
	GroundCostObj = GHinitCgroundHeight ( 50 );
	while ( fscanf ( fd,"%s", fct ) != EOF )
	{
		if ( fct[0]=='#' )
		{
			/* comment in file: ignore the line ! */
			do
			{
				gc=getc ( fd );
			}
			while ( gc!='\n' && gc!=EOF );
			continue;
		}

		if ( strcmp ( fct,"info_pos_by_mat" ) ==0 )
		{
			if ( !read_desc_name ( fd,name ) )  return ( read_desc_error ( fct ) );
			if ( !read_desc_mat_scaled ( fd,pos,scale ) ) return ( read_desc_error ( fct ) );
			p3d_mat4Copy ( pos, Transfo );
			continue;
		}
#ifdef P3D_PLANNER
		if ( ( strcmp ( fct,"p3d_add_desc_vert" ) ==0 ) ||
		        ( strcmp ( fct,"M3D_add_desc_vert" ) ==0 ) )
		{
			if ( !read_desc_double ( fd,3,dtab ) ) return ( read_desc_error ( fct ) );
			AddSpaceScaledVertex ( GroundCostObj,dtab[0],dtab[1],dtab[2] );
			continue;
		}

		if ( ( strcmp ( fct,"p3d_add_desc_face" ) ==0 ) ||
		        ( strcmp ( fct,"M3D_add_desc_face" ) ==0 ) )
		{
			if ( !read_desc_line_int ( fd,&n,itab ) ) return ( read_desc_error ( fct ) );
			GHaddTriangle ( GroundCostObj,itab[0] -1 ,itab[1] -1 ,itab[2] - 1 );
			continue;
		}
#endif
	}
	GHinitializeGrid ( GroundCostObj );
	return TRUE;
}



// add by XB
int num_data_error ( char *msg )
{
	PrintError ( ( "MP: p3d_read_desc: wrong number of data function %s\n",msg ) );
	while ( p3d_inside_desc() ) p3d_end_desc();
	return ( FALSE );
}

