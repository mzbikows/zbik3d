/******************************************************************************
 *Zbik3D
 *Author:Mariusz ���bikowski
 *****************************************************************************/

//JOINTS ( POLYCRANK )
#include "polycrank.h"
//JOINTS ( IRp6 - POSTUMENT, TRACK )
#include "irp6.h"

//BASIC LIBRARY
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//PLIB PUI
#include <GL/glut.h>
#include <plib/pu.h>
#include <plib/puAux.h>

//SYNCHRONIZATION
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//IO
#include <iostream>

#include <sys/time.h>

using namespace std;

#define FONT_COLOUR   1,1,1,1 //Black

#define DISPLAY_FREQUENCY	60

pthread_mutex_t access_tablica; //dostep do 15-elementowej tablicy

// synchronizacja watkow pobierajacych dane z EDP
pthread_cond_t receive_cond;
pthread_mutex_t receive_mtx;
bool track_synch, postument_synch;

int track_enabled = 2;//0-off and synchro position,1-on,2-simulation
int postument_enabled = 2;//0-off and synchro position,1-on,2-simulation

//Lab012

char hostip_postument[16] = "127.0.0.1";
int hostport_postument = 50001;
char hostip_track[16] = "127.0.0.1";
int hostport_track = 50000;

int socket_postument;
int socket_track;

/* Variable controlling various rendering modes. */
static int stencilReflection = 1, stencilShadow = 1, offsetShadow = 1;
static int renderShadow = 0, renderReflection = 0;
static int directionalLight = 0;
static int forceExtension = 0;
static float lightAngle = M_PI/2, lightHeight = 15;
int main_window;
int lightMoving = 0, lightStartX=0, lightStartY=0;
int cameraMoving = 0;
int rotation = 0;

float zoom = 0.0f;
float roty = 0.0f;
float tx = 0;
float ty = 0;
int lastx=0;
int lasty=0;
float deltatx=0;
float front_zoom =0, front_roty=0, front_tx=0, front_ty=0 ;
float back_zoom =0, back_roty=0, back_tx=0, back_ty=0 ;
float left_zoom =0, left_roty=0, left_tx=0, left_ty=0 ;
float right_zoom =0, right_roty=0, right_tx=0, right_ty=0 ;
float top_zoom =0, top_roty=0, top_tx=0, top_ty=0 ;
//static puInput *input1, *input2 ;

static puMenuBar   *main_menu_bar ;
//static puButton    *hide_menu_button ;
static puDialogBox *dialog_box, *dialog_box2 ;
static puText      *dialog_box_message ;
static puOneShot   *dialog_box_ok_button, *dialog_box_ok_button2;
static puText      *timer_text ;

//MRROCPP Connection
static puInput *ip_track, *ip_postument, *port_track, *port_postument;

static puInput *path_save_trajectory, *state_save_trajectory;      //SAVER TRAJECTORY
static puInput *path_load_trajectory, *state_load_trajectory;      //LOADER TRAJECTORY

static puInput *path_save_configuration, *state_save_configuration;//SAVER CONFIGURATION
static puInput *path_load_configuration, *state_load_configuration;//LOADER CONFIGURATION

puSlider *player, *speed;//Player
puSlider *slider1, *slider2, *slider3;//Background color
puSlider *slider11, *slider22, *slider33;//Light Track
puSlider *slider111, *slider222, *slider333;//Light Postument

puSlider *slider_crank1,*slider_crank2,*slider_crank3,*slider_crank4,*slider_crank5,*slider_crank6,*slider_crank7,*slider_crank8;//Polycrank

puText *text1, *text2, *text3, *text4, *text5, *text6, *text7, *text8;
puText *text11, *text22, *textcrank;
puText *text9, *text99, *text999;
puText *text0, *text00, *text000, *text0000;
static puButton *button_track, *button_postument;//MRROCPP Connection
static puButton *button_save_trajectory, *button_load_trajectory;//Trajectory Saver & Player
static puButton *replay, *forward_play_trajectory, *backward_play_trajectory ;//Trajectory Player
static puButton *button_save_configuration, *button_load_configuration;//Configuration Saver & Player
static puFrame *diode_track, *diode_postument;//Connection with MRROCPP
enum
{
  MISSING, EXTENSION, ONE_DOT_ONE
};
int polygonOffsetVersion;

static GLfloat lightPosition[4]={1,15,1,0.0};
//static GLfloat lightColor[] = {0.8, 1.0, 0.8, 1.0}; /* green-tinted */

int   obj, obj2, obj3, obj4, obj5 = 0;//obj-wiews, obj2-shadow,obj3-light

//Light Track
GLfloat light1_ambient[4] =  {0.1f, 0.2f, 0.3f, 1.0f};
GLfloat light1_diffuse[4] =  {1.0f, 1.0f, 1.0f, 1.0f};
GLfloat light1_position[4] = { 0.0f, 8.0f, 10.0f, 0.0f};

//Light Postument
GLfloat light2_ambient[4] =  {0.1f, 0.2f, 0.3f, 1.0f};
GLfloat light2_diffuse[4] =  {1.0f, 1.0f, 1.0f, 1.0f};
GLfloat light2_position[4] = {0.0f, 8.0f, -10.0f, 0.0f};

static GLfloat floorPlane[4];
static GLfloat floorShadow[4][4];
GLint other,q8,q7,q6,q5,q4,q3,q2,d1;//Track, Postument
GLint crankbase,crankq1,crankq2,crankq3,crankq4,crankq5,crankq6,crankq7,crankq8,crankq9finger1,crankq9finger2;//Polycrank elements
GLint crankjointq2q1,crankjointq3q2,crankjointq4q3,crankjointq5q4,crankjointq6q5,crankjointq7q6;//Polycrank joints
bool polycrank = 1; //mode robots(polycrank or irp6)
//initilization angle joints table of Polycrank
float crank1 = 0;
float crank2 = 0;
float crank3 = 0;
float crank4 = 0;
float crank5 = 0;
float crank6 = 0;
float crank7 = 0;
float crank8 = 0.12;
//synchronization position
float joints[15]=
				{
					//**************TRACK*****************************
					12.5*((0.0+0.124f)/1.333f),		//Track_d1 Td1 TorJezdny       joints[0]
					((-0.087*180)/M_PI)+10,			//Track_q2 Tq2 KorpusObrot     joints[1]
					-90-((-1.542*180)/M_PI),       //Track_q3 Tq3 KolumnaPrzodTyl joints[2]
					-((0.024*180)/M_PI),           //Track_q4 Tq4 RamieGoraDol    joints[3]
					-((1.219*180)/M_PI),           //Track_q5 Tq5 LacznikGoraDol  joints[4]
					-((2.591*180)/M_PI)+270 ,      //Track_q6 Tq6 ChwytakObrot    joints[5]
					-((-2.664*180)/M_PI)+90 ,      //Track_q7 Tq7 KiscObrot       joints[6]
					0.2f - (0.2f*((0.074-0.054f)/(0.090f-0.054f))),//Track_q8 Tq8 Palec1, Palec2  joints[7]
					//***************POSTUMENT************************
					((-0.101*180)/M_PI),       //Postu_q1 Pq1 KorpusObrot     joints[8]
					-90-((-1.542*180)/M_PI),   //Postu_q2 Pq2 KolumnaPrzodTyl joints[9]
					-((0.049*180)/M_PI),       //Postu_q3 Pq3 RamieGoraDol    joints[10]
					-((1.198*180)/M_PI),       //Postu_q4 Pq4 LacznikGoraDol  joints[11]
					-((2.101*180)/M_PI)+270,   //Postu_q5 Pq5 ChwytakObrot    joints[12]
					-((-2.749*180)/M_PI)+90,   //Postu_q6 Pq6 KiscObrot       joints[13]
					0.2f - (0.2f*((0.074f-0.054f)/(0.090f-0.054f)))//Postu_q7 Pq7 Palec1, Palec2  joints[14]
				}; //joints angles

float r = 0.0, g = 0.0, b = 0.0; //initial color of background

int state = 0; //views:state0-front,state1-back,state2-left,state3-right,state4-top

bool mouse_stop = 0;

//TRAJECTORY FILE
//SAVER TRAJECTORY
bool save_trajectory = 0;
char filename_save_trajectory[255] = "trajectory.txt";

//CONFIGURATION FILE
//SAVER CONFIGURATION
bool save_configuration = 0;
char filename_save_configuration[255] = "config.txt";
//LOADER CONFIGURATION
bool load_configuration = 0;
char filename_load_configuration[255] = "config.txt";

//PLAYER TRAJECTORY
bool load_trajectory = 0;
bool forward_play = 0;
bool backward_play = 0;
bool replay_movie = 0;
char filename_load_trajectory[255] = "movie.txt";
char set_filename_load_trajectory[255] = "movie.txt";
int i=0;//index
FILE * fp;
long dlugosc;
long lines;//value of packet 15-elements
float **tab;//bufor

float player_place=0.0;//movie slider
int interval=20;//speed
bool hidden_player=0;

static void play_forward(void);
static void play_backward(void);
static void redraw(void);
static void timerCallback (int value);

void myGlutIdle( void )
{
	/*
	According to the GLUT specification, the current window is
	undefined during an idle callback.  So we need to explicitly change
	it if necessary
	*/

	if ( glutGetWindow() != main_window ) glutSetWindow(main_window);
	glutPostRedisplay();
}

void init()
{
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    crankbase = GenPolycrankBaseList(); //PolycrankPodstawa
	crankq1 = GenPolycrankQ1List(); //PolycrankQ1
	crankjointq2q1 = GenPolycrankJointQ2Q1List(); //PolycrankJointQ2Q1
    crankq2 = GenPolycrankQ2List(); //PolycrankQ2
	crankjointq3q2 = GenPolycrankJointQ3Q2List(); //PolycrankJointQ3Q2
	crankq3 = GenPolycrankQ3List(); //PolycrankQ3
	crankjointq4q3 = GenPolycrankJointQ4Q3List(); //PolycrankJointQ4Q3
	crankq4 = GenPolycrankQ4List(); //PolycrankQ4
	crankjointq5q4 = GenPolycrankJointQ5Q4List(); //PolycrankJointQ5Q4
    crankq5 = GenPolycrankQ5List(); //PolycrankQ5
	crankjointq6q5 = GenPolycrankJointQ6Q5List(); //PolycrankJointQ6Q5
    crankq6 = GenPolycrankQ6List(); //PolycrankQ6
	crankjointq7q6 = GenPolycrankJointQ7Q6List(); //PolycrankJointQ7Q6
	crankq7 = GenPolycrankQ7List(); //PolycrankQ7
	crankq8 = GenPolycrankQ8List(); //PolycrankQ8
	crankq9finger1 = GenPolycrankFinger1List();
	crankq9finger2 = GenPolycrankFinger2List();

	other = GenOtherList(); //Sto���
    d1 = GenTrackQ1List(); //TorJezdny
	q2 = GenTrackQ2List(); //KorpusObrot
	q3 = GenTrackQ3List(); //KolumnaPrzodTyl
    q4 = GenTrackQ4List(); //RamieGoraDol
	q5 = GenTrackQ5List(); //LacznikGoraDol
	q6 = GenTrackQ6List(); //ChwytakObrot
	q7 = GenTrackQ7List(); //KiscObrot
    q8 = GenTrackQ8List(); //Palec1, Palec2
}

int SaveTrajectoryFile ( void )
{
	FILE * fp = fopen( filename_save_trajectory, "a" );
	if(!fp) {
		perror("fopen()");
		return -1;
	}
	float value;

//TRACK
value = ((1.333*joints[0])/12.5)-0.124;//Trackd1 TorJezdny
fprintf( fp, "+%f ", value);
value = (joints[1]*M_PI)/180.0;        //TrackQ2 KorpusObrot
fprintf( fp, "+%f ", value);
value =  -M_PI*(joints[2]+90.0)/180.0; //TrackQ3 KolumnaPrzodTyl
fprintf( fp, "+%f ", value);
value = -(joints[3]*M_PI)/180.0;       //TrackQ4 RamieGoraDol
fprintf( fp, "+%f ", value);
value = -(joints[4]*M_PI)/180.0;       //TrackQ5 LacznikGoraDol
fprintf( fp, "+%f ", value);
value = -M_PI*(joints[5]-270.0)/180.0; //TrackQ6 ChwytakObrot
fprintf( fp, "+%f ", value);
value = -M_PI*(joints[6]-90.0)/180.0;  //TrackQ7 KiscObrot
fprintf( fp, "+%f ", value);
value = -(5*(joints[7]-0.2)*(0.090-0.054))+0.054;//TrackQ8 Palec1, Palec2
fprintf( fp, "+%f ", value);
//POSTUMENT
value = (joints[8]*M_PI)/180.0;        //PostumentQ2 KorpusObrot
fprintf( fp, "+%f ", value);
value =  -M_PI*(joints[9]+90.0)/180.0; //PostumentQ3 KolumnaPrzodTyl
fprintf( fp, "+%f ", value);
value = -(joints[10]*M_PI)/180.0;       //PostumentQ4 RamieGoraDol
fprintf( fp, "+%f ", value);
value = -(joints[11]*M_PI)/180.0;       //PostumentQ5 LacznikGoraDol
fprintf( fp, "+%f ", value);
value = -M_PI*(joints[12]-270.0)/180.0; //PostumentQ6 ChwytakObrot
fprintf( fp, "+%f ", value);
value = -M_PI*(joints[13]-90.0)/180.0;  //PostumentQ7 KiscObrot
fprintf( fp, "+%f ", value);
value = -(5*(joints[14]-0.2)*(0.090-0.054))+0.054;//PostumentQ8 Palec1, Palec2
fprintf( fp, "+%f\n", value);

	fclose(fp);

	return 0;
}

int SaveConfigurationFile ( void )
{
	FILE * fp = fopen( filename_save_configuration, "w" );
	if(!fp) {
		perror("fopen()");
		return -1;
	}

		switch(state)
		{
				case 0:
					{
						front_zoom = zoom;
						front_roty = roty;
						front_tx = tx;
						front_ty = ty;
						break;
					}
				case 1:
					{
						back_zoom = zoom;
						back_roty = roty;
						back_tx = tx;
						back_ty = ty;
						break;
					}
				case 2:
					{
						left_zoom = zoom;
						left_roty = roty;
						left_tx = tx;
						left_ty = ty;
						break;
					}
				case 3:
					{
						right_zoom = zoom;
						right_roty = roty;
						right_tx = tx;
						right_ty = ty;
						break;
					}
				case 4:
					{
						top_zoom = zoom;
						top_roty = roty;
						top_tx = tx;
						top_ty = ty;
					}
		}

	//Write to configuration file
	//1 line - Set Background Color (RGB)
	fprintf( fp, "%f %f %f\n", r,g,b);
	//2 line - Set Front View zoom roty tx ty
	fprintf( fp, "%f %f %f %f\n", front_zoom, front_roty, front_tx, front_ty);
	//3 line - Set Back View zoom roty tx ty
	fprintf( fp, "%f %f %f %f\n", back_zoom, back_roty, back_tx, back_ty);
	//4 line - Set TrackSide View zoom roty tx ty
	fprintf( fp, "%f %f %f %f\n", left_zoom, left_roty, left_tx, left_ty);
	//5 line - Set PostumentSide View zoom roty tx ty
	fprintf( fp, "%f %f %f %f\n", right_zoom, right_roty, right_tx, right_ty);
	//6 line - Set Top View zoom roty tx ty
	fprintf( fp, "%f %f %f %f\n", top_zoom, top_roty, top_tx, top_ty);
	//7 line - Set LightTrack Light1
	fprintf( fp, "%f %f %f\n",light1_diffuse[0] ,light1_diffuse[1] ,light1_diffuse[2]);
	//8 line - Set LightPostument Light2
	fprintf( fp, "%f %f %f\n",light2_diffuse[0] ,light2_diffuse[1] ,light2_diffuse[2]);
	//9 line - Set ip&port robot track
	fprintf( fp, "%s %d\n", hostip_track , hostport_track);
	//10 line - Set ip&port robot postument
	fprintf( fp, "%s %d\n", hostip_postument , hostport_postument);

	fclose(fp);

	return 0;
}

static void LoadConfigurationFile ( void )
{
	FILE * fp;

	if ((fp = fopen( filename_load_configuration, "r" ))==NULL)
	{
		state_load_configuration->setValue("File not exists");
		button_load_configuration->setValue(0);
		load_configuration = 0;
		return;
	}
	else
	{
		fscanf(fp, "%f %f %f", &r, &g, &b);

		fscanf( fp, "%f %f %f %f", &front_zoom, &front_roty, &front_tx, &front_ty);
		fscanf( fp, "%f %f %f %f", &back_zoom, &back_roty, &back_tx, &back_ty);

		fscanf( fp, "%f %f %f %f", &left_zoom, &left_roty, &left_tx, &left_ty);
		fscanf( fp, "%f %f %f %f", &right_zoom, &right_roty, &right_tx, &right_ty);

		fscanf( fp, "%f %f %f %f", &top_zoom, &top_roty, &top_tx, &top_ty);

		fscanf( fp, "%f %f %f", &light1_diffuse[0], &light1_diffuse[1], &light1_diffuse[2]);
		fscanf( fp, "%f %f %f", &light2_diffuse[0], &light2_diffuse[1], &light2_diffuse[2]);

		fscanf( fp, "%s %d", hostip_track, &hostport_track);
		fscanf( fp, "%s %d", hostip_postument, &hostport_postument);

		fclose(fp);

		switch(state)
		{
				case 0:
					{
						zoom = front_zoom;
						roty = front_roty;
						tx = front_tx;
						ty = front_ty;
						break;
					}
				case 1:
					{
						zoom = back_zoom;
						roty = back_roty;
						tx = back_tx;
						ty = back_ty;
						break;
					}
				case 2:
					{
						zoom = left_zoom;
						roty = left_roty;
						tx = left_tx;
						ty = left_ty;
						break;
					}
				case 3:
					{
						zoom = right_zoom;
						roty = right_roty;
						tx = right_tx;
						ty = right_ty;
						break;
					}
				case 4:
					{
						zoom = top_zoom;
						roty = top_roty;
						tx = top_tx;
						ty = top_ty;
					}
		}

		//Background color
		slider3->setValue(b);
		slider2->setValue(g);
		slider1->setValue(r);
		//LightTrack Light1
		slider33->setValue(light1_diffuse[2]);
		slider22->setValue(light1_diffuse[1]);
		slider11->setValue(light1_diffuse[0]);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse );
		//LightPostument Light2
		slider333->setValue(light2_diffuse[2]);
		slider222->setValue(light2_diffuse[1]);
		slider111->setValue(light2_diffuse[0]);
		glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse );
		//ip&ports
		ip_track->setValue( hostip_track );
		port_track->setValue( hostport_track );
		ip_postument->setValue( hostip_postument );
		port_postument->setValue( hostport_postument );
	}
	myGlutIdle();
}

int LoadTrajectoryFile ( void )
{
	int i,j;

	#ifdef _WIN32
	# define const_line 151
	#else
	# define const_line 151
	#endif

	if ((fp = fopen( filename_load_trajectory, "r" ))==NULL)
	{
		state_load_trajectory->setValue("File not exists");
		button_load_trajectory->setValue(0);
		load_trajectory = 0;
		return 0;
    }
	else
	{
		fseek (fp, 0, SEEK_END); //set pointer to the end of file
		dlugosc = ftell(fp);
		fseek (fp, 0, SEEK_SET);

		if (dlugosc % 151 == 0)
		{
			lines =(int) (dlugosc/151.0) ;
		}
		else if (dlugosc % 152 == 0)
		{
			lines =(int) (dlugosc/152.0) ;
		}

		tab = (float **)calloc(lines, sizeof(*tab));

		for (i = 0; i < lines; i++)
		{
			tab[i] = (float*) calloc(15, sizeof(float));
		}

		for(i=0;i<lines;i++)
		{
			for(j=0;j<15;j++)
			{
				fscanf(fp, "%f ",&tab[i][j]);
			}
		}

		fclose(fp);

		if (dlugosc % 151 != 0 && dlugosc % 152 != 0)
		{
			state_load_trajectory->setValue("Wrong file format");

			button_load_trajectory->setValue(0);
			forward_play_trajectory->setValue(0);
			backward_play_trajectory->setValue(0);
			load_trajectory = 0;
			forward_play=0;
			backward_play=0;

			return 0;
		}

	}
	return 0;
}

static void play_forward(void)
{
char str1[255] = "End of ";

if(load_trajectory == 1 && forward_play == 1)
{
	ulMilliSecondSleep(interval);
	if (i<lines)
	{
		//Trackd1 TorJezdny
		joints[0]=(float)12.5*((tab[i][0]+0.124f)/1.333f);
		//TrackQ2 KorpusObrot
		joints[1]=(float)(tab[i][1]*180.0f/M_PI);
		//TrackQ3 KolumnaPrzodTyl
		joints[2]=(float)-(tab[i][2]*180.0f/M_PI)-90;
		//TrackQ4 RamieGoraDol
		joints[3]=(float)-(tab[i][3]*180.0/M_PI);
		//TrackQ5  LacznikGoraDol
		joints[4]=(float)-(tab[i][4]*180.0/M_PI);
		//TrackQ6 ChwytakObrot
		joints[5]=(float)-(tab[i][5]*180.0/M_PI)+270;
		//TrackQ7 KiscObrot
		joints[6]=(float)-(tab[i][6]*180.0/M_PI)+90;
		//TrackQ8 Palec1, Palec2
		joints[7] =(float) 0.2f - (0.2f*((tab[i][7]-0.054f)/(0.090f-0.054f)));
		//PostumentQ2 KorpusObrot
		joints[8]=(float)(tab[i][8]*180.0/M_PI);
		//PostumentQ3 KolumnaPrzodTyl
		joints[9]=(float)-(tab[i][9]*180.0/M_PI)-90;
		//PostumentQ4 RamieGoraDol
		joints[10]=(float)-(tab[i][10]*180.0/M_PI);
		//PostumentQ5  LacznikGoraDol
		joints[11]=(float)-(tab[i][11]*180.0/M_PI);
		//PostumentQ6 ChwytakObrot
		joints[12]=(float)-(tab[i][12]*180.0/M_PI)+270;
		//PostumentQ7 KiscObrot
		joints[13]=(float)-(tab[i][13]*180.0/M_PI)+90;
		//PostumentQ8 Palec1, Palec2
		joints[14] =(float) 0.2f - 0.2f*((tab[i][14]-0.054f)/(0.090f-0.054f));
		i++;//increment
		player->setValue((float)(i/(float)lines));
	}
	if(i==lines)
	{
		if (replay_movie == 0)
		{
			strcat(str1,filename_load_trajectory);
			state_load_trajectory->setValue(str1);

			//in the end reset robots
			track_enabled = 0;
			postument_enabled = 0;
			forward_play=0;
			if (hidden_player==0)
			{
				button_load_trajectory->reveal();
				backward_play_trajectory->reveal () ;
			}

			backward_play_trajectory->setValue (0) ;
			forward_play_trajectory->setValue (0) ;
		}
		if (replay_movie == 1)
		{
			i=0;
		}
	}
}
if(load_trajectory == 0 && forward_play == 1)
{
	state_load_trajectory->setValue("At first load movie");
}
	myGlutIdle();
}

static void play_backward(void)
{
char str1[255] = "Begin of ";

if(load_trajectory==1 && backward_play == 1)
{
	ulMilliSecondSleep(interval);
	if (i>0)
	{
		i--;//decrement
		//Trackd1 TorJezdny
		joints[0]=(float)12.5*((tab[i][0]+0.124f)/1.333f);
		//TrackQ2 KorpusObrot
		joints[1]=(float)(tab[i][1]*180.0f/M_PI);
		//TrackQ3 KolumnaPrzodTyl
		joints[2]=(float)-(tab[i][2]*180.0f/M_PI)-90;
		//TrackQ4 RamieGoraDol
		joints[3]=(float)-(tab[i][3]*180.0/M_PI);
		//TrackQ5  LacznikGoraDol
		joints[4]=(float)-(tab[i][4]*180.0/M_PI);
		//TrackQ6 ChwytakObrot
		joints[5]=(float)-(tab[i][5]*180.0/M_PI)+270;
		//TrackQ7 KiscObrot
		joints[6]=(float)-(tab[i][6]*180.0/M_PI)+90;
		//TrackQ8 Palec1, Palec2
		joints[7] =(float) 0.2f - (0.2f*((tab[i][7]-0.054f)/(0.090f-0.054f)));
		//PostumentQ2 KorpusObrot
		joints[8]=(float)(tab[i][8]*180.0/M_PI);
		//PostumentQ3 KolumnaPrzodTyl
		joints[9]=(float)-(tab[i][9]*180.0/M_PI)-90;
		//PostumentQ4 RamieGoraDol
		joints[10]=(float)-(tab[i][10]*180.0/M_PI);
		//PostumentQ5  LacznikGoraDol
		joints[11]=(float)-(tab[i][11]*180.0/M_PI);
		//PostumentQ6 ChwytakObrot
		joints[12]=(float)-(tab[i][12]*180.0/M_PI)+270;
		//PostumentQ7 KiscObrot
		joints[13]=(float)-(tab[i][13]*180.0/M_PI)+90;
		//PostumentQ8 Palec1, Palec2
		joints[14] =(float) 0.2f - 0.2f*((tab[i][14]-0.054f)/(0.090f-0.054f));
		player->setValue((float)(i/(float)lines));
	}
	if(i==0)
	{
		if (replay_movie == 0)
		{
			strcat(str1,filename_load_trajectory);
			state_load_trajectory->setValue(str1);

			//in the end reset robots
			track_enabled = 0;
			postument_enabled = 0;
			backward_play=0;

			if (hidden_player==0)
			{
				button_load_trajectory->reveal();
				forward_play_trajectory->reveal () ;
			}

			backward_play_trajectory->setValue (0) ;
			forward_play_trajectory->setValue (0) ;
		}
		if (replay_movie == 1)
		{
			i=lines;
		}
	}
}
if(load_trajectory==0 && backward_play == 1)
{
	state_load_trajectory->setValue("At first load movie");
}
	myGlutIdle();
}

static void go_away_cb ( puObject * )
{
	// Delete the dialog box when its 'OK' button is pressed.

	puDeleteObject( dialog_box ) ;
	dialog_box = NULL ;
}

static void go_away_cb2 ( puObject * )
{
	// Delete the dialog box when its 'OK' button is pressed.

	puDeleteObject( dialog_box2 ) ;
	dialog_box2 = NULL ;
}

static void mk_dialog ( const char *fmt, ... )
{
	static char txt [ 512 ] ;

	va_list argptr ;
	va_start(argptr, fmt) ;
	vsprintf( txt, fmt, argptr ) ;
	va_end(argptr) ;

	dialog_box = new puDialogBox ( 150, 70 ) ;
	{
		new puFrame ( 0, 0, 400, 150 ) ;
		dialog_box_message   = new puText         ( 10, 100 ) ;
		dialog_box_message   -> setLabel          ( txt ) ;
		dialog_box_ok_button = new puOneShot      ( 180, 10, 240, 50 ) ;
		dialog_box_ok_button -> setLegend         ( "OK" ) ;
		dialog_box_ok_button -> makeReturnDefault ( TRUE ) ;
		dialog_box_ok_button -> setCallback       ( go_away_cb ) ;
	}

	dialog_box -> close  () ;
	dialog_box -> reveal () ;
}

static void mk_dialog2 ( const char *fmt, ... )
{
	static char txt [ 512 ] ;

	va_list argptr ;
	va_start(argptr, fmt) ;
	vsprintf( txt, fmt, argptr ) ;
	va_end(argptr) ;

	dialog_box2 = new puDialogBox ( 150, 70 ) ;
	{
		new puFrame ( 0, 0, 400, 150 ) ;
		dialog_box_message   = new puText         ( 10, 100 ) ;
		dialog_box_message   -> setLabel          ( txt ) ;
		dialog_box_ok_button2 = new puOneShot      ( 180, 10, 240, 50 ) ;
		dialog_box_ok_button2 -> setLegend         ( "OK" ) ;
		dialog_box_ok_button2 -> makeReturnDefault ( TRUE ) ;
		dialog_box_ok_button2 -> setCallback       ( go_away_cb2 ) ;
	}
	dialog_box2 -> close  () ;
	dialog_box2 -> reveal () ;
}

static GLfloat floorVertices[4][3] =
{
	{ -25.0, -0.3, 25.0 },
	{ 25.0, -0.3, 25.0 },
	{ 25.0, -0.3, -25.0 },
	{ -25.0, -0.3, -25.0 },
};

/* Draw a floor (possibly textured). */
static void drawFloor(void)
{
	glDisable(GL_LIGHTING);

	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3fv(floorVertices[0]);
	glTexCoord2f(0.0, 16.0);

	glVertex3fv(floorVertices[1]);
	glTexCoord2f(16.0, 16.0);
	glVertex3fv(floorVertices[2]);
	glTexCoord2f(16.0, 0.0);
	glVertex3fv(floorVertices[3]);
	glEnd();

	glEnable(GL_LIGHTING);
}

enum
{
	X, Y, Z, W
};
enum
{
	A, B, C, D
};

/* Create a matrix that will project the desired shadow. */
void shadowMatrix(GLfloat shadowMat[4][4], GLfloat groundplane[4], GLfloat lightpos[4])
{
	GLfloat dot;
	/* Find dot product between light position vector and ground plane normal. */
	dot = groundplane[X] * lightpos[X] + groundplane[Y] * lightpos[Y] + groundplane[Z] * lightpos[Z] + groundplane[W] * lightpos[W];

	shadowMat[0][0] = dot - lightpos[X] * groundplane[X];
	shadowMat[1][0] = 0.f - lightpos[X] * groundplane[Y];
	shadowMat[2][0] = 0.f - lightpos[X] * groundplane[Z];
	shadowMat[3][0] = 0.f - lightpos[X] * groundplane[W];

	shadowMat[X][1] = 0.f - lightpos[Y] * groundplane[X];
	shadowMat[1][1] = dot - lightpos[Y] * groundplane[Y];
	shadowMat[2][1] = 0.f - lightpos[Y] * groundplane[Z];
	shadowMat[3][1] = 0.f - lightpos[Y] * groundplane[W];

	shadowMat[X][2] = 0.f - lightpos[Z] * groundplane[X];
	shadowMat[1][2] = 0.f - lightpos[Z] * groundplane[Y];
	shadowMat[2][2] = dot - lightpos[Z] * groundplane[Z];
	shadowMat[3][2] = 0.f - lightpos[Z] * groundplane[W];

	shadowMat[X][3] = 0.f - lightpos[W] * groundplane[X];
	shadowMat[1][3] = 0.f - lightpos[W] * groundplane[Y];
	shadowMat[2][3] = 0.f - lightpos[W] * groundplane[Z];
	shadowMat[3][3] = dot - lightpos[W] * groundplane[W];
}

/* Find the plane equation given 3 points. */
void findPlane(GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3])
{
	GLfloat vec0[3], vec1[3];

	/* Need 2 vectors to find cross product. */
	vec0[X] = v1[X] - v0[X];
	vec0[Y] = v1[Y] - v0[Y];
	vec0[Z] = v1[Z] - v0[Z];

	vec1[X] = v2[X] - v0[X];
	vec1[Y] = v2[Y] - v0[Y];
	vec1[Z] = v2[Z] - v0[Z];

	/* find cross product to get A, B, and C of plane equation */
	plane[A] = vec0[Y] * vec1[Z] - vec0[Z] * vec1[Y];
	plane[B] = -(vec0[X] * vec1[Z] - vec0[Z] * vec1[X]);
	plane[C] = vec0[X] * vec1[Y] - vec0[Y] * vec1[X];
	plane[D] = -(plane[A] * v0[X] + plane[B] * v0[Y] + plane[C] * v0[Z]);
}

void DrawPolycrankBase()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankbase);
glPopMatrix();
}

void DrawPolycrankQ1()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq1);
glPopMatrix();
}

void DrawPolycrankJointQ2Q1()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq2q1);
glPopMatrix();
}

void DrawPolycrankQ2()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq2);
glPopMatrix();
}

void DrawPolycrankJointQ3Q2()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq3q2);
glPopMatrix();
}

void DrawPolycrankQ3()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq3);
glPopMatrix();
}

void DrawPolycrankJointQ4Q3()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq4q3);
glPopMatrix();
}

void DrawPolycrankQ4()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq4);
glPopMatrix();
}

void DrawPolycrankJointQ5Q4()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq5q4);
glPopMatrix();
}

void DrawPolycrankQ5()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq5);
glPopMatrix();
}

void DrawPolycrankJointQ6Q5()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq6q5);
glPopMatrix();
}

void DrawPolycrankQ6()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq6);
glPopMatrix();
}

void DrawPolycrankJointQ7Q6()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankjointq7q6);
glPopMatrix();
}

void DrawPolycrankQ7()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq7);
glPopMatrix();
}

void DrawPolycrankQ8()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
    glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq8);
glPopMatrix();
}

void DrawPolycrankFinger2()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq9finger2);
}

void DrawPolycrankFinger1()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 10.0f, 0.0f);
	glCallList(crankq9finger1);
}

void DrawOther()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(other);
glPopMatrix();
}
/*--------------------------------------d1------------------------------------*/
void DrawTrackd1()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(d1);
glPopMatrix();
}

/*--------------------------------------q2------------------------------------*/
void DrawPostumentQ2()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q2);
glPopMatrix();
}

void DrawTrackQ2()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q2);
glPopMatrix();
}

/*--------------------------------------q3------------------------------------*/
//PostumentKolumnaPrzodTyl
void DrawPostumentQ3()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q3);
glPopMatrix();
}

//TrackKolumnaPrzodTyl
void DrawTrackQ3()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q3);
glPopMatrix();
}
/*--------------------------------------q4------------------------------------*/
//PostumentRamieGoraDol q4
void DrawPostumentQ4()
{
glPushMatrix();
	glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q4);
glPopMatrix();
}

//TrackRamieGoraDol q4
void DrawTrackQ4()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q4);
glPopMatrix();
}
/*--------------------------------------q5------------------------------------*/
//PostumentLacznikGoraDol q5
void DrawPostumentQ5()
{
glPushMatrix();
	glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q5);
glPopMatrix();
}

//TrackLacznikGoraDol q5
void DrawTrackQ5()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q5);
glPopMatrix();
}
/*--------------------------------------q6------------------------------------*/
//PostumentChwytakObrot q6
void DrawPostumentQ6()
{
glPushMatrix();
	glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q6);
glPopMatrix();
}

//TrackChwytakObrot q6
void DrawTrackQ6()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q6);
glPopMatrix();
}
/*--------------------------------------q7------------------------------------*/
//PostumentKiscObrot q7
void DrawPostumentQ7()
{
glPushMatrix();
	glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q7);
glPopMatrix();
}

//TrackKiscObrot q7
void DrawTrackQ7()
{
glPushMatrix();
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q7);
glPopMatrix();
}
/*--------------------------------------q8--------------------------------------*/
//Fingers q8
void DrawTrackFinger2()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glCallList(q8);
}

void DrawTrackFinger1()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glTranslatef (0.0f, 0.0, 1.0);
	glCallList(q8);
}

void DrawPostumentFinger2()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q8);
}

void DrawPostumentFinger1()
{
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f, 8.0f, 0.0f);
	glTranslatef (0.0f, 0.0, -1.0);
	glScalef(1.0, 1.0, -1.0);
	glCallList(q8);
}

static void move_light(void)
{
glPushMatrix();
	glDisable(GL_LIGHTING);
		glColor3f(1.0, 1.0, 0.0);
		if (directionalLight)
		{
			/* Draw an arrowhead. */
			glDisable(GL_CULL_FACE);
				glTranslatef(lightPosition[0], lightPosition[1], lightPosition[2]);
				glRotatef(lightAngle * -180.0 / M_PI, 0, 1,0 );
				glRotatef(atan(lightHeight/12) * 180.0 / M_PI, 0, 0, 1);

				glBegin(GL_TRIANGLE_FAN);
					glVertex3f(0, 0, 0);
					glVertex3f(2, 1, 1);
					glVertex3f(2, -1, 1);
					glVertex3f(2, -1, -1);
					glVertex3f(2, 1, -1);
					glVertex3f(2, 1, 1);
				glEnd();

				/* Draw a white line from light direction. */
				glColor3f(1.0, 1.0, 1.0);

				glBegin(GL_LINES);
					glVertex3f(0.1, 0, 0);
					glVertex3f(5, 0, 0);
				glEnd();
			glEnable(GL_CULL_FACE);
		}
		else
		{
			/* Draw a yellow ball at the light source. */
			glTranslatef(lightPosition[0], lightPosition[1], lightPosition[2]);
			glutSolidSphere(1.0, 5, 5);
		}
	glEnable(GL_LIGHTING);
glPopMatrix();
}

static void shadow_render(void)
{
if (renderShadow)
{
	/* Render the projected shadow. */
	if (stencilShadow)
	{
		/* Now, only render where stencil is set above 5 (ie, 6 wherethe top floor is).  Update stencil with 2 where the shadow
		gets drawn so we don't redraw (and accidently reblend) the shadow). */
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_NOTEQUAL, 0x0, 0x2);
		glStencilMask(0x2);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	}
	/* To eliminate depth buffer artifacts, we use polygon offset to raise the depth of the projected shadow slightly so
		that it does not depth buffer alias with the floor. */
	if (offsetShadow)
	{
		switch (polygonOffsetVersion)
		{
			case EXTENSION:
			#ifdef GL_EXT_polygon_offset
				glEnable(GL_POLYGON_OFFSET_EXT);
				break;
			#endif
			#ifdef GL_VERSION_1_1
			case ONE_DOT_ONE:
				glEnable(GL_POLYGON_OFFSET_FILL);
				break;
			#endif
		}
	}
	/* Render 50% black shadow color on top of whatever the  floor appareance is. */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_LIGHTING);  /* Force the 50% black. */
	glColor4f(0.0, 0.0, 0.0, 0.5);
	//glPushMatrix();
	/* Project the shadow. */
	glMultMatrixf((GLfloat *) floorShadow);

glPushMatrix();
	//q2 KorpusObrot
	glTranslatef(-11.265500, 6.658370,4.480520 );
	glRotatef(joints[8], 0.0, 1.0, 0.0);
	glTranslatef(11.265500, -6.658370,-4.480520 );
	DrawPostumentQ2();
	//q3 KolumnaPrzodTyl
	glTranslatef (-11.27050, 9.071130 , 4.480520);
	glRotatef(joints[9], 1.0, 0.0, 0.0);
	glTranslatef (11.27050, -9.071130 , -4.480520);
	DrawPostumentQ3();
	//q4 RamieGoraDol
	glTranslatef (-11.250500, 13.577670, 4.480520);
	glRotatef(joints[10], 1.0, 0.0, 0.0);
	glTranslatef (11.250500, -13.577670, -4.480520);
	DrawPostumentQ4();
	//q5 LacznikGoraDol
	glTranslatef (-11.270500, 13.577670, -2.218800);
	glRotatef(joints[11], 1.0, 0.0, 0.0);
	glTranslatef (11.270500, -13.577670, 2.218800);
	DrawPostumentQ5();
	//q6 ChwytakObrot
	glTranslatef (-11.269700f, 13.56767f , -3.028870f);
	glRotatef(joints[12], 0.0, 0.0, 1.0);
	glTranslatef (11.269700f, -13.56767f , 3.028870f);
	DrawPostumentQ6();
	//q7 KiscObrot
	glTranslatef (-11.2595f, 12.907740f, -4.368580f);
	glRotatef(joints[13], 0.0, 1.0, 0.0);
	glTranslatef (11.2595f, -12.907740f, 4.368580f);
	DrawPostumentQ7();
	//Postument q8 finger1
	glPushMatrix();
		glTranslatef (joints[14], 0.0f, 0.0f);
		DrawPostumentFinger1();
	glPopMatrix();
	//Postument q8 finger2
	glPushMatrix();
		glTranslatef (-joints[14], 0.0, 0.0);
		DrawPostumentFinger2();
	glPopMatrix();
glPopMatrix();

//Shadow Robot Track
glPushMatrix();
		//d1 TorJezdny
		glTranslatef(-joints[0], 0.0, 0.0);
		DrawTrackd1();
		//q2 KorpusObrot
		glTranslatef(11.265500, 6.658370,4.480520 );
		glRotatef(joints[1], 0.0, 1.0, 0.0);
		glTranslatef(-11.265500, -6.658370,-4.480520 );
		DrawTrackQ2();
		//q3 KolumnaPrzodTyl
		glTranslatef (11.27050, 9.071130 , 4.480520);
		glRotatef(joints[2], 1.0, 0.0, 0.0);
		glTranslatef (-11.27050, -9.071130 , -4.480520);
		//q3 KolumnaPrzodTyl
		glTranslatef (11.27050, 9.071130 , 4.480520);
		glRotatef(joints[2], 1.0, 0.0, 0.0);
		glTranslatef (-11.27050, -9.071130 , -4.480520);
		DrawTrackQ3();
		//q4 RamieGoraDol
		glTranslatef (11.250500, 13.577670, 4.480520);
		glRotatef(joints[3], 1.0, 0.0, 0.0);
		glTranslatef (-11.250500, -13.577670, -4.480520);
		DrawTrackQ4();
		//q5 LacznikGoraDol
		glTranslatef (11.270500, 13.577670, -2.218800);
		glRotatef(joints[4], 1.0, 0.0, 0.0);
		glTranslatef (-11.270500,- 13.577670, 2.218800);
		DrawTrackQ5();
		//q6 ChwytakObrot
		glTranslatef (11.269700f, 13.56767f , -3.028870f);
		glRotatef(joints[5], 0.0, 0.0, 1.0);
		glTranslatef (-11.269700f, -13.56767f , 3.028870f);
		DrawTrackQ6();
		//q7 KiscObrot
		glTranslatef (11.2595f, 12.907740f, -4.368580f);
		glRotatef(joints[6], 0.0, 1.0, 0.0);
		glTranslatef (-11.2595f, -12.907740f, 4.368580f);
		DrawTrackQ7();
		//Track q8 finger2
		glPushMatrix();
			glTranslatef (joints[7], 0.0f, 0.0f);
			DrawTrackFinger2();
		glPopMatrix();
		//Track q8 finger1
		glPushMatrix();
			glTranslatef (-joints[7], 0.0, 0.0);
			DrawTrackFinger1();
		glPopMatrix();
glPopMatrix();

glPushMatrix();
	DrawOther();
glPopMatrix();

	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
	if (offsetShadow)
	{
	switch (polygonOffsetVersion)
	{
#ifdef GL_EXT_polygon_offset
	case EXTENSION:
		glDisable(GL_POLYGON_OFFSET_EXT);
		break;
#endif
#ifdef GL_VERSION_1_1
	case ONE_DOT_ONE:
		glDisable(GL_POLYGON_OFFSET_FILL);
	break;
#endif
	case MISSING:
		/* Oh well. */
		break;
	}
	}
	if (stencilShadow)
	{
		glDisable(GL_STENCIL_TEST);
	}
}
}

static void draw_robots(void)
{
//Robot Track
glPushMatrix();
	//d1 TorJezdny
	glTranslatef(-joints[0], 0.0, 0.0);
	DrawTrackd1();

	//q2 KorpusObrot
	glTranslatef(11.265500, 6.658370,4.480520 );
	glRotatef(joints[1], 0.0, 1.0, 0.0);
	glTranslatef(-11.265500, -6.658370,-4.480520 );
	DrawTrackQ2();

	//q3 KolumnaPrzodTyl
	glTranslatef (11.27050, 9.071130 , 4.480520);
	glRotatef(joints[2], 1.0, 0.0, 0.0);
	glTranslatef (-11.27050, -9.071130 , -4.480520);
	DrawTrackQ3();

	//q4 RamieGoraDol
	glTranslatef (11.250500, 13.577670, 4.480520);
	glRotatef(joints[3], 1.0, 0.0, 0.0);
	glTranslatef (-11.250500, -13.577670, -4.480520);
	DrawTrackQ4();

	//q5 LacznikGoraDol
	glTranslatef (11.270500, 13.577670, -2.218800);
	glRotatef(joints[4], 1.0, 0.0, 0.0);
	glTranslatef (-11.270500,- 13.577670, 2.218800);
	DrawTrackQ5();

	//q6 ChwytakObrot
	glTranslatef (11.269700f, 13.56767f , -3.028870f);
	glRotatef(joints[5], 0.0, 0.0, 1.0);
	glTranslatef (-11.269700f, -13.56767f , 3.028870f);
	DrawTrackQ6();

	//q7 KiscObrot
	glTranslatef (11.2595f, 12.907740f, -4.368580f);
	glRotatef(joints[6], 0.0, 1.0, 0.0);
	glTranslatef (-11.2595f, -12.907740f, 4.368580f);
	DrawTrackQ7();

	//Track q8 finger2
	glPushMatrix();
		glTranslatef (joints[7], 0.0f, 0.0f);
		DrawTrackFinger2();
	glPopMatrix();
	//Track q8 finger1
	glPushMatrix();
		glTranslatef (-joints[7], 0.0, 0.0);
		DrawTrackFinger1();
		glPopMatrix();
	glPopMatrix();

//Robot Postument
glPushMatrix();

	//q2 KorpusObrot
	glTranslatef(-11.265500, 6.658370,4.480520 );
	glRotatef(joints[8], 0.0, 1.0, 0.0);
	glTranslatef(11.265500, -6.658370,-4.480520 );
	DrawPostumentQ2();

	//q3 KolumnaPrzodTyl
	glTranslatef (-11.27050, 9.071130 , 4.480520);
	glRotatef(joints[9], 1.0, 0.0, 0.0);
	glTranslatef (11.27050, -9.071130 , -4.480520);
	DrawPostumentQ3();

	//q4 RamieGoraDol
	glTranslatef (-11.250500, 13.577670, 4.480520);
	glRotatef(joints[10], 1.0, 0.0, 0.0);
	glTranslatef (11.250500, -13.577670, -4.480520);
	DrawPostumentQ4();

	//q5 LacznikGoraDol
	glTranslatef (-11.270500, 13.577670, -2.218800);
	glRotatef(joints[11], 1.0, 0.0, 0.0);
	glTranslatef (11.270500, -13.577670, 2.218800);
	DrawPostumentQ5();

	//q6 ChwytakObrot
	glTranslatef (-11.269700f, 13.56767f , -3.028870f);
	glRotatef(joints[12], 0.0, 0.0, 1.0);
	glTranslatef (11.269700f, -13.56767f , 3.028870f);
	DrawPostumentQ6();

	//q7 KiscObrot
	glTranslatef (-11.2595f, 12.907740f, -4.368580f);
	glRotatef(joints[13], 0.0, 1.0, 0.0);
	glTranslatef (11.2595f, -12.907740f, 4.368580f);
	DrawPostumentQ7();

	//Postument q8 finger1
	glPushMatrix();
		glTranslatef (joints[14], 0.0f, 0.0f);
		DrawPostumentFinger1();
	glPopMatrix();
	//Postument q8 finger2
	glPushMatrix();
		glTranslatef (-joints[14], 0.0, 0.0);
		DrawPostumentFinger2();
	glPopMatrix();
glPopMatrix();

//Other
glPushMatrix();
	DrawOther();
glPopMatrix();
}

static void draw_polycrank(void)
{
//Polycrank
glPushMatrix();
		DrawPolycrankBase();

		glTranslatef(0.0f, 1.0f, -0.7f);
		glRotatef(180, 0.0, 1.0, 0.0);
		glTranslatef(0.0f, -1.0f, 0.7f);
		DrawPolycrankQ1();

		DrawPolycrankJointQ2Q1();

		glTranslatef(0.0f, 1.0f, -0.7f);
		glRotatef(crank1, 0.0, 1.0, 0.0);
		glTranslatef(0.0f, -1.0f, 0.7f);
		DrawPolycrankQ2();

		DrawPolycrankJointQ3Q2();

		glTranslatef(0.0f, 11.0f, 2.15f);
		glRotatef(crank2, 0.0, 1.0, 0.0);
		glTranslatef(0.0f, -11.0f, -2.15f);
		DrawPolycrankQ3();

		DrawPolycrankJointQ4Q3();

		glTranslatef(0.0f, 11.0f, 0.42f);
		glRotatef(crank3, 0.0, 1.0, 0.0);
		glTranslatef(0.0f, -11.0f, -0.42f);
		DrawPolycrankQ4();

		DrawPolycrankJointQ5Q4();

		glTranslatef(0.0f, 16.21f, 0.1f);
		glRotatef(crank4, 0.0, 0.0, 1.0);
		glTranslatef(0.0f, -16.21f, -0.1f);
		DrawPolycrankQ5();

		DrawPolycrankJointQ6Q5();

		glTranslatef(0.0f, 17.75f, 1.22f);
		glRotatef(crank5, 0.0, 0.0, 1.0);
		glTranslatef(0.0f, -17.75f, -1.22f);
		DrawPolycrankQ6();

		DrawPolycrankJointQ7Q6();

		glTranslatef(0.0, 19.28, 2.21);
		glRotatef(crank6, 0.0, 0.0, 1.0);
		glTranslatef(0.0, -19.28, -2.21);
		DrawPolycrankQ7();

		glTranslatef(0.0, 19.28, 2.83);
		glRotatef(crank7, 1.0, 0.0, 0.0);
		glTranslatef(0.0, -19.28, -2.83);
		DrawPolycrankQ8();

		//Polycrank q9 finger1
		glPushMatrix();
			glTranslatef (0.0f,-crank8+0.12, 0.0f);
			DrawPolycrankFinger1();
		glPopMatrix();
		//Polycrank q9 finger2
		glPushMatrix();
			glTranslatef (0.0f,crank8-0.12, 0.0);
			DrawPolycrankFinger2();
		glPopMatrix();

glPopMatrix();
}

static void views(void)
{
//Zapisywanie ustawienie w danym widoku po przejsciu do nastepnego widoku
if(state == 0 && obj != 0)
{
	front_zoom = zoom;
	front_roty = roty;
	front_tx = tx;
	front_ty = ty;
}
if(state == 1 && obj != 1)
{
	back_zoom = zoom;
	back_roty = roty;
	back_tx = tx;
	back_ty = ty;
}
if(state == 2 && obj != 2)
{
	left_zoom = zoom;
	left_roty = roty;
	left_tx = tx;
	left_ty = ty;
}
if(state == 3 && obj != 3)
{
	right_zoom = zoom;
	right_roty = roty;
	right_tx = tx;
	right_ty = ty;
}
if(state == 4 && obj != 4)
{
	top_zoom = zoom;
	top_roty = roty;
	top_tx = tx;
	top_ty = ty;
}

//Przy powrocie do w widoku
if(state != 0 && obj == 0)
{
	zoom = front_zoom;
	roty = front_roty;
	tx = front_tx;
	ty = front_ty;
}
if(state != 1 && obj == 1)
{
	zoom = back_zoom;
	roty = back_roty;
	tx = back_tx;
	ty = back_ty;
}
if(state != 2 && obj == 2)
{
	zoom = left_zoom;
	roty = left_roty;
	tx = left_tx;
	ty = left_ty;
}
if(state != 3 && obj == 3)
{
	zoom = right_zoom;
	roty = right_roty;
	tx = right_tx;
	ty = right_ty;
}
if(state != 4 && obj == 4)
{
	zoom = top_zoom;
	roty = top_roty;
	tx = top_tx;
	ty = top_ty;
}
if (obj != 4)
{
	glMatrixMode(GL_MODELVIEW); //normal
	glLoadIdentity();
	gluLookAt
		(
			0.0, 8.0, -80.0,
			0.0, 8.0, 0.0,
			0.0, 1.0, 0.0
		);
}
if (obj == 4)
{
	glMatrixMode(GL_MODELVIEW);//top
	glLoadIdentity();
	gluLookAt
		(
			0.0, 60.0, 0.0,
			0.0, 8.0, 0.0,
			0.0, 0.0, -1.0
		);
}
	if (obj == 0) state = 0; //front
	if (obj == 1) state = 1; //back
	if (obj == 2) state = 2; //left
	if (obj == 3) state = 3; //right
	if (obj == 4) state = 4; //top
}

static void modes(void)
{
	if ( obj3 == 0 )
	{
		directionalLight = 1;
	}
	else if(obj3 == 1 )
	{
		directionalLight = 0;
	}
	if ( obj2 == 0 ) //OFFSET
	{
		stencilShadow = 1;
		renderShadow = 1 ;
		offsetShadow = 1;
	}
	else if ( obj2 == 1 ) //STENCILING
	{
		renderShadow=1;
		stencilShadow=0;
	}
}

static void redraw(void)
{
	views();
	modes();
	if ((stencilReflection && renderReflection) || (stencilShadow && renderShadow) )
	{
		glStencilMask(0xffffffff);
		glClearStencil(0x4);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	}
	else
	{
		/* Avoid clearing stencil when not using it. */
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	/* Reposition the light source. */
	lightPosition[0] =15*cos(lightAngle);
	lightPosition[1] = lightHeight;
	lightPosition[2] =15*sin(lightAngle);

	if (directionalLight)
	{
		lightPosition[3] = 0.0;
	}
	else
	{
		lightPosition[3] = 1.0;
	}
	shadowMatrix(floorShadow, floorPlane, lightPosition);

	glPushMatrix();
	glTranslatef(0,0,-zoom);
	glTranslatef(tx, ty, 0);
	if(state==0)
	{
		glRotatef(roty, 0.0, 1.0, 0.0);
	}
	else if(state==1)
	{
		glRotatef(180+roty, 0.0, 1.0, 0.0);
	}
	else if (state==2)
	{
		glRotatef(90+roty, 0.0, 1.0, 0.0);
	}
	else if (state==3)
	{
		glRotatef(-90+roty, 0.0, 1.0, 0.0);
	}
	if (renderReflection)
	{
		if (stencilReflection)
		{
		/* We can eliminate the visual "artifact" of seeing the "flipped"model underneath the floor by using stencil.  The idea is draw the floor without color or
		depth update but so that a stencil value of one is where the floor will be.  Later when rendering the model reflection, we will only update pixels
		with a stencil value of 1 to make sure the reflection only lives on the floor, not below the floor. */

		/* Don't update color or depth. */
		glDisable(GL_DEPTH_TEST);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

		/* Draw 1 into the stencil buffer. */
		glEnable(GL_STENCIL_TEST);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glStencilFunc(GL_ALWAYS, 1, 0x1);
		glStencilMask(0x1);

		/* Now render floor; floor pixels just get their stencil set to 1. */
		drawFloor();

		/* Re-enable update of color and depth. */
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glEnable(GL_DEPTH_TEST);

		/* Now, only render where stencil is set to 1. */
		glStencilFunc(GL_EQUAL, 1, 0x1);  /* draw if ==1 */
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	}

		glPushMatrix();

		/* The critical reflection step: Reflect 3D model through the floor
		(the Y=0 plane) to make a relection. */
		glScalef(1.0, -1.0, 1.0);

		/* To avoid our normals getting reversed and hence botched lighting
		on the reflection, turn on normalize.  */
		glEnable(GL_NORMALIZE);
		glCullFace(GL_FRONT);

		/* Disable noramlize again and re-enable back face culling. */
		glDisable(GL_NORMALIZE);
		glCullFace(GL_BACK);

		glPopMatrix();

		if (stencilReflection)
		{
			glDisable(GL_STENCIL_TEST);
		}
	}
	/* Back face culling will get used to only draw either the top or the bottom floor.  This let's us get a floor with two distinct appearances.
	The top floor surface is reflective and kind of red. The bottom floor surface is not reflective and blue. */
	/* Draw "bottom" of floor in blue. */
	glFrontFace(GL_CW);  /* Switch face orientation. */
	//glColor4f(0.1, 0.1, 0.7, 1.0);//color of floor
	glColor4f(0.0, 0.0, 0.0, 1.0);//color of floor
	drawFloor();
	glFrontFace(GL_CCW);
	if (renderShadow && stencilShadow)
	{
		/* Draw the floor with stencil value 2.  This helps us only draw the shadow once per floor pixel (and only on the floor pixels). */
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 0x2, 0x2);
		glStencilMask(0x2);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	}
	/* Draw "top" of floor.  Use blending to blend in reflection. */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(1.0, 1.0, 1.0, 0.3);
	drawFloor();
	glDisable(GL_BLEND);

	if (renderShadow && stencilShadow)
	{
		glDisable(GL_STENCIL_TEST);
	}
	if (polycrank==0)
	{
		draw_robots();
		move_light();
		shadow_render();
	}
	if (polycrank==1)
	{
		draw_polycrank();
	}

	glPopMatrix();

	/* set the background colour using global variables */
	glClearColor(r,g,b,0);

	time_t t = time ( NULL ) ;
	timer_text -> setLabel ( ctime ( & t ) ) ;

	puDisplay();
	glutSwapBuffers();
}

/* called when a mouse button is pressed or released */
void MouseButton(int button,int state,int x,int y)
{
	lastx=x;
	lasty=y;
	myGlutIdle();
	if (button == GLUT_LEFT_BUTTON) //ROTATION
	{
		if (state == GLUT_DOWN && mouse_stop==0)
		{
			rotation = 1;
			lightStartX = x;
			lightStartY = y;
		}
		if (state == GLUT_DOWN && mouse_stop==1)
		{
			rotation = 0;
		}
		if (state == GLUT_UP && mouse_stop==1)
		{
			mouse_stop = 0;
		}
		if (state == GLUT_UP && mouse_stop==0)
		{
			rotation = 0;
		}
	}
	if (button == GLUT_RIGHT_BUTTON) //TRANSLATION
	{
		if (state == GLUT_DOWN)
		{
			cameraMoving = 1;
			lightStartX = x;
			lightStartY = y;
		}
		if (state == GLUT_UP)
		{
			cameraMoving = 0;
		}
	}
	//LEFT & RIGHT BUTTON - ZOOM
	if (button == GLUT_MIDDLE_BUTTON) //LIGHT MOVING
	{
		if (state == GLUT_DOWN)
		{
			lightMoving = 1;
			lightStartX = x;
			lightStartY = y;
		}
		if (state == GLUT_UP)
		{
			lightMoving = 0;
		}
	}
	/* see if the mouse clicked on any of the user interface controls */
	puMouse(button,state,x,y);
	glutPostRedisplay();
}

/* called when the mouse moves */
void MouseMove(int x,int y)
{
	int diffx=x-lastx;
	int diffy=y-lasty;
	lastx=x;
	lasty=y;
	myGlutIdle();
	if (lightMoving) //LIGHT MOVING
	{
		lightAngle += (x-lightStartX)/40.0;
		lightHeight += (lightStartY - y)/20.0;
		lightStartX = x;
		lightStartY = y;
	}
	if (mouse_stop == 0)
	{
		if ((rotation == 1) &&  (cameraMoving == 0)) //ROTATION
		{
			roty += (float) 0.5f * diffx;
		}
	}
	if ((cameraMoving == 1) && (rotation == 0)) //CAMERA MOVING
	{
		tx -= (float) 0.05f * diffx;
		ty -= (float) 0.05f * diffy;
	}
	if ((rotation == 1) && (cameraMoving == 1))  //ZOOM
	{
		zoom += (float) 0.05f * diffx;
	}

	/* Pass information about the position of the mouse to pui */
	puMouse(x,y);
	glutPostRedisplay();
}

static void keyfn ( unsigned char key, int x, int y)
{
	switch(key)
	{
		case '\033': exit(EXIT_SUCCESS); break;
		default: break;
	}
	puKeyboard ( key, PU_DOWN ) ;

	glutPostRedisplay () ;
}

void pressKey(int key, int x, int y)
{
	switch (key)
	{
		case GLUT_KEY_F1://rotation in left
		{
			roty+=3.0;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_F2://rotation in right
		{
			roty-=3.0;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_F3://near
		{
			zoom+=1.0;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_F4://far
		{
			zoom-=1.0;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_LEFT://translation in left
		{
			tx -= 0.5f;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_RIGHT://translation in right
		{
			tx += 0.5f;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_UP://translation to up
		{
			ty -= 0.5f;
			myGlutIdle();
			break;
		}
		case GLUT_KEY_DOWN://translation to down
		{
			ty += 0.5f;
			myGlutIdle();
			break;
		}
	}
}

/* When not visible, stop animating.  Restart when visible again. */
//static void visible(int vis)
//{
//	// TODO: this does not work for multiple screens, so seems pretty useless
//}

static int supportsOneDotOne(void)
{
	const char *version;
	int major, minor;

	version = (char *) glGetString(GL_VERSION);
	if (sscanf(version, "%d.%d", &major, &minor) == 2)
		return major >= 1 && minor >= 1;

	return 0;            /* OpenGL version string malformed! */
}

void changeSize(int w, int h)
{
	float ratio;

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if(h == 0)
		h = 1;

	ratio = 1.0f * w / h;
	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the clipping volume
	gluPerspective(30.0,ratio,1,160.0);
	glMatrixMode(GL_MODELVIEW);
}

/* The display function - called when redraw is needed */
void Display()
{
	glClear(GL_COLOR_BUFFER_BIT);

	time_t t = time ( NULL ) ;
	timer_text -> setLabel ( ctime ( & t ) ) ;

	/* display the user interface */
	puDisplay();

	glutSwapBuffers();
}

//Configuration
static void configuration_hide_cb ( puObject * )
{
	path_save_configuration->hide () ;
	state_save_configuration->hide () ;
	button_save_configuration->hide () ;
	text00->hide () ;

	path_load_configuration->hide () ;
	state_load_configuration->hide () ;
	button_load_configuration->hide () ;
	text000->hide() ;
}

static void configuration_show_cb ( puObject * )
{
	path_load_configuration->reveal () ;
	state_load_configuration->reveal () ;
	button_load_configuration->reveal () ;
	text000->reveal () ;

	path_save_configuration->reveal () ;
	state_save_configuration->reveal () ;
	button_save_configuration->reveal () ;
	text00->reveal () ;
	myGlutIdle();
}

//PLayer
static void trajectory_player_hide_cb ( puObject * )
{
	hidden_player = 1;
	player->hide () ;
	speed->hide () ;
	path_load_trajectory->hide () ;
	state_load_trajectory->hide () ;
	backward_play_trajectory->hide () ;
	forward_play_trajectory->hide () ;
	button_load_trajectory->hide () ;
	replay->hide () ;
	text0000->hide () ;
}

static void trajectory_player_show_cb ( puObject * )
{
	hidden_player = 0;
	player->reveal () ;
	speed->reveal () ;
	path_load_trajectory->reveal () ;
	state_load_trajectory->reveal () ;
	if(forward_play==1)
	{
		forward_play_trajectory->reveal () ;
	}
	else if(backward_play==1)
	{
		backward_play_trajectory->reveal () ;
	}
	else
	{
		forward_play_trajectory->reveal () ;
		backward_play_trajectory->reveal () ;
		button_load_trajectory->reveal () ;
	}
	replay->reveal () ;
	text0000->reveal () ;
	//hide mrrocpp connection
	postument_enabled = 0;
	track_enabled = 0;
	button_track -> setValue(0);
	button_postument -> setValue(0);
	diode_postument->setColorScheme(1,0,0,1);
	diode_track->setColorScheme(1,0,0,1);

	diode_track -> hide () ;
	diode_postument -> hide () ;
	button_track -> hide () ;
	button_postument -> hide () ;
	button_save_trajectory -> hide () ;
	path_save_trajectory -> hide () ;
	state_save_trajectory -> hide () ;
	text0 -> hide ();

	myGlutIdle();
}

static void show_connection_cb ( puObject *cb )
{
	//show mrrocpp connection
	diode_track -> reveal () ;
	diode_postument -> reveal () ;
	button_track -> reveal () ;
	button_postument -> reveal () ;
	button_save_trajectory -> reveal () ;
	path_save_trajectory -> reveal () ;
	state_save_trajectory -> reveal () ;
	text0 -> reveal ();

	//hide trajectory player
	forward_play = 0;
	backward_play = 0;
	postument_enabled = 0;
	track_enabled = 0;
	backward_play_trajectory->setValue (0) ;
	forward_play_trajectory->setValue (0) ;

	player->hide () ;
	speed->hide () ;
	path_load_trajectory->hide () ;
	state_load_trajectory->hide () ;
	backward_play_trajectory->hide () ;
	forward_play_trajectory->hide () ;
	button_load_trajectory->hide () ;
	replay->hide () ;
	text0000->hide () ;

	myGlutIdle();
}

static void hide_connection_cb ( puObject *cb )
{
	diode_track -> hide () ;
	diode_postument -> hide () ;
	button_track -> hide () ;
	button_postument -> hide () ;
	button_save_trajectory -> hide () ;
	path_save_trajectory -> hide ();
	state_save_trajectory -> hide ();
	text0 -> hide ();
}

static void show_ipports_cb ( puObject *cb )
{
	ip_track -> reveal () ;
	port_track -> reveal () ;
	ip_postument -> reveal () ;
	port_postument -> reveal () ;
	text9 -> reveal () ;
	text99 -> reveal () ;
	text999 -> reveal () ;

	myGlutIdle();
}

static void hide_ipports_cb ( puObject *cb )
{
	ip_track -> hide () ;
	port_track -> hide () ;
	ip_postument -> hide () ;
	port_postument -> hide () ;
	text9 -> hide () ;
	text99 -> hide () ;
	text999 -> hide () ;
}

/*
static void show_time_cb ( puObject *cb )
{
	timer_text->reveal ();
}

static void hide_time_cb ( puObject *cb )
{
	timer_text->hide ();
}
*/

static void view_top_cb ( puObject *cb )
{
	obj = 4;
}
static void view_postument_cb( puObject *cb )
{
	obj = 3;
}
static void view_track_cb( puObject *cb )
{
	obj = 2;
}
static void view_back_cb( puObject *cb )
{
	obj = 1;
}
static void view_front_cb( puObject *cb )
{
	obj = 0;
}
static void view_reset_cb ( puObject *cb )
{
	zoom = 0.0f;
	roty = 0.0f;
	tx = 0;
	ty = 0;
}
static void view_resetall_cb ( puObject *cb )
{
	front_zoom = 0.0;
	front_roty = 0.0;
	front_tx = 0.0;
	front_ty = 0.0;

	back_zoom = 0.0;
	back_roty = 0.0;
	back_tx = 0.0;
	back_ty = 0.0;

	right_zoom = 0.0;
	right_roty = 0.0;
	right_tx = 0.0;
	right_ty = 0.0;

	left_zoom = 0.0;
	left_roty = 0.0;
	left_tx = 0.0;
	left_ty = 0.0;

	top_zoom = 0.0;
	top_roty = 0.0;
	top_tx = 0.0;
	top_ty = 0.0;

	zoom = 0.0;
	roty = 0.0;
	tx = 0.0;
	ty = 0.0;
}

static void shadow_stenciling_cb( puObject *cb )
{
	obj2 = 1;
}

static void shadow_offset_cb( puObject *cb )
{
	obj2 = 0;
}
static void light_positional_cb( puObject *cb )
{
	obj3 = 0;
}
static void light_directional_cb( puObject *cb )
{
	obj3 = 1;
}

static void exit_cb ( puObject * )
{
	exit ( 1 ) ;
}

static void show_simulation_cb ( puObject *cb )
{
	//show simulation

	slider3->reveal ();
	slider2->reveal ();
	slider1->reveal ();

	slider33->reveal ();
	slider22->reveal ();
	slider11->reveal ();

	slider333->reveal ();
	slider222->reveal ();
	slider111->reveal ();

	text1->reveal();
	text11->reveal();
	text22->reveal();

	myGlutIdle();
}

static void hide_simulation_cb ( puObject *cb )
{
	slider1->hide();
	slider2->hide();
	slider3->hide();

	slider11->hide();
	slider22->hide();
	slider33->hide();

	slider111->hide();
	slider222->hide();
	slider333->hide();

	text1->hide();
	text11->hide();
	text22->hide();
}

static void window_fullscreen_cb ( puObject *cb )
{
	glutFullScreen();
}

static void window_normal_cb ( puObject *cb )
{
	glutPositionWindow(300, 100);
	glutReshapeWindow(900, 700);
}

static void window_small_cb ( puObject *cb )
{
	glutPositionWindow(300, 100);
	glutReshapeWindow(730, 600);
}

static void help_mouse_cb ( puObject *cb )
{
	mk_dialog(
"Rotation - Click Left Button Mouse\nTranslation - Click Right Button Mouse\nMove light - Click Middle Button Mouse\nZoom - Click Left & Right Button Mouse"
			) ;
}

static void help_key_cb ( puObject *cb )
{
	mk_dialog(
"Left Rotation-key F1, Right Rotation- key F2\nNear-key F3, Far-key F4\nMove Right-Right Cursor, Move Left-Left Cursor\nMove Down-Down, Cursor Move Up-Up Cursor"
				) ;
}

static void help_author_cb ( puObject *cb )
{
	mk_dialog (
"Author - Mariusz Zbikowski\n\nmzbikows@gmail.com\n"
				) ;
}
static void readme_cb ( puObject *cb )
{
	mk_dialog (
"Main functions Zbik3D:\n\nConnection with MRROC++\nLoad & save configuration file\nSave & play trajectory file\nGraphic effects"
				) ;
}

static void track_postument_cb ( puObject *cb )
{
	polycrank = 0;
    textcrank->hide() ;
	slider_crank1->hide () ;
	slider_crank2->hide () ;
	slider_crank3->hide () ;
	slider_crank4->hide () ;
	slider_crank5->hide () ;
	slider_crank6->hide () ;
	slider_crank7->hide () ;
	slider_crank8->hide () ;

	front_zoom = 0.0;
	front_roty = 0.0;
	front_tx = 0.0;
	front_ty = 0.0;

	back_zoom = 0.0;
	back_roty = 0.0;
	back_tx = 0.0;
	back_ty = 0.0;

	right_zoom = 0.0;
	right_roty = 0.0;
	right_tx = 0.0;
	right_ty = 0.0;

	left_zoom = 0.0;
	left_roty = 0.0;
	left_tx = 0.0;
	left_ty = 0.0;

	top_zoom = 0.0;
	top_roty = 0.0;
	top_tx = 0.0;
	top_ty = 0.0;

	zoom = 0.0;
	roty = 0.0;
	tx = 0.0;
	ty = 0.0;
}

static void polycrank_cb ( puObject *cb )
{
	polycrank = 1;
	textcrank->reveal () ;
	slider_crank1->reveal () ;
	slider_crank2->reveal () ;
	slider_crank3->reveal () ;
	slider_crank4->reveal () ;
	slider_crank5->reveal () ;
	slider_crank6->reveal () ;
	slider_crank7->reveal () ;
	slider_crank8->reveal () ;

	front_zoom = 0.0;
	front_roty = 0.0;
	front_tx = 0.0;
	front_ty = 0.0;

	back_zoom = 0.0;
	back_roty = 0.0;
	back_tx = 0.0;
	back_ty = 0.0;

	right_zoom = 0.0;
	right_roty = 0.0;
	right_tx = 0.0;
	right_ty = 0.0;

	left_zoom = 0.0;
	left_roty = 0.0;
	left_tx = 0.0;
	left_ty = 0.0;

	top_zoom = 0.0;
	top_roty = 0.0;
	top_tx = 0.0;
	top_ty = 0.0;

	zoom = 0.0;
	roty = 0.0;
	tx = 0.0;
	ty = 0.0;
}

/* Menu bar entries: */
static const char      *file_submenu    [] = { "Exit          Esc", "HideConfiguration", "ShowConfiguration", "Track & Postument", "Polycrank", NULL } ;
static puCallback file_submenu_cb [] = { exit_cb, configuration_hide_cb, configuration_show_cb, track_postument_cb, polycrank_cb, NULL } ;

static const char      *player_submenu    [] = { "HideTrajectoryPlayer", "ShowTrajectoryPlayer", NULL } ;
static puCallback player_submenu_cb [] = { trajectory_player_hide_cb, trajectory_player_show_cb, NULL } ;

static const char      *configs_submenu    [] = { "HideIP_Ports", "ShowIP_Ports", "HideConnection", "ShowConnection", NULL } ;
static puCallback configs_submenu_cb [] = { hide_ipports_cb, show_ipports_cb, hide_connection_cb, show_connection_cb, NULL } ;

static const char      *view_submenu    [] = { "ResetAll", "Reset", "Top", "PostumentSide", "TrackSide", "Back", "Front", NULL } ;
static puCallback view_submenu_cb [] = { view_resetall_cb, view_reset_cb, view_top_cb, view_postument_cb, view_track_cb, view_back_cb, view_front_cb, NULL } ;

static const char      *effects_submenu    [] = { "HideRGB", "ShowRGB", "StencilingShadow", "OffsetShadow", "DirectionalLight", "PositionalLight", NULL } ;
static puCallback effects_submenu_cb [] = { hide_simulation_cb, show_simulation_cb, shadow_stenciling_cb, shadow_offset_cb, light_directional_cb, light_positional_cb, NULL } ;

static const char      *window_submenu    [] = { "Small", "Normal", "Fullscreen", NULL } ;
static puCallback window_submenu_cb [] = { window_small_cb, window_normal_cb, window_fullscreen_cb, NULL } ;

static const char      *help_submenu    [] = { "Mouse", "Keyboard", "Author", "ReadMe", NULL } ;
static puCallback help_submenu_cb [] = { help_mouse_cb, help_key_cb, help_author_cb, readme_cb, NULL } ;

//SAVER TRAJECTORY
void save_trajectory_cb( puObject *pWidget )
{
	pWidget->getValue(&save_trajectory);
	if ( save_trajectory == 1 )
	{
		state_save_trajectory->setValue("Saving NOW");
	}
	if ( save_trajectory == 0 )
	{
		state_save_trajectory->setValue("Not saving NOW");
	}
}

void path_save_trajectory_cb( puObject *pWidget )
{
	pWidget->getValue(filename_save_trajectory);
}

//SAVER CONFIGURATION
void save_configuration_cb( puObject *pWidget )
{
	char str1[255] = "Saved ";

	pWidget->getValue(&save_configuration);
	if ( save_configuration == 0 )
	{
		state_save_configuration->setValue("Not saved yet");
	}
	if ( save_configuration == 1 )
	{
		strcat(str1,filename_save_configuration);
		state_save_configuration->setValue(str1);
		SaveConfigurationFile();
		button_save_configuration->setValue(0);
		save_configuration = 0;
	}
}

void path_save_configuration_cb( puObject *pWidget )
{
	pWidget->getValue(filename_save_configuration);
}

//LOADER CONFIGURATION
void load_configuration_cb( puObject *pWidget )
{
	char str1[255] = "Loaded ";

	pWidget->getValue(&load_configuration);

	if ( load_configuration == 0 )
	{
		state_load_configuration->setValue("Not loaded yet");
	}
	if ( load_configuration == 1 )
	{

if ((fp = fopen( filename_load_configuration, "r" ))==NULL)
{
	state_load_configuration->setValue("File not exists");
	button_load_configuration->setValue(0);
	load_configuration = 0;
	return;
}
		strcat(str1,filename_load_configuration);
		state_load_configuration->setValue(str1);
		button_load_configuration->setValue(0);
		load_configuration = 0;
		LoadConfigurationFile();
	}
}

void path_load_configuration_cb( puObject *pWidget )
{
	pWidget->getValue(filename_load_configuration);
}

//PLAYER TRAJECTORY
void Player_CB( puObject *pWidget )
{
	mouse_stop = 1;

	if(load_trajectory==1)
	{
		pWidget->getValue(&player_place);
		i=(int) (player_place * lines);
	}
	if(load_trajectory==0)
	{
		player->setValue(0);
		state_load_trajectory->setValue("At first load movie");
	}
}

void Speed_CB(  puObject *ob )
{
	mouse_stop = 1;
	int frequency;
	if(load_trajectory==1)
	{
		ob -> getValue (&frequency);
		ob -> setLegend ( ob -> getStringValue () ) ;
		interval=(int)1000/frequency;
	}
	if(load_trajectory==0)
	{
		speed->setValue("50");
		state_load_trajectory->setValue("At first load movie");
	}
}

void replay_cb( puObject *pWidget )
{
if(load_trajectory==1)
{
	pWidget->getValue(&replay_movie);
}
if(load_trajectory==0)
{
	replay->setValue(0);
	state_load_trajectory->setValue("At first load movie");
}
}

//LOAD
void load_trajectory_cb( puObject *pWidget )
{
	char str1[255] = "Loaded ";

	pWidget->getValue(&load_trajectory);

if ((fp = fopen( set_filename_load_trajectory, "r" ))==NULL)
{
		state_load_trajectory->setValue("File not exists");
		button_load_trajectory->setValue(0);
		load_trajectory = 0;
		return;
}
else
{
	if ( load_trajectory == 0 )
	{
		state_load_trajectory->setValue("Not playing NOW");
	}
	if ( load_trajectory == 1 )
	{
		//free memory before
		for (i = 0; i < lines; i++)
		{
			free(tab[i]);
		}
		free(tab);
		tab = NULL;
		strcpy(filename_load_trajectory,set_filename_load_trajectory);
		strcat(str1,filename_load_trajectory);
		state_load_trajectory->setValue(str1);
		i=0;//return always to the begin of movie
		player->setValue(0);
		LoadTrajectoryFile();
		button_load_trajectory->setValue(0);
	}
}
}

//FORWARD
void forward_play_trajectory_cb( puObject *pWidget )
{
	char str1[255] = "Playing forward ";
	char str2[255] = "Stop playing forward ";


if ((fp = fopen( filename_load_trajectory, "r" ))==NULL)
{
	//state_load_trajectory->setValue("File not exists");
	state_load_trajectory->setValue(filename_load_trajectory);
	button_load_trajectory->setValue(0);
	forward_play_trajectory->setValue(0);
	backward_play_trajectory->setValue(0);
	load_trajectory = 0;
	forward_play=0;
	backward_play=0;
	return;
}
else
{
if(load_trajectory==1)
{
	pWidget->getValue(&forward_play);
	if ( forward_play == 1 )
	{
		strcat(str1,filename_load_trajectory);
		state_load_trajectory->setValue(str1);
		backward_play_trajectory->hide();
		button_load_trajectory->hide();
		glutIdleFunc(play_forward);
	}
	if ( forward_play == 0 )
	{
		strcat(str2,filename_load_trajectory);
		state_load_trajectory->setValue(str2);
		button_load_trajectory->reveal();
		backward_play_trajectory->reveal();

		myGlutIdle();
	}
}
if(load_trajectory==0)
{
	forward_play=0;
	forward_play_trajectory->setValue(0);
	state_load_trajectory->setValue("At first load movie");
}
}
}

//BACKWARD
void backward_play_trajectory_cb( puObject *pWidget )
{
	char str1[255] = "Playing backward ";
	char str2[255] = "Stop playing backward ";

if ((fp = fopen( filename_load_trajectory, "r" ))==NULL)
{
	state_load_trajectory->setValue("File not exists");
	button_load_trajectory->setValue(0);
	forward_play_trajectory->setValue(0);
	backward_play_trajectory->setValue(0);
	load_trajectory = 0;
	forward_play=0;
	backward_play=0;

	return;
}
else
{
if(load_trajectory==1)
{
	pWidget->getValue(&backward_play);
	if ( backward_play == 1 )
	{
		strcat(str1,filename_load_trajectory);
		state_load_trajectory->setValue(str1);
		forward_play_trajectory->hide();
		button_load_trajectory->hide();
		glutIdleFunc(play_backward);
	}
	if ( backward_play == 0 )
	{
		strcat(str2,filename_load_trajectory);
		state_load_trajectory->setValue(str2);
		button_load_trajectory->reveal();
		forward_play_trajectory->reveal();

		myGlutIdle();
	}
}
if(load_trajectory==0)
{
	backward_play=0;
	backward_play_trajectory->setValue(0);
	state_load_trajectory->setValue("At first load movie");
}
}
}

void path_load_trajectory_cb( puObject *pWidget )
{
	pWidget->getValue(set_filename_load_trajectory);
}

//MRROCPP CONNECTION
void track_cb( puObject *pWidget )
{
	pWidget->getValue(&track_enabled);
	if(track_enabled == 0) diode_track->setColorScheme(1,0,0,1);//red
	//if(track_enabled == 1)  button_track->hide() ;
}

void postument_cb( puObject *pWidget )
{
	pWidget->getValue(&postument_enabled);
	if(postument_enabled == 0) diode_postument->setColorScheme(1,0,0,1);//red
	//if(postument_enabled == 1)  button_postument->hide() ;
}
//IP&PORTS Robots
void ip_track_cb( puObject *pWidget )
{
	pWidget->getValue(hostip_track);
	myGlutIdle();
}

void ip_postument_cb( puObject *pWidget )
{
	pWidget->getValue(hostip_postument);
	myGlutIdle();
}

void port_track_cb( puObject *pWidget )
{
	pWidget->getValue(&hostport_track);
	myGlutIdle();
}

void port_postument_cb( puObject *pWidget )
{
	pWidget->getValue(&hostport_postument);
	myGlutIdle();
}

//Background
void Red_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&r);
	myGlutIdle();
}

void Green_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&g);
	myGlutIdle();
}

void Blue_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&b);
	myGlutIdle();
}

//LightTrack
void LightTrackRed_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light1_diffuse[0]);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse );
	myGlutIdle();
}

void LightTrackGreen_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light1_diffuse[1]);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse );
	myGlutIdle();
}

void LightTrackBlue_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light1_diffuse[2]);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse );
	myGlutIdle();
}

//LightPostument
void LightPostumentRed_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light2_diffuse[0]);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse );
	myGlutIdle();
}

void LightPostumentGreen_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light2_diffuse[1]);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse );
	myGlutIdle();
}

void LightPostumentBlue_CB( puObject *pWidget )
{
	mouse_stop = 1;
	pWidget->getValue(&light2_diffuse[2]);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse );
	myGlutIdle();
}

void crank1_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank1);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank2_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank2);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank3_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank3);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank4_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank4);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank5_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank5);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank6_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank6);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank7_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank7);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

void crank8_cb( puObject *ob )
{
	mouse_stop = 1;
	ob->getValue(&crank8);
	ob -> setLegend ( ob -> getStringValue () ) ;
	myGlutIdle();
}

int main_display(int argc,char** argv)
{
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL | GLUT_MULTISAMPLE);
	glutInitWindowSize(730,600);
	glutInitWindowPosition(300,100);

	main_window = glutCreateWindow( "Zbik3D Graphical multi-robot simulation environment MAIL: mzbikows@gmail.com");

	init();

	glutDisplayFunc(redraw);
	glutMouseFunc(MouseButton);
	glutMotionFunc(MouseMove);
	glutKeyboardFunc(keyfn);
	glutSpecialFunc(pressKey);
//	glutVisibilityFunc(visible);

	// redrawing is handled from timer instad of idle callback
	glutTimerFunc(1000/DISPLAY_FREQUENCY, timerCallback, 0);
//	glutIdleFunc(redraw);

#ifdef GL_VERSION_1_1
	if (supportsOneDotOne() && !forceExtension) {
	polygonOffsetVersion = ONE_DOT_ONE;
	glPolygonOffset(-2.0, -1.0);
	} else
#endif
{
#ifdef GL_EXT_polygon_offset
	/* check for the polygon offset extension */
	if (glutExtensionSupported("GL_EXT_polygon_offset")) {
	polygonOffsetVersion = EXTENSION;
	// glPolygonOffsetEXT(-0.1, -0.002);
	} else
#endif
	{
		polygonOffsetVersion = MISSING;
		printf("\nKomunikat: Missing polygon offset.\n");
		printf("           Expect shadow depth aliasing artifacts.\n\n");
	}
}
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glLineWidth(3.0);
	glutReshapeFunc(changeSize);

	/* Setup floor plane for projected shadow calculations. */
	findPlane(floorPlane, floorVertices[1], floorVertices[2], floorVertices[3]);

	/****************************************/
	/*       Set up OpenGL lights           */
	/****************************************/

	glEnable(GL_LIGHTING);
	glEnable( GL_NORMALIZE );

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

	glEnable(GL_LIGHTING);
	glEnable( GL_NORMALIZE );
	glEnable(GL_LIGHT2);
	glLightfv(GL_LIGHT2, GL_AMBIENT, light2_ambient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

	/****************************************/
	/*          Enable z-buferring          */
	/****************************************/

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glLineWidth(5.0);

	/****************************************/
	/*                PUI                   */
	/****************************************/
	puInit () ;

	puSetDefaultStyle        ( PUSTYLE_SMALL_SHADED ) ;
	puSetDefaultColourScheme ( 0.3f, 0.4f, 0.6f, 1.0f) ;

	timer_text = new puText ( 0, 80 ) ;
	timer_text -> setColour ( PUCOL_LABEL, 0.0, 0.0, 0.0 ) ;
	timer_text->hide ();

	/* Make the menu bar */

	main_menu_bar = new puMenuBar () ;
	{
		main_menu_bar -> add_submenu ( "Configuration", (char **) file_submenu, file_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "TrajectoryPlayer", (char **) player_submenu, player_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "MRROC++Connection", (char **) configs_submenu, configs_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "View", (char **) view_submenu, view_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "GraphicEffects", (char **) effects_submenu, effects_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "WindowSize", (char **) window_submenu, window_submenu_cb ) ;
		main_menu_bar -> add_submenu ( "Help", (char **) help_submenu, help_submenu_cb ) ;
	}
	main_menu_bar -> close () ;

//Track
	text999 = new puText( 40, 120 );
	text999->setLabel("Track");
	text999->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text999->hide();

	ip_track = new puInput ( 0, 100, 120, 120 ) ;
	ip_track->setValue( hostip_track );
	ip_track->setLabel ( "IP" ) ;
	ip_track->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	ip_track->setCallback(ip_track_cb);
	ip_track->hide ();

	port_track = new puInput ( 0, 80, 120, 100 ) ;
	port_track->setValue( hostport_track  );
	port_track->setLabel ( "Port" ) ;
	port_track->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	port_track->setCallback(port_track_cb);
	port_track->hide ();

//Postument
	text99 = new puText( 20, 60 );
	text99->setLabel("Postument");
	text99->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text99->hide();

	ip_postument = new puInput ( 0, 40, 120, 60 ) ;
	ip_postument->setValue( hostip_postument );
	ip_postument->setLabel ( "IP" ) ;
	ip_postument->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	ip_postument->setCallback(ip_postument_cb);
	ip_postument->hide ();

	port_postument = new puInput ( 0, 20, 120, 40 ) ;
	port_postument->setValue( hostport_postument );
	port_postument->setLabel ( "Port" ) ;
	port_postument->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	port_postument->setCallback(port_postument_cb);
	port_postument->hide ();

//Tips
	text9 = new puText( 0, 0 );
	text9->setLabel("After input press ENTER");
	text9->setColour ( PUCOL_LABEL, 1.0, 0.0, 0.0 ) ;
	text9->hide();

//Saver Trejectory
	button_save_trajectory = new puButton ( 222, 241, 222+17, 241+17, PUBUTTON_VCHECK ) ;
	button_save_trajectory->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_save_trajectory->setLabel ( "Save" ) ;
	button_save_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_save_trajectory->setValue ( 0 ) ;
	button_save_trajectory->setCallback ( save_trajectory_cb );
	button_save_trajectory->hide() ;

	path_save_trajectory = new puInput ( 0, 240, 222, 260 ) ;
	path_save_trajectory->setValue( filename_save_trajectory );
	path_save_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	path_save_trajectory->setCallback(path_save_trajectory_cb);
	path_save_trajectory->hide ();

	state_save_trajectory = new puInput ( 0, 220, 240, 240 ) ;
	state_save_trajectory->setValue( "Not saving NOW" );
	state_save_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	state_save_trajectory->disableInput();
	state_save_trajectory->hide ();

	text0 = new puText( 0, 257 );
	text0->setLabel(" SAVER TRAJECTORY");
	text0->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text0->hide();

//Loader Configuration
	button_load_configuration = new puButton ( 222, 361, 222+17, 361+17, PUBUTTON_VCHECK ) ;
	button_load_configuration->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_load_configuration->setLabel ( "Load" ) ;
	button_load_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_load_configuration->setValue ( 0 ) ;
	button_load_configuration->setCallback ( load_configuration_cb );
	button_load_configuration->hide() ;

	path_load_configuration = new puInput ( 0, 360, 222, 380 ) ;
	path_load_configuration->setValue( filename_load_configuration );
	path_load_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	path_load_configuration->setCallback(path_load_configuration_cb);
	path_load_configuration->hide ();

	state_load_configuration = new puInput ( 0, 340, 240, 360 ) ;
	state_load_configuration->setValue( "Not loading NOW" );
	state_load_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	state_load_configuration->disableInput();
	state_load_configuration->hide ();

	text000 = new puText( 0, 377 );
	text000->setLabel("LOADER CONFIGURATION");
	text000->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text000->hide();

//Saver Configuration
	button_save_configuration = new puButton ( 222, 301, 222+17, 301+17, PUBUTTON_VCHECK ) ;
	button_save_configuration->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_save_configuration->setLabel ( "Save" ) ;
	button_save_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_save_configuration->setValue ( 0 ) ;
	button_save_configuration->setCallback ( save_configuration_cb );
	button_save_configuration->hide() ;

	path_save_configuration = new puInput ( 0, 300, 222, 320 ) ;
	path_save_configuration->setValue( filename_save_configuration );
	path_save_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	path_save_configuration->setCallback(path_save_configuration_cb);
	path_save_configuration->hide ();

	state_save_configuration = new puInput ( 0, 280, 240, 300 ) ;
	state_save_configuration->setValue( "Not saved yet" );
	state_save_configuration->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	state_save_configuration->disableInput();
	state_save_configuration->hide ();

	text00 = new puText( 0, 317 );
	text00->setLabel("SAVER CONFIGURATION");
	text00->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text00->hide();

//Trajectory Player
	player = new puSlider( 0, 100, 20 );
	player->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	player->setCallback(Player_CB);
	player->setSize(220,20);
	player->setPosition(0,198);
	player->setValue("0.0");
	player->hide();

	speed = new puSlider( 0, 100, 20 );
	speed->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	speed->setMaxValue   ( 100.0f ) ;
	speed->setMinValue   ( 10.0f ) ;
	speed->setStepSize   ( 1.0f ) ;
	speed->setCBMode     ( PUSLIDER_ALWAYS ) ;
	speed->setLabel ( "Speed[Hz]" ) ;
	speed->setCallback(Speed_CB);
	speed->setSize(75,20);
	speed->setPosition(0,140);
	speed->setValue("50");
	speed->setLegend("50");
	speed->hide();

	button_load_trajectory = new puButton ( 222, 241, 222+17, 241+17, PUBUTTON_VCHECK ) ;
	button_load_trajectory->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_load_trajectory->setLabel ( "Load" ) ;
	button_load_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_load_trajectory->setValue ( 0 ) ;
	button_load_trajectory->setCallback ( load_trajectory_cb );
	button_load_trajectory->hide() ;

	forward_play_trajectory = new puButton ( 0, 180, 17, 180+17, PUBUTTON_VCHECK ) ;
	forward_play_trajectory->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	forward_play_trajectory->setLabel ( "Play FORWARD" ) ;
	forward_play_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	forward_play_trajectory->setCallback( forward_play_trajectory_cb ) ;
	forward_play_trajectory->hide ();

	backward_play_trajectory = new puButton ( 0, 161, 17, 161+17, PUBUTTON_VCHECK ) ;
	backward_play_trajectory->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	backward_play_trajectory->setLabel ( "Play BACKWARD" ) ;
	backward_play_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	backward_play_trajectory->setCallback( backward_play_trajectory_cb );
	backward_play_trajectory->hide ();

	replay = new puButton ( 152, 143, 152+17, 143+17, PUBUTTON_VCHECK ) ;
	replay->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	replay->setLabel ( "Replay" ) ;
	replay->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	replay->setCallback(replay_cb);
	replay->hide ();

	path_load_trajectory = new puInput ( 0, 240, 222, 260 ) ;
	path_load_trajectory->setValue( filename_load_trajectory );
	path_load_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	path_load_trajectory->setCallback(path_load_trajectory_cb);
	path_load_trajectory->hide ();

	state_load_trajectory = new puInput ( 0, 220, 240, 240 ) ;
	state_load_trajectory->setValue( "Not playing NOW" );
	state_load_trajectory->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	state_load_trajectory->disableInput();
	state_load_trajectory->hide ();

	text0000 = new puText( 0, 257 );
	text0000->setLabel("PLAYER TRAJECTORY");
	text0000->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text0000->hide();

//Connection with MRROC++
	button_track = new puButton ( 10, 146, 10+16, 146+16, PUBUTTON_VCHECK ) ;
	button_track->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_track->setLabel ( "Track" ) ;
	button_track->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_track->setValue ( 0 ) ;
	button_track->setCallback ( track_cb );
	button_track->hide() ;

	button_postument = new puButton ( 80, 146, 80+16, 146+16, PUBUTTON_VCHECK ) ;
	button_postument->setLabelPlace ( PUPLACE_CENTERED_RIGHT ) ;
	button_postument->setLabel ( "Postument" ) ;
	button_postument->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	button_postument->setValue ( 0 ) ;
	button_postument->setCallback ( postument_cb );
	button_postument->hide() ;

	diode_track = new puFrame ( 0, 163, 50, 213 ) ;
	diode_track->setColorScheme(1,0,0,1);
	diode_track->hide();

	diode_postument = new puFrame ( 67, 163, 117, 213 ) ;
	diode_postument->setColorScheme(1,0,0,1);
	diode_postument->hide();

//Simulation
//Color Background
	text1 = new puText( 532, 100 );
	text1->setLabel("Background Color");
	text1->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text1->hide();

	slider3 = new puSlider( 0, 100, 20 );
	slider3->setLabel("B");
	slider3->setColour ( PUCOL_LABEL, 0.0, 0.0, 1.0 ) ;
	slider3->setCallback(Blue_CB);
	slider3->setSize(100,20);
	slider3->setPosition(550,0);
	slider3->hide();
	/* create a new vertical slider. int min = 0, int max = 100 */
	slider2 = new puSlider( 0, 100, 20 );
	slider2->setLabel("G");
	slider2->setColour ( PUCOL_LABEL, 0.0, 1.0, 0.0 ) ;
	slider2->setCallback(Green_CB);
	slider2->setSize(100,20);
	slider2->setPosition(550,40);
	slider2->hide();
	/* create a new horizontal slider. int min = 0, int max = 100 */
	slider1 = new puSlider( 0, 100, 20 );
	slider1->setLabel("R");
	slider1->setColour ( PUCOL_LABEL, 1.0, 0.0, 0.0 ) ;
	slider1->setCallback(Red_CB);
	slider1->setSize(100,20);
	slider1->setPosition(550,80);
	slider1->hide();

//Light Track
	text11 = new puText( 250, 100 );
	text11->setLabel("Track Light");
	text11->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text11->hide();

	slider33 = new puSlider( 0, 100, 20 );
	slider33->setLabel("B");
	slider33->setColour ( PUCOL_LABEL, 0.0, 0.0, 1.0 ) ;
	slider33->setCallback(LightTrackBlue_CB);
	slider33->setSize(100,20);
	slider33->setPosition(250,0);
	slider33->setValue(1);
	slider33->hide();
	/* create a new vertical slider. int min = 0, int max = 100 */
	slider22 = new puSlider( 0, 100, 20 );
	slider22->setLabel("G");
	slider22->setColour ( PUCOL_LABEL, 0.0, 1.0, 0.0 ) ;
	slider22->setCallback(LightTrackGreen_CB);
	slider22->setSize(100,20);
	slider22->setPosition(250,40);
	slider22->setValue(1);
	slider22->hide();
	/* create a new horizontal slider. int min = 0, int max = 100 */
	slider11 = new puSlider( 0, 100, 20 );
	slider11->setLabel("R");
	slider11->setColour ( PUCOL_LABEL, 1.0, 0.0, 0.0 ) ;
	slider11->setCallback(LightTrackRed_CB);
	slider11->setSize(100,20);
	slider11->setPosition(250,80);
	slider11->setValue(1);
	slider11->hide();
//Light Postument
	text22 = new puText( 385, 100 );
	text22->setLabel("Postument Light");
	text22->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	text22->hide();

	slider333 = new puSlider( 0, 100, 20 );
	slider333->setLabel("B");
	slider333->setColour ( PUCOL_LABEL, 0.0, 0.0, 1.0 ) ;
	slider333->setCallback(LightPostumentBlue_CB);
	slider333->setSize(100,20);
	slider333->setPosition(400,0);
	slider333->setValue(1);
	slider333->hide();
	/* create a new vertical slider. int min = 0, int max = 100 */
	slider222 = new puSlider( 0, 100, 20 );
	slider222->setLabel("G");
	slider222->setColour ( PUCOL_LABEL, 0.0, 1.0, 0.0 ) ;
	slider222->setCallback(LightPostumentGreen_CB);
	slider222->setSize(100,20);
	slider222->setPosition(400,40);
	slider222->setValue(1);
	slider222->hide();
	/* create a new horizontal slider. int min = 0, int max = 100 */
	slider111 = new puSlider( 0, 100, 20 );
	slider111->setLabel("R");
	slider111->setColour ( PUCOL_LABEL, 1.0, 0.0, 0.0 ) ;
	slider111->setCallback(LightPostumentRed_CB);
	slider111->setSize(100,20);
	slider111->setPosition(400,80);
	slider111->setValue(1);
	slider111->hide();

	textcrank = new puText( 0, 160 );
	textcrank->setLabel("Polycrank Simulation");
	textcrank->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	textcrank->reveal();

	slider_crank1 = new puSlider( 0, 100, 20 );
	slider_crank1->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank1->setMaxValue   ( 180.0f ) ;
	slider_crank1->setMinValue   ( -180.0f ) ;
	slider_crank1->setStepSize   ( 1.0f ) ;
	slider_crank1->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank1->setLabel ( "Q1 (Degrees)" ) ;
	slider_crank1->setCallback(crank1_cb);
	slider_crank1->setSize(75,20);
	slider_crank1->setPosition(0,140);
	slider_crank1->setValue("0");
	slider_crank1->setLegend("0");
	slider_crank1->reveal();

	slider_crank2 = new puSlider( 0, 100, 20 );
	slider_crank2->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank2->setMaxValue   ( 180.0f ) ;
	slider_crank2->setMinValue   ( -180.0f ) ;
	slider_crank2->setStepSize   ( 1.0f ) ;
	slider_crank2->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank2->setLabel ( "Q2 (Degrees)" ) ;
	slider_crank2->setCallback(crank2_cb);
	slider_crank2->setSize(75,20);
	slider_crank2->setPosition(0,120);
	slider_crank2->setValue("0");
	slider_crank2->setLegend("0");
	slider_crank2->reveal();

	slider_crank3 = new puSlider( 0, 100, 20 );
	slider_crank3->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank3->setMaxValue   ( 180.0f ) ;
	slider_crank3->setMinValue   ( -180.0f ) ;
	slider_crank3->setStepSize   ( 1.0f ) ;
	slider_crank3->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank3->setLabel ( "Q3 (Degrees)" ) ;
	slider_crank3->setCallback(crank3_cb);
	slider_crank3->setSize(75,20);
	slider_crank3->setPosition(0,100);
	slider_crank3->setValue("0");
	slider_crank3->setLegend("0");
	slider_crank3->reveal();

	slider_crank4 = new puSlider( 0, 100, 20 );
	slider_crank4->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank4->setMaxValue   ( 180.0f ) ;
	slider_crank4->setMinValue   ( -180.0f ) ;
	slider_crank4->setStepSize   ( 1.0f ) ;
	slider_crank4->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank4->setLabel ( "Q4 (Degrees)" ) ;
	slider_crank4->setCallback(crank4_cb);
	slider_crank4->setSize(75,20);
	slider_crank4->setPosition(0,80);
	slider_crank4->setValue("0");
	slider_crank4->setLegend("0");
	slider_crank4->reveal();

	slider_crank5 = new puSlider( 0, 100, 20 );
	slider_crank5->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank5->setMaxValue   ( 180.0f ) ;
	slider_crank5->setMinValue   ( -180.0f ) ;
	slider_crank5->setStepSize   ( 1.0f ) ;
	slider_crank5->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank5->setLabel ( "Q5 (Degrees)" ) ;
	slider_crank5->setCallback(crank5_cb);
	slider_crank5->setSize(75,20);
	slider_crank5->setPosition(0,60);
	slider_crank5->setValue("0");
	slider_crank5->setLegend("0");
	slider_crank5->reveal();

	slider_crank6 = new puSlider( 0, 100, 20 );
	slider_crank6->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank6->setMaxValue   ( 180.0f ) ;
	slider_crank6->setMinValue   ( -180.0f ) ;
	slider_crank6->setStepSize   ( 1.0f ) ;
	slider_crank6->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank6->setLabel ( "Q6 (Degrees)" ) ;
	slider_crank6->setCallback(crank6_cb);
	slider_crank6->setSize(75,20);
	slider_crank6->setPosition(0,40);
	slider_crank6->setValue("0");
	slider_crank6->setLegend("0");
	slider_crank6->reveal();

	slider_crank7 = new puSlider( 0, 100, 20 );
	slider_crank7->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank7->setMaxValue   ( 180.0f ) ;
	slider_crank7->setMinValue   ( -180.0f ) ;
	slider_crank7->setStepSize   ( 1.0f ) ;
	slider_crank7->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank7->setLabel ( "Q7 (Degrees)" ) ;
	slider_crank7->setCallback(crank7_cb);
	slider_crank7->setSize(75,20);
	slider_crank7->setPosition(0,20);
	slider_crank7->setValue("0");
	slider_crank7->setLegend("0");
	slider_crank7->reveal();

	slider_crank8 = new puSlider( 0, 100, 20 );
	slider_crank8->setColour ( PUCOL_LABEL, 1.0, 1.0, 1.0 ) ;
	slider_crank8->setMaxValue   ( 0.12f ) ;
	slider_crank8->setMinValue   ( 0.05f ) ;
	slider_crank8->setStepSize   ( 0.01f ) ;
	slider_crank8->setCBMode     ( PUSLIDER_ALWAYS ) ;
	slider_crank8->setLabel ( "Q8 (Metre)" ) ;
	slider_crank8->setCallback(crank8_cb);
	slider_crank8->setSize(75,20);
	slider_crank8->setPosition(0,0);
	slider_crank8->setValue("0.12");
	slider_crank8->setLegend("0.12");
	slider_crank8->reveal();

	glutMainLoop();

	glutDestroyWindow(main_window);

	return 0;
}

//funkcja pobierajaca poolzenia robota TRACK z serwera EDP i zapisujaca dane do tablicy w odpowiednim miejscu
int zapis_track(void)
{
if (track_enabled == 1)
{
		//Buffers
		char msg[2] = "1" ;               /*message which was send*/
		char Buffer[36];
		char bufor[4];                   /* Temporary Buffer for string */
		//Flags
		int timeout_counter = 0;
		int len = 0;
		int shiftBuffer = 0;                 /* shift in Byffer per 4 bytes */
		float current=0;
		int synchronization = 0;
		int i=0;
		struct sockaddr_in servaddr;

		bzero(&servaddr,sizeof(servaddr));
		servaddr.sin_family      = AF_INET;
		servaddr.sin_addr.s_addr = inet_addr(hostip_track);
		servaddr.sin_port        = htons(hostport_track);

	while(timeout_counter < 200)
	{
		if (sendto(socket_track, msg,strlen(msg), 0, (struct sockaddr *)&servaddr,sizeof(servaddr)) != 1) {
			perror("sendto()");
			diode_track->setColorScheme(1,0,0,1);
			track_enabled = 0;
			mk_dialog ( "Error connecting to TRACK" ) ;
			button_track->setValue ( 0 ) ;
			return -1;
		}

		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(socket_track, &rfds);

		struct timeval timeout;

		timeout.tv_sec  = 0;
		timeout.tv_usec = 20000;
 
		int r = select(socket_track+1, &rfds, NULL, NULL, &timeout);
		
		if(r < 0) {
			perror("select()");
			return -1;
		} else if (r == 0) {
			timeout_counter++;
			continue;
		}
		
		assert(FD_ISSET(socket_track, &rfds));

		len=recvfrom(socket_track,Buffer,sizeof(Buffer),0,NULL,NULL);

		if (len>0)
		{
			diode_track->setColorScheme(0,1,0,1);
			synchronization=Buffer[0];

			if (synchronization==0)//timeout=4s
			{
				diode_track->setColorScheme(1,0,0,1);//red
				track_enabled = 0;
				mk_dialog ( "                 TRACK not synchronized" ) ;
				button_track->setValue ( 0 ) ;
				return 0;
			}
			pthread_mutex_lock(&access_tablica);
			for (shiftBuffer=4; shiftBuffer<=32; shiftBuffer+=4)
			{
				for(i=0; i<4; i++)
				{
					bufor[i]=Buffer[shiftBuffer+i];
				}
				memcpy(&current,bufor,4);

				switch((shiftBuffer/4)-1)
				{
				case 0://Trackd1 TorJezdny
					joints[0]=(float)12.5*((current+0.124f)/1.333f);
					break;
				case 1://TrackQ2 KorpusObrot
					joints[1]=(float)(current*180.0f/M_PI);
					break;
				case 2: //TrackQ3 KolumnaPrzodTyl
					joints[2]=(float)-(current*180.0f/M_PI)-90;
					break;
				case 3: //TrackQ4 RamieGoraDol
					joints[3]=(float)-(current*180.0/M_PI);
					break;
				case 4: //TrackQ5  LacznikGoraDol
					joints[4]=(float)-(current*180.0/M_PI);
					break;
				case 5: //TrackQ6 ChwytakObrot
					joints[5]=(float)-(current*180.0/M_PI)+270;
					break;
				case 6: //TrackQ7 KiscObrot
					joints[6]=(float)-(current*180.0/M_PI)+90;
					break;
				case 7: //TrackQ8 Palec1, Palec2
					joints[7] =(float) 0.2f - (0.2f*((current-0.054f)/(0.090f-0.054f)));
					break;
				}//end of switch
			}//end of for
			pthread_mutex_unlock(&access_tablica);
			if(track_enabled == 0) diode_track->setColorScheme(1,0,0,1);//red
			return 0;
		}//end of if len>0
		else
		{
			perror("track read failed");

			ulMilliSecondSleep(20);
		}
	}//end of while

	diode_track->setColorScheme(1,0,0,1);//red
	track_enabled = 0;
	mk_dialog ( "       Timeout on TRACK, try later " ) ;
	button_track->setValue ( 0 ) ;

	return -1;
}//end of track enabled
if (track_enabled == 0)
{ //**************TRACK*SYNCHRO****************************
	pthread_mutex_lock(&access_tablica);
	joints[0] = 12.5*((0.0+0.124f)/1.333f);     //Track_d1 Td1 TorJezdny       joints[0]
	joints[1] = ((-0.087*180)/M_PI)+10;         //Track_q2 Tq2 KorpusObrot     joints[1]
	joints[2] =	-90-((-1.542*180)/M_PI);        //Track_q3 Tq3 KolumnaPrzodTyl joints[2]
	joints[3] = -((0.024*180)/M_PI);            //Track_q4 Tq4 RamieGoraDol    joints[3]
	joints[4] = -((1.219*180)/M_PI);            //Track_q5 Tq5 LacznikGoraDol  joints[4]
	joints[5] = -((2.591*180)/M_PI)+270;        //Track_q6 Tq6 ChwytakObrot    joints[5]
	joints[6] = -((-2.664*180)/M_PI)+90;        //Track_q7 Tq7 KiscObrot       joints[6]
	joints[7] = 0.2f - (0.2f*((0.074-0.054f)/(0.090f-0.054f)));//Track_q8 Tq8 Palec1, Palec2  joints[7]
	track_enabled = 2;//mode simulation
	pthread_mutex_unlock(&access_tablica);
}
return 0;
}

//funkcja pobierajaca polzenia robota POSTUMENT z serwera EDP i zapisujaca dane do tablicy w odpowiednim miejscu
int zapis_postument(void)
{

if (postument_enabled == 1)
{
		//Buffers
		char msg[2] = "1" ;               /*message which was send*/
		char Buffer[36];
		char bufor[4];                   /* Temporary Buffer for string */
		//Flags
		int timeout_counter = 0;
		int len = 0;
		int shiftBuffer = 0;                 /* shift in Byffer per 4 bytes */
		float current=0;
		int synchronization = 0;
		int i=0;
		struct sockaddr_in servaddr;

		bzero(&servaddr,sizeof(servaddr));
		servaddr.sin_family      = AF_INET;
		servaddr.sin_addr.s_addr = inet_addr(hostip_postument);
		servaddr.sin_port        = htons(hostport_postument);


	while(timeout_counter < 200)
	{
		if (sendto(socket_postument, msg,strlen(msg), 0, (struct sockaddr *)&servaddr,sizeof(servaddr)) != 1) {
			perror("sendto()");
			diode_postument->setColorScheme(1,0,0,1);
			postument_enabled = 0;
			mk_dialog2 ( "Error connecting to POSTUMENT" ) ;
			button_postument->setValue ( 0 ) ;
			return -1;
		}

		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(socket_postument, &rfds);

		struct timeval timeout;

		timeout.tv_sec  = 0;
		timeout.tv_usec = 20000;

		int r = select(socket_postument+1, &rfds, NULL, NULL, &timeout);
		
		if(r < 0) {
			perror("select()");
			return -1;
		} else if (r == 0) {
			timeout_counter++;
			continue;
		}
		
		assert(FD_ISSET(socket_postument, &rfds));

		len=recvfrom(socket_postument,Buffer,sizeof(Buffer),0,NULL,NULL);

		if (len>0)
		{
			diode_postument->setColorScheme(0,1,0,1);//green
			synchronization=Buffer[0];

			if (synchronization==0)
			{
				diode_postument->setColorScheme(1,0,0,1);//red
				postument_enabled = 0;
				mk_dialog2 ( "        POSTUMENT not synchronized" ) ;
				button_postument->setValue ( 0 ) ;

				return 0;
			}
			pthread_mutex_lock(&access_tablica);
			for (shiftBuffer=4; shiftBuffer<=32; shiftBuffer+=4)
			{
				for(i=0; i<4; i++)
				{
					bufor[i]=Buffer[shiftBuffer+i];
				}
				memcpy(&current,bufor,4);

				switch((shiftBuffer/4)+7)
				{
				case 8://PostumentQ2 KorpusObrot
					joints[8]=(float)(current*180.0/M_PI);
					break;
				case 9: //PostumentQ3 KolumnaPrzodTyl
					joints[9]=(float)-(current*180.0/M_PI)-90;
					break;
				case 10: //PostumentQ4 RamieGoraDol
					joints[10]=(float)-(current*180.0/M_PI);
					break;
				case 11: //PostumentQ5  LacznikGoraDol
					joints[11]=(float)-(current*180.0/M_PI);
					break;
				case 12: //PostumentQ6 ChwytakObrot
					joints[12]=(float)-(current*180.0/M_PI)+270;
					break;
				case 13: //PostumentQ7 KiscObrot
					joints[13]=(float)-(current*180.0/M_PI)+90;
					break;
				case 14: //PostumentQ8 Palec1, Palec2
					joints[14] =(float) 0.2f - 0.2f*((current-0.054f)/(0.090f-0.054f));
					break;
				}
			}
			pthread_mutex_unlock(&access_tablica);
			if(postument_enabled == 0) diode_postument->setColorScheme(1,0,0,1);//red
			return 0;
		}
		else
		{
			perror("postument read failed");

			ulMilliSecondSleep(20);
		}
	}//end of while

	diode_postument->setColorScheme(1,0,0,1);//red
	postument_enabled = 0;
	mk_dialog2 ( "       Timeout on POSTUMENT, try later " ) ;
	button_postument->setValue ( 0 ) ;

	return -1;
}//end of postument enabled
if (postument_enabled == 0)
{//***************POSTUMENT*SYNCHRO***********************
	pthread_mutex_lock(&access_tablica);
	joints[8]  = ((-0.101*180)/M_PI);       //Postu_q1 Pq1 KorpusObrot     joints[8]
	joints[9]  = -90-((-1.542*180)/M_PI);   //Postu_q2 Pq2 KolumnaPrzodTyl joints[9]
	joints[10] = -((0.049*180)/M_PI);       //Postu_q3 Pq3 RamieGoraDol    joints[10]
	joints[11] = -((1.198*180)/M_PI);       //Postu_q4 Pq4 LacznikGoraDol  joints[11]
	joints[12] = -((2.101*180)/M_PI)+270;   //Postu_q5 Pq5 ChwytakObrot    joints[12]
	joints[13] = -((-2.749*180)/M_PI)+90;   //Postu_q6 Pq6 KiscObrot       joints[13]
	joints[14] = 0.2f - (0.2f*((0.074-0.054f)/(0.090f-0.054f)));//Postu_q7 Pq7 Palec1, Palec2  joints[14]
	pthread_mutex_unlock(&access_tablica);
	postument_enabled = 2;//mode simulation
}
return 0;
}

void *track(void *unused)
{
	while(1)
	{
		if(pthread_mutex_lock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_lock() failed, exiting thread\n");
			return NULL;
		}
		while(!track_synch) {
			if(pthread_cond_wait(&receive_cond, &receive_mtx)) {
				fprintf(stderr, "pthread_cond_wait() failed, exiting thread\n");
				return NULL;
			}
		}
		track_synch = false;
		if(pthread_mutex_unlock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_unlock() fialed, exiting thread\n");
		}
		zapis_track();
//		printf("track()\n");
	}

	return NULL;
}

void *postument(void *unused)
{
	while(1)
	{
		if(pthread_mutex_lock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_lock() failed, exiting thread\n");
			return NULL;
		}
		while(!postument_synch) {
			if(pthread_cond_wait(&receive_cond, &receive_mtx)) {
				fprintf(stderr, "pthread_cond_wait() failed, exiting thread\n");
				return NULL;
			}
		}
		postument_synch = false;
		if(pthread_mutex_unlock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_unlock() fialed, exiting thread\n");
		}
		zapis_postument();
//		printf("postument()\n");
	}

	return NULL;
}

void *synchronize(void *unused)
{
	while(1)
	{
//		printf("synchronize()\n");
		ulMilliSecondSleep(20);
		if ( save_trajectory == 1 )
		{
			SaveTrajectoryFile();
		}
		if(pthread_mutex_lock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_lock() failed, exiting thread\n");
			return NULL;
		}

		track_synch = true;
		postument_synch = true;

		if(pthread_cond_broadcast(&receive_cond)) {
			fprintf(stderr, "pthread_cond_broadcast() failed, exiting thread\n");
		}

		if(pthread_mutex_unlock(&receive_mtx)) {
			fprintf(stderr, "pthread_mutex_lock() failed, exiting thread\n");
			return NULL;
		}

//		redraw();
	}
	return NULL;
}

static void timerCallback (int value)
{
	/* Do timer processing */
	/* maybe glutPostRedisplay(), if necessary */
	/* call back again after elapsedUSecs have passed */
	glutPostRedisplay();
	glutTimerFunc (1000/DISPLAY_FREQUENCY, timerCallback, value);

	/*
	static double last_display_time;
	double display_time;

	struct timeval tv;
	if (gettimeofday(&tv, NULL) == -1) {
		perror("gettimeofday()");
	}
	display_time  = tv.tv_sec + (((double) tv.tv_usec) / 1000000.0);
	double period = display_time - last_display_time;
	printf("\rdisplay frequency %4.0f", 1.0/period);
	fflush(stdout);
	last_display_time = display_time;
	*/
}


int main(int argc, char *argv[])
{
	pthread_t th1, th2, th3;

	// Initialize UDP sockets
	socket_postument = socket(AF_INET,SOCK_DGRAM,0);
	assert(socket_postument >= 0);

	socket_track = socket(AF_INET,SOCK_DGRAM,0);
	assert(socket_track >= 0);

	pthread_mutex_init(&access_tablica, NULL);
	pthread_mutex_init(&receive_mtx, NULL);
	pthread_cond_init(&receive_cond, NULL);

	pthread_create(&th1, NULL, track, NULL);       //watek pobierajacy aktualne polozenia robota track
	pthread_create(&th2, NULL, postument, NULL);   //watek pobierajacy aktualne polozenia robota postument
	pthread_create(&th3, NULL, synchronize, NULL); //watek odwieszjacy co 20 ms watki track, postument

	main_display(argc, argv);

	pthread_join(th1, NULL);
	pthread_join(th2, NULL);
	pthread_join(th3, NULL);

	pthread_mutex_destroy(&access_tablica);
	pthread_mutex_destroy(&receive_mtx);
	pthread_cond_destroy(&receive_cond);

	close(socket_postument);
	close(socket_track);

	return 0;
}
