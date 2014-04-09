/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

//! These are user defined include files
//! Included in double quotes - the path to find these has to be given at compile time
#include <iostream>
#include "render.hpp"
#include "cs296_base.hpp"
#include "callbacks.hpp"
#include <sys/time.h>
//! GLUI is the library used for drawing the GUI
//! Learn more about GLUI by reading the GLUI documentation
//! Learn to use preprocessor diectives to make your code portable
#ifndef __APPLE__
#include "GL/glui.h"
#else
#include "GL/glui.h"
#endif

//! These are standard include files
//! These are usually available at standard system paths like /usr/include
//! Read about the use of include files in C++
#include <cstdio>


//! Notice the use of extern. Why is it used here?
namespace cs296
{
  extern int32 test_index;
  extern int32 test_selection;
  extern int32 test_count;
  extern cs296::sim_t* entry;
  extern cs296::base_sim_t* test;
  extern cs296::settings_t settings;
  extern const int32 frame_period;
  extern float settings_hz;
  extern int32 width;
  extern int32 height;
  extern int32 main_window;
};

//! This opens up the cs296 namespace
//! What is the consequence of opening up a namespace?
using namespace cs296;


//! This function creates all the GLUI gui elements
void create_glui_ui(void)
{
  GLUI *glui = GLUI_Master.create_glui_subwindow( main_window, GLUI_SUBWINDOW_BOTTOM );
  
  glui->add_statictext("Simulation Timesteps"); 
  GLUI_Spinner* velocityIterationSpinner =
    glui->add_spinner("Velocity Iterations", GLUI_SPINNER_INT, &settings.velocity_iterations);
  velocityIterationSpinner->set_int_limits(1, 500);
  
  GLUI_Spinner* positionIterationSpinner =
    glui->add_spinner("Position Iterations", GLUI_SPINNER_INT, &settings.position_iterations);
  positionIterationSpinner->set_int_limits(0, 100);
  
  GLUI_Spinner* hertzSpinner =
    glui->add_spinner("Sim steps per frame", GLUI_SPINNER_FLOAT, &settings_hz);
  hertzSpinner->set_float_limits(5.0f, 200.0f);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Simulation Parameters"); 
  glui->add_checkbox("Warm Starting", &settings.enable_warm_starting);
  glui->add_checkbox("Time of Impact", &settings.enable_continuous);
  glui->add_checkbox("Sub-Stepping", &settings.enable_sub_stepping);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Display Options"); 
  GLUI_Panel* drawPanel =	glui->add_panel("Draw");
  glui->add_checkbox_to_panel(drawPanel, "Shapes", &settings.draw_shapes);
  glui->add_checkbox_to_panel(drawPanel, "Joints", &settings.draw_joints);
  glui->add_checkbox_to_panel(drawPanel, "AABBs", &settings.draw_AABBs);
  glui->add_checkbox_to_panel(drawPanel, "Statistics", &settings.draw_stats);
  glui->add_checkbox_to_panel(drawPanel, "Profile", &settings.draw_profile);
  
  new GLUI_Column( glui, false );
  glui->add_button("Pause", 0, callbacks_t::pause_cb);
  glui->add_button("Single Step", 0, callbacks_t::single_step_cb);
  glui->add_button("Restart", 0, callbacks_t::restart_cb);
  
  glui->add_button("Quit", 0,(GLUI_Update_CB)callbacks_t::exit_cb);
  glui->set_main_gfx_window( main_window );
}


//! This is the main function
int main(int argc, char** argv)
{
	 test_count = 1;
  test_index = 0;
  test_selection = test_index;
  
  entry = sim;
  test = entry->create_fcn();

  if(argc==1){
	
	  //! This initializes GLUT
	  glutInit(&argc, argv);
	  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	  glutInitWindowSize(width, height);

	  char title[56];
	  sprintf(title, "Cheetah Simulation by Group 17 . Running on Box2D %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	  main_window = glutCreateWindow(title);

	  //! Here we setup all the callbacks we need
	  //! Some are set via GLUI
	  GLUI_Master.set_glutReshapeFunc(callbacks_t::resize_cb);  
	  GLUI_Master.set_glutKeyboardFunc(callbacks_t::keyboard_cb);
	  GLUI_Master.set_glutSpecialFunc(callbacks_t::keyboard_special_cb);
	  GLUI_Master.set_glutMouseFunc(callbacks_t::mouse_cb);
	  //! Others are set directly
	  glutDisplayFunc(callbacks_t::display_cb);
	  glutMotionFunc(callbacks_t::mouse_motion_cb);
	  glutKeyboardUpFunc(callbacks_t::keyboard_up_cb); 
	  glutTimerFunc(frame_period, callbacks_t::timer_cb, 0);

	  //! We create the GLUI user interface
	  create_glui_ui();

	  //! Enter the infinite GLUT event loop
	  glutMainLoop();
	}
  else{
	int no_of_it=atoi(argv[1]);
	 const b2Profile& p = test->get_world()->GetProfile();
	  float32 tot_time=0,col_time=0,solve_velocity_time=0,solve_pos_time=0;
	  long  double sttime,endtime;
	  timeval tim; 
	  tim.tv_sec=0;
	  tim.tv_usec=0; 
	  settimeofday(&tim,NULL);
	  gettimeofday(&tim,NULL);
	  sttime=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);
	  for (int i = 0; i < no_of_it; i++)
	  {
		  test->step(&settings);
		  tot_time += p.step;
		  col_time+=p.collide;
		  solve_pos_time+=p.solvePosition;
		  solve_velocity_time+=p.solveVelocity;
		//  tot_time += p.step + p.collide + p.solve + p.solveInit + p.solveVelocity + p.solvePosition + p.solveTOI + p.broadphase;		
	  }
	  gettimeofday(&tim,NULL);
	  endtime=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);
	  //printf("%f\n", tot_time);
	  tot_time/=(float)no_of_it;		
	  col_time/=(float)no_of_it;		
	  solve_pos_time/=(float)no_of_it;		
	  solve_velocity_time/=(float)no_of_it;		
	   printf("Number of iterations : %d\n",no_of_it);
	  printf("Average time per step is %.5f ms\n",tot_time);
	  printf("Average time for collisions is %.5f ms\n",col_time);
	  printf("Average time per for velocity updates is %.5f ms\n",solve_velocity_time);
	  printf("Average time per for position updates is %.5f ms\n",solve_pos_time);
	  printf("\nTotal loop time is %Lf ms\n",(endtime-sttime));
	}
 

  return 0;
}
