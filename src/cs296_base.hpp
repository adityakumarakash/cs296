/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

 //! What is the difference between a class and a struct in C++?
  /**
   * \brief
   * A class defines the properties of and object which contains the different features of objetcs which are of different datatype
   * The struct also does the same but the difference between them is class implements OBJECT ORIENTED PROGRAMMING Concepts of inheritance
   * Abstraction etc while this feature is not implemented by struct
   * By default the class members are private while the struct members are public
   * 
   * */
  class base_sim_t;
  struct settings_t;
  
  //! Why do we use a typedef
  /**
   * \brief
   * We use a type def to define a datatype as an alias to some other data type<br>
   * For Example : typedef roll int<br>
   * This defines a roll data type which is other name for int. It improves readability of the code
   * 
   * */
  typedef base_sim_t* sim_create_fcn(); 

  /*! Simulation settings. Some can be controlled in the GUI.
   * 
   * This class provides various settings for controlling the simulation happening.
   * parameters such as velocity iterations and position iterations which define the accuracy and efficiency of simulation
   * are defined in this class. The description of members provide further details of the class.
   * */
  struct settings_t
  {
     //! Notice the initialization of the class members in the constructor
    //! How is this happening?
    /*!
     * \brief This way of initialization is called 'member initialization list'. <br>
     * This way of initialization is in some sense better than the initialization by the parameter list of constructor.
     * There is a necessity associated with this kind of intialization when a class or a struct is member of some other class.
     * In these cases if the contained class has only one contructor that needs an argument to initialize the class , then the class
     * cannot be declared in the other class. But if the contructor uses the member initialization list then we can 
     * also use the contructor as default initializer also.
     * */
    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
   b2Vec2 view_center;			//!<The centre of the frame is specified by this
    float32 hz;					//!<The frequency used in the simulation
    int32 velocity_iterations;	//!<The number of times velocity is iterated is specified by this
    int32 position_iterations;	//!<The position contraint iterations
    int32 draw_shapes;			//!<To decide whether or not to draw the shape in the frame
    int32 draw_joints;			//!<To decide whether or not to draw the joints in the frame
    int32 draw_AABBs;			//!<To decide whether or not to draw the AABB's in the frame
    int32 draw_pairs;			//!<To decide whether or not to draw the pairs in the frame
    int32 draw_contact_points;	//!<To decide whether or not to draw the contact_points in the frame
    int32 draw_contact_normals;	//!<To decide whether or not to draw the normal in the frame
    int32 draw_contact_forces;	//!<To decide whether or not to draw the CONTACT FORCES in the frame
    int32 draw_friction_forces;	//!<To decide whether or not to draw the friction forces in the frame
    int32 draw_COMs;			//!<To decide whether or not to draw the COMS in the frame
    int32 draw_stats;			//!<To decide whether or not to draw the stats in the frame
    int32 draw_profile;			//!<To decide whether or not to draw the profile in the frame
    int32 enable_warm_starting;	//!<To decide whether or not to enable warm starting in the frame
    int32 enable_continuous;	//!<To decide whether or not to enable continuos in the frame
    int32 enable_sub_stepping;	//!<To decide whether or not to enable sub stepping in the frame
    int32 pause;				//!<To decide whether or not to enable pause in the frame
    int32 single_step;			//!<To decide whether or not to enable single step in the frame
  };
  
  //!Struct to initialize the simulation.
  struct sim_t
  {
    const char *name;
    sim_create_fcn *create_fcn;//!<A pointer to base_sim_t type.

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048;  
  
  /*!Defines point of contact between two colliding objects.
	 * 
	 * Contacts are objects created by Box2D to manage collision between two fixtures. 
	 * There are different kinds of contacts, derived from b2Contact, for managing contact between different kinds of fixtures. <br>
	 * A contact point is a point where two shapes touch. Box2D approximates contact with a small number of points.
   * */
  struct contact_point_t
  {
	  /*!This defines Fixture of first object colliding.
	 * */
    b2Fixture* fixtureA;
    /*!This defines Fixture of second object colliding.
	 * */
    b2Fixture* fixtureB; 
    b2Vec2 normal; //!<Stores the coordinates of the normal vector.
    b2Vec2 position; //!<Stores the coordinates of the position vector.
    b2PointState state; 	//!<Defines the state of the two colliding bodies
  };
  
   /*!One can receive contact data by implementing b2ContactListener.
   * 
   * The bas_sim_t class inherits from the b2ConatctListener class.
   * The contact listener supports several events: begin, end, pre-solve, and post-solve.
   * */
  class base_sim_t : public b2ContactListener
  {
  public:
  //! The bodies used in simulation is deffined here so that we can change the properties of them for the keypress events
    b2Body* cheetahBody;  //!< This defines the body of the cheetah
	b2RevoluteJointDef mainwheel; //!< This defines the rotating wheel of the body
	b2RevoluteJoint* m_motorJoint;
    int counter;
    base_sim_t();
    
 //Cheetah motion is declared here
	void jump_cheetah();//!< This function makes the cheetah jump
	void stop_cheetah(); //!< This function makes the cheetah stop at its position
	void left_cheetah(); //!< This function makes the cheetah move towards left
	void right_cheetah(); //!< This function makes the cheetah move toward right
	
	//!< This function RETURNS THE world
	 b2World* get_world(void){ return m_world;}
    
    //! Virtual destructors - amazing objects. Why are these necessary?
    
    /**
     * \brief
     * They are necessary to deallocate the memory which is previously dynamically allocated.
     * */
    virtual ~base_sim_t();
    
    //!< This function sets the text line variable
    void set_text_line(int32 line) { m_text_line = line; }
    //!< This function draws title of the simulation screen
    void draw_title(int x, int y, const char *string);
    
    //!< This is the step function which is called in each of the iteration and is responsible for the simulation
    virtual void step(settings_t* settings);

	//!< This function assigns the keyboard functions
    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    //! This functcion Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    //! Callbacks for derived classes.
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }

  //!How are protected members different from private memebers of a class in C++ ?
  /**
   * \brief
   * A private member can't be inherited but can be accesses only from the member function of the same class while a protected data member can be inherited to a different class but like private it is also accesible only from the member function of teh same class
   * */
  protected:

    //! What are Friend classes?
    /**
     * \brief
     * Friend Classes are those classes whose member functions and data memebers can be accesses from a class
     */
    friend class contact_listener_t;
    
     b2Body* m_ground_body;	//!<The ground is defined by this
    b2AABB m_world_AABB;	//!<the simulation world of AABB
    contact_point_t m_points[k_max_contact_points];	//!<stores the contact points
    int32 m_point_count;	//!<Max no. of contact points

    debug_draw_t m_debug_draw;	//!< Shapes in the world AABB are drawn by this
    int32 m_text_line;			
    b2World* m_world;			//!<This tells about the simulation world which is current

    int32 m_step_count;			//!<stores steps
    
    b2Profile m_max_profile;	//!<stores the maximum number of profiles
    b2Profile m_total_profile;	//!<stores the total number of profiles
  };
}

#endif
