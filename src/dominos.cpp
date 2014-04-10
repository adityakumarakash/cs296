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
#include <stdio.h>
#define DEGTORAD 0.0174532925199432957f
#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
	
	b2Vec2 m_offset; //! This is the offset variable which moves a body from particular distance from center
	b2Body* m_wheel; //! This is main wheel which is used on walking of cheetah
	float32 m_motorSpeed; //! Variable storing the motorspeed
	bool m_motorOn; //! Bool value to check is motor is On
	int val=0;
	float32 c = 2.5;
	
	/*! \brief Function to create leg
	 * 
	 * This function is used to create the leg of the cheetah with different orientaation
	 * */
	void dominos_t::CreateLeg(float32 s, const b2Vec2& wheelAnchor)
	{
		//! Cordinates controlling the orientation of leg
		b2Vec2 p1(c * 5.4f * s, -6.1f * c);
		b2Vec2 p2(c * 7.2f * s, -1.2f * c);
		b2Vec2 p3(c * 4.3f * s, -1.9f * c);
		b2Vec2 p4(c * 3.1f * s, 0.8f * c);
		b2Vec2 p5(c * 6.0f * s, 1.5f * c);
		b2Vec2 p6(c * 2.5f * s, 3.7f * c);

		b2FixtureDef fd1, fd2;
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0f;
		fd2.density = 1.0f;
 /*! \brief 
  *
  * name -poly1 <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Define a polygon shape which is used in cheetah<br>
  * 
  */
   /*! \brief 
  *
  * name -poly2 <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Defines a polygon which is used in cheetah<br>
  * 
  */

		b2PolygonShape poly1, poly2;

		if (s > 0.0f)
		{
			b2Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p2;
			vertices[2] = p3;
			poly1.Set(vertices, 3);

			vertices[0] = b2Vec2_zero;
			vertices[1] = p5 - p4;
			vertices[2] = p6 - p4;
			poly2.Set(vertices, 3);
		}
		else
		{
			b2Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p3;
			vertices[2] = p2;
			poly1.Set(vertices, 3);

			vertices[0] = b2Vec2_zero;
			vertices[1] = p6 - p4;
			vertices[2] = p5 - p4;
			poly2.Set(vertices, 3);
		}

		fd1.shape = &poly1;
		fd2.shape = &poly2;
/*!		
 * name -bd1 <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for ------ <br>
  */
  /*!		
 * name -bd2 <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for ------ <br>
  */
		b2BodyDef bd1, bd2;
		bd1.type = b2_dynamicBody;
		bd2.type = b2_dynamicBody;
		bd1.position = m_offset;
		bd2.position = p4 + m_offset;

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

		b2Body* body1 = m_world->CreateBody(&bd1);
		b2Body* body2 = m_world->CreateBody(&bd2);

		body1->CreateFixture(&fd1);
		body2->CreateFixture(&fd2);

  /*! \brief 
  * name -distancejoint <br>
  * datatype -b2DistanceJointDef <br>
  * operation -Defining an spring joint between two bodies to staility <br>
  * value - this joins body1 and body2<br>
  * 
  */
		b2DistanceJointDef distanceJoint;

		distanceJoint.dampingRatio = 0.7f;
		distanceJoint.frequencyHz = 8.0f;

		distanceJoint.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
		m_world->CreateJoint(&distanceJoint);

		distanceJoint.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
		m_world->CreateJoint(&distanceJoint);

		distanceJoint.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&distanceJoint);

		distanceJoint.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&distanceJoint);
		/*!
 * name -rjd <br>
  * datatype -b2RevoluteJointDef <br>
  * operation -Defining an anchor point where the bodies are joined. <br>
  * value - Joints the body2 and cheetahBody<br>
  */
		b2RevoluteJointDef rjd;

		rjd.Initialize(body2, cheetahBody, p4 + m_offset);
		m_world->CreateJoint(&rjd);
		
		float32 damping_const = 5.0f;
		
		if(s == -1)
		{
			// making the first leg part 1
			/*!
 * name -leg1 <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the leg1<br>
  */
  /*!
 * name -leg2 <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the leg2<br>
  */
  /*!		
 * name -bodydef <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for the parts of leg <br>
  */
		 /*! \brief 
  *
  * name -shape <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for the leg of the cheetah<br>
  * 
  */
  /*!
   * name -fixture <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for the leg of the cheetah<br>
  */
			b2Body* leg1;	
			b2Body* leg2;
			
			{
				b2BodyDef bodyDef;
				bodyDef.type = b2_dynamicBody;
				bodyDef.position.Set(m_offset.x + -14.0f, m_offset.y + -13.0f);
				bodyDef.angle = -0.38 * b2_pi;
				bodyDef.angularDamping = damping_const;
				
				b2PolygonShape shape;
				shape.SetAsBox(5.0f, 0.5f);
				
				b2FixtureDef fixture;
				fixture.density = 0.1f;
				fixture.friction = 1.0f;
				fixture.shape = &shape;
				fixture.filter.groupIndex = -1;
				
				leg1 = m_world->CreateBody(&bodyDef);
				leg1->CreateFixture(&fixture);
			}
			
			{
				b2BodyDef bodyDef;
				bodyDef.type = b2_dynamicBody;
				bodyDef.position.Set(m_offset.x + -13.0f, m_offset.y + -18.0f);
				bodyDef.angle = 0.40 * b2_pi;
				bodyDef.angularDamping = damping_const;
				
				b2PolygonShape shape;
				shape.SetAsBox(7.0f, 0.5f);
				
				b2FixtureDef fixture;
				fixture.density = 0.1f;
				fixture.friction = 1.0f;
				fixture.shape = &shape;
				fixture.filter.groupIndex = -1;
				
				leg2 = m_world->CreateBody(&bodyDef);
				leg2->CreateFixture(&fixture);
			}	
			
			{
				// joining the toes to the last body part 
				/*!
    * name -joint <br>
  * datatype - b2WeldJointDef <br>
  * operation -Defining a joint which joins two bodies rigidly <br>
  * value - Joints the leg1 and body1<br>
  */
				b2WeldJointDef joint;
				b2Vec2 vec;
				vec.Set(m_offset.x + -14.1f, m_offset.y + -13.0f);
				joint.Initialize(leg1 ,body1 , vec);
				m_world->CreateJoint(&joint);
			}
			
			{	
				// now joining joint1
				
				b2WeldJointDef joint;//!joins the two parts of a leg
				b2Vec2 vec(m_offset.x + -12.0f, m_offset.y + -17.5f);
				joint.Initialize(leg2, leg1, vec);
				m_world->CreateJoint(&joint);
			}
			
		}
		
			/*!
 * name -leg1 <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the leg1<br>
  */
  /*!
 * name -leg2 <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the leg2<br>
  */
  /*!		
 * name -bodydef <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for the parts of leg <br>
  */
		 /*! \brief 
  *
  * name -shape <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for the leg of the cheetah<br>
  * 
  */
  /*!
   * name -fixture <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for the leg of the cheetah<br>
  */
		else 		// back legs
		{
			
			
			// making the first leg part 1
			b2Body* leg1;	
			b2Body* leg2;
			
			{
				b2BodyDef bodyDef;
				bodyDef.type = b2_dynamicBody;
				bodyDef.position.Set(m_offset.x + 13.7f, m_offset.y + -14.0f);
				bodyDef.angle = -0.42 * b2_pi;
				bodyDef.angularDamping = damping_const;
				
				b2PolygonShape shape;
				shape.SetAsBox(5.0f, 0.5f);
				
				b2FixtureDef fixture;
				fixture.density = 0.1f;
				fixture.friction = 1.0f;
				fixture.shape = &shape;
				fixture.filter.groupIndex = -1;
				
				leg1 = m_world->CreateBody(&bodyDef);
				leg1->CreateFixture(&fixture);
			}	
			
			{
				b2BodyDef bodyDef;
				bodyDef.type = b2_dynamicBody;
				bodyDef.position.Set(m_offset.x + 15.0f, m_offset.y + -19.5f);
				bodyDef.angle = 0.46 * b2_pi;
				bodyDef.angularDamping = damping_const;
				
				b2PolygonShape shape;
				shape.SetAsBox(5.0f, 0.5f);
				
				b2FixtureDef fixture;
				fixture.density = 0.1f;
				fixture.friction = 1.0f;
				fixture.shape = &shape;
				fixture.filter.groupIndex = -1;
				
				leg2 = m_world->CreateBody(&bodyDef);
				leg2->CreateFixture(&fixture);
			}
				/*!
    * name -joint <br>
  * datatype - b2WeldJointDef <br>
  * operation -Defining a joint which joins two bodies rigidly <br>
  * value - Joints the leg1 and body1<br>
  */		
			{
				// joining the toes to the last bony part 
				b2WeldJointDef joint;
				b2Vec2 vec;
				vec.Set(m_offset.x + 13.7f, m_offset.y + -15.0f);
				joint.Initialize(leg1 ,body1 , vec);
				m_world->CreateJoint(&joint);
			}
			
			{	
				// now joining joint1
				b2WeldJointDef joint;//!joins the two parts of the leg
				b2Vec2 vec(m_offset.x + 15.0f, m_offset.y + -20.5f);
				joint.Initialize(leg2, leg1, vec);
				m_world->CreateJoint(&joint);
			}
			
		}
	}
	
	

  dominos_t::dominos_t()
  {
	  
	  //GROUND
	  /*!
 * name -ground <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the ground<br>
  */
  /*!
   * name -shape <br>
  * datatype -b2EdgeShape <br>
  * operation -Defines an edge between two given vectors. <br>
  * value - Contains the shape for ground<br>
  */
  /*!		
 * name -bd <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for ground <br>
  */
	  {
		  b2Body* ground;
		  b2EdgeShape shape;
		  shape.Set(b2Vec2(-900.0f, 5.0f), b2Vec2(900.0f, 5.0f));
		  b2BodyDef bd; 
		  ground = m_world->CreateBody(&bd); 
		  ground->CreateFixture(&shape, 0.0f);
	  }
	  

		m_offset.Set(0.0f, 31.0f);
		m_motorSpeed = -5.0;
		m_motorOn = true;
		b2Vec2 pivot(0.0f, 0.8f);

		// The main body part named cheetahbody
		  /*! \brief 
  *
  * name -shape <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for cheetahbody(rectangle which support the whole body)<br>
  * 
  */
   /*!
   * name -sd <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for cheetah body<br>
  */
  /*!		
 * name -bd <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for the cheetah body <br>
  */

		{
			b2PolygonShape shape;
			shape.SetAsBox(20.0f, 2.0f);

			b2FixtureDef sd;
		    sd.restitution = 0.1f;
			sd.density =  5.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			
			bd.type = b2_dynamicBody;
			bd.position = pivot + m_offset;
			cheetahBody = m_world->CreateBody(&bd);
			cheetahBody->CreateFixture(&sd);
		}
/*!
  * name -shape <br>
  * datatype -b2CircleShape <br>
  * operation -Defines an circle shape with a radius <br>
  * value - A cirlce if radius c*1.6<br>
  *
  */
   /*!
   * name -sd <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for cirlce<br>
  */ 
		{
			b2CircleShape shape;
			shape.m_radius = c * 1.6f;

			b2FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = pivot + m_offset;
			m_wheel = m_world->CreateBody(&bd);
			m_wheel->CreateFixture(&sd);
		}

		{			
			mainwheel.Initialize(m_wheel, cheetahBody, pivot + m_offset);
			mainwheel.collideConnected = false;
			mainwheel.motorSpeed = m_motorSpeed;
			mainwheel.maxMotorTorque = 5000.0f;
			mainwheel.enableMotor = m_motorOn;
			m_motorJoint = (b2RevoluteJoint*)m_world->CreateJoint(&mainwheel);
		}

		b2Vec2 wheelAnchor;//! THis defines a anchor for the wheel 
		
		
		wheelAnchor = pivot + b2Vec2(0.0f, -1.2f);
		val=1;
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg( 1.0f, wheelAnchor);
		val=2;
		m_wheel->SetTransform(m_wheel->GetPosition(), 120.0f * b2_pi / 180.0f);
		CreateLeg( -1.0f, wheelAnchor);
		CreateLeg( 1.0f, wheelAnchor);
		
	/*!	
		   * name -jd <br>
  * datatype -b2RevoluteJointDef <br>
  * operation -Defining an anchor point where the bodies are joined. <br>
  * value - Joints two parts <br>
  */
		b2RevoluteJointDef jd;
		
		  b2Vec2 anchor;//!works as an anchor two join two parts
		  //HEAD
/*!
 * name -head <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the head of the cheetah<br>
  */
    /*! \brief 
  *
  * name -poly <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for the head of the cheetah<br>
  * 
  */
  /*!		
 * name -bd <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for the head of the cheetah <br>
  */
  /*!
   * name -fd <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for the head<br>
  */
 
			b2Vec2 headref(m_offset.x + 5.4f, m_offset.y  + -32.0f);
		  b2Body* head;
		  {
			  b2PolygonShape poly;
			  b2Vec2 vertices[8];
			  vertices[0].Set(headref.x + 0, headref.y + 0);
			  vertices[1].Set(headref.x + 9, headref.y + 3);
			  vertices[2].Set(headref.x + 10, headref.y + -2);
			  vertices[3].Set(headref.x + 4, headref.y + -5.5);
			  vertices[4].Set(headref.x + -3, headref.y + -5);
			  vertices[5].Set(headref.x + -3.5, headref.y + -4);
			  vertices[6].Set(headref.x + -3, headref.y + -3);
			  vertices[7].Set(headref.x + 0, headref.y + -2);
			  poly.Set(vertices, 8);
			  b2BodyDef bd;
			  bd.position.Set(headref.x + -44.0f, headref.y + 34.0f);
			  bd.type = b2_dynamicBody;
			  head = m_world->CreateBody(&bd);
			  b2FixtureDef* fd= new b2FixtureDef;
			  fd->shape = new b2PolygonShape;
			  fd->shape = &poly;
			  fd->density = 0.00001f;
			  fd->filter.groupIndex = 1;
			  head->CreateFixture(fd);
		  
		  }
		 
		 
		  //MAIN B0DY
		  /*!
 * name -mainbody <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the main body<br>
  */
   /*! \brief 
  *
  * name -poly <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for main body of the cheetah<br>
  * 
  */
  /*!		
 * name -bd <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for main body of the cheetah <br>
  */
   /*!
   * name -fd <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for main body of the cheetah<br>
  */
  /*!
    * name -jd1 <br>
  * datatype - b2WeldJointDef <br>
  * operation -Defining a joint which joins two bodies rigidly <br>
  * value - Joints the cheetahbody and mainbody<br>
  */
		  b2Vec2 bodyref(m_offset.x + 6.0f, m_offset.y  + -32.0f);
		  
		  
		  b2Body* mainbody;
		  {
				b2PolygonShape poly;
				b2Vec2 vertices[8];
				vertices[0].Set(bodyref.x + 0, bodyref.y + 0);
				vertices[1].Set(bodyref.x + 10, bodyref.y + 3);
				vertices[2].Set(bodyref.x + 28, bodyref.y + 0.5);
				vertices[3].Set(bodyref.x + 28, bodyref.y + -1.5);
				vertices[4].Set(bodyref.x + 14.5, bodyref.y + -10);
				vertices[5].Set(bodyref.x + 10, bodyref.y + -10.2);
				vertices[6].Set(bodyref.x + 4, bodyref.y + -8);
				vertices[7].Set(bodyref.x + 0.5, bodyref.y + -3);
				poly.Set(vertices, 8);
				b2BodyDef bd;
				bd.position.Set(bodyref.x + -33.0f, bodyref.y + 36.0f);
				bd.type = b2_dynamicBody;
				mainbody = m_world->CreateBody(&bd);
				b2FixtureDef* fd= new b2FixtureDef;
				fd->shape = new b2PolygonShape;
				fd->shape = &poly;
				fd->density = 0.0f;
				fd->filter.groupIndex = -1;
				mainbody->CreateFixture(fd);

				b2WeldJointDef jd1;

				anchor.Set(bodyref.x + -20.0f, bodyref.y + 30.0f);
				jd1.Initialize(cheetahBody, mainbody, anchor);
				m_world->CreateJoint(&jd1);
		  
		  }
		  
		  
		//NECK
/*!
 * name -neck[3] <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains array of pointers to the neck part of the cheetah<br>
  */
  /*! \brief 
  *
  * name -hrev <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for horizontal box of the pieces of neck<br>
  * 
  */
  /*! \brief 
  *
  * name -vrev <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for vertical part of the pieces of neck<br>
  * 
  */
  /*!		
 * name -bd2 <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for parts of the neck of the cheetah <br>
  */
  /*!
   * name -fd2 <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for the parts of the neck of the cheetah<br>
  */
  /*!
    * name -jd1 <br>
  * datatype - b2WeldJointDef <br>
  * operation -Defining a joint which joins two bodies rigidly <br>
  * value - Joints the neck[i] and neck[i-1] for i=1,2<br>
  */
		b2Vec2 neckref(m_offset.x + 11.3f, m_offset.y  + -33.5f);
		b2Body* neck[3];
		{
			jd.enableLimit = true;
			jd.lowerAngle = -0.f * b2_pi;
			jd.upperAngle = 0.0f * b2_pi; 
			for (int i = 0; i < 3; i++)
			{				
				b2PolygonShape hrec,vrec;
				hrec.SetAsBox(0.2f, 2.0f);
				vrec.SetAsBox(0.4f, 1.5f);
				b2BodyDef bd2;
				bd2.position.Set(neckref.x + -32.5f-0.9*i, neckref.y + 34.5f);
				bd2.type = b2_dynamicBody;
				bd2.angle=10*DEGTORAD;
				neck[i] = m_world->CreateBody(&bd2);
				b2FixtureDef* fd2 = new b2FixtureDef;
				fd2->density = 0.0001f;
				fd2->filter.groupIndex = 1;
				fd2->shape = new b2PolygonShape;
				fd2->shape = &hrec;
				neck[i]->CreateFixture(fd2);
				fd2->shape = &vrec;
				neck[i]->CreateFixture(fd2);
				if(i>0)
				{
					b2WeldJointDef jd1;
					anchor.Set(neckref.x + -32.0f-0.9*i, neckref.y + 34.5f);
					jd1.Initialize(neck[i-1], neck[i], anchor);
					m_world->CreateJoint(&jd1);
				}
			}
			
			b2WeldJointDef jd1;
			anchor.Set(neckref.x + -28.0f-0.9*3, neckref.y + 34.5f);
			jd1.Initialize(neck[2],head, anchor);
			m_world->CreateJoint(&jd1);
			
			anchor.Set(neckref.x + -28.0f-0.9*0, neckref.y + 34.5f);
			jd1.Initialize(neck[0],cheetahBody, anchor);
			m_world->CreateJoint(&jd1);
		}	
		
	
	  
	//Hip
/*!
 * name -hip <br>
  * datatype -b2Body* <br>
  * operation -Pointer to b2Body object. <br>
  * value - This contains pointer to the hip part of the cheetah<br>
  */
  /*!  
  *
  * name poly <br>
  * datatype -b2PolygonShape <br>
  * operation -To define a convex Polygon. <br>
  * value - Shape for hip part of the cheetah<br>
  * 
  */
  /*!
  * name -hipfd <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for hip part of the cheetah<br>
  */ 
  /*!		
 * name -hipbd <br>
  * datatype -b2BodyDef <br>
  * operation -Holds all the data needed to construct a rigid body. <br>
  * value -Contains all data for hip part of the cheetah <br>
  */
	b2Vec2 hipref(m_offset.x + 0.5f, m_offset.y  + -32.0f);
	  b2Body* hip;
	  {
		b2PolygonShape poly;
		b2Vec2 vertices[8];
		vertices[0].Set(hipref.x + 0,hipref.y + 0);
		vertices[1].Set(hipref.x + 10, hipref.y + 1);
		vertices[2].Set(hipref.x + 13, hipref.y + -3);
		vertices[3].Set(hipref.x + 11, hipref.y + -6);
		vertices[4].Set(hipref.x + 4.4, hipref.y + -5);
		vertices[5].Set(hipref.x + 3.6, hipref.y + -3.6);
		vertices[6].Set(hipref.x + 1.4, hipref.y + -2.6);
		vertices[7].Set(hipref.x + -1, hipref.y + -4);
		poly.Set(vertices, 8);
		b2FixtureDef hipfd;
		hipfd.shape = &poly;
		hipfd.density = 0.0f;
		hipfd.friction = 1.0f;
		hipfd.restitution = 0.8f;
		hipfd.filter.groupIndex = -1;
		  
/*!
    * name -jd1 <br>
  * datatype - b2WeldJointDef <br>
  * operation -Defining a joint which joins two bodies rigidly <br>
  * value - Joints the cheetahBody and hip part of the cheetah<br>
  */ 
		b2BodyDef hipbd;
		hipbd.type = b2_dynamicBody;
		hipbd.position.Set(hipref.x + 6.0f, hipref.y + 37.0f);
		hip = m_world->CreateBody(&hipbd);
		hip->CreateFixture(&hipfd);
		b2WeldJointDef jd1;

		anchor.Set(hipref.x + 6.0f, hipref.y + 35.5f);
		jd1.Initialize(cheetahBody, hip, anchor);
		m_world->CreateJoint(&jd1);
	  
	  } 
	  
	  
	b2Vec2 ref(m_offset.x + 3.0f, m_offset.y  + -33.0f);
	float32 tail_density = 0.0000001f;
	anchor.Set(ref.x + 6.0f, ref.y + 35.5f);
		 //TAIL
	{
		//Tail object1
/*!
 * name -tail1 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object1<br>
 */
  
/*! 
  *
  * name -hrec <br>
  * datatype -b2PolygonShape <br>
  * operation -To define the horizontal part of the 1st piece of the tail. <br>
  * value - This contains shape for horizontal box which is in a shape of rectangle<br>
  */
/*! 
  *
  * name -vrec <br>
  * datatype -b2PolygonShape <br>
  * operation -To define the vertical part of the 1st piece of the tail. <br>
  * value - This contains shape for vertical box which is in a shape of rectangle<br>
  */
/*!	
 * name -bd2 <br>
 * datatype -b2BodyDef <br>
 * operation -Holds all the data needed to construct a rigid body. <br>
 * value -Contains all data for tail object1 <br>
 */
 /*!
   * name -fd2 <br>
  * datatype -b2FixtureDef <br>
  * operation -Used to attach a shape to a body for collision detection <br>
  * value - Defines a fixture for tail object1<br>
  */
		b2Body* tail1;
		b2PolygonShape hrec,vrec;
		hrec.SetAsBox(1.0f, 2.0f);
		vrec.SetAsBox(2.0f, 1.0f);

		b2BodyDef bd2;
		bd2.angle=10*DEGTORAD;
		bd2.position.Set(ref.x + 19.0f, ref.y + 35.5f);
		bd2.type = b2_dynamicBody;
		bd2.type = b2_dynamicBody;

		tail1 = m_world->CreateBody(&bd2);
		b2FixtureDef* fd2 = new b2FixtureDef;
		fd2->density = tail_density;
		fd2->shape = new b2PolygonShape;
		fd2->shape = &hrec;
		fd2->filter.groupIndex = -2;
		tail1->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail1->CreateFixture(fd2);

		jd.enableLimit = true;
		jd.lowerAngle = -0.1f * b2_pi;
		jd.upperAngle = 0.2f * b2_pi; 
		anchor.Set(ref.x + 18.0f, ref.y + 35.0f);
		jd.Initialize(tail1, cheetahBody, anchor);
		m_world->CreateJoint(&jd);

/*!
 * name -tail2 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object2<br>
 */
		//Tail object2
		b2Body* tail2;
		tail2 = m_world->CreateBody(&bd2);
		bd2.angle=-35*DEGTORAD;
		bd2.position.Set(ref.x + 22.7f, ref.y + 34.5f);
		tail2 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.9f, 1.9f);
		vrec.SetAsBox(1.9f, 0.9f);
		fd2->shape = &hrec;
		tail2->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail2->CreateFixture(fd2);

		anchor.Set(ref.x + 21.0f, ref.y + 35.0f);
		jd.Initialize(tail1,tail2, anchor);
		m_world->CreateJoint(&jd);

  /*! \brief 
  * name -joint4 <br>
  * datatype -b2DistanceJointDef <br>
  * operation -Defining an spring joint between two bodies to staility <br>
  * value - this joins tail1 and tail2<br>
  * 
  */
		// making a distance joint
		b2DistanceJointDef joint4;
		const b2Vec2 pt1(ref.x + 19.0f, ref.y + 37.6f);
		const b2Vec2 pt2(ref.x + 23.2f, ref.y + 36.5f);
		joint4.Initialize(tail1, tail2, pt1, pt2);
		joint4.collideConnected = true;
		joint4.frequencyHz = 1.0f;
		joint4.dampingRatio =0.8f;
		m_world->CreateJoint(&joint4);
		
/*!
 * name -tail3 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object3<br>
 */

		//Tailobject3
		b2Body* tail3;
		tail3 = m_world->CreateBody(&bd2);
		bd2.angle=-60*DEGTORAD;
		bd2.position.Set(ref.x + 25.0f, ref.y + 32.0f);
		tail3 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.85f, 1.8f);
		vrec.SetAsBox(1.8f, 0.85f);
		fd2->shape = &hrec;
		tail3->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail3->CreateFixture(fd2);

		anchor.Set(ref.x + 23.1f, ref.y + 33.0f);
		jd.Initialize(tail2,tail3, anchor);
		m_world->CreateJoint(&jd);
		
/*!
 * name -tail4 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object4<br>
 */

		//Tailobject4
		b2Body* tail4;
		tail4 = m_world->CreateBody(&bd2);
		bd2.angle=-80*DEGTORAD;
		bd2.position.Set(ref.x + 26.0f, ref.y + 28.3f);
		tail4 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.80f, 1.7f);
		vrec.SetAsBox(1.7f, 0.80f);
		fd2->shape = &hrec;
		tail4->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail4->CreateFixture(fd2);

		anchor.Set(ref.x + 25.5f, ref.y + 30.0f);
		jd.Initialize(tail3,tail4, anchor);
		m_world->CreateJoint(&jd);
		
/*!
 * name -tail5 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object5<br>
 */

		//Tailobject5
		b2Body* tail5;
		tail5 = m_world->CreateBody(&bd2);
		bd2.angle=-90*DEGTORAD;
		bd2.position.Set(ref.x + 26.0f, ref.y + 25.2f);
		tail5 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.80f, 1.6f);
		vrec.SetAsBox(1.6f, 0.70f);
		fd2->shape = &hrec;
		tail5->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail5->CreateFixture(fd2);

		anchor.Set(ref.x + 25.5f, ref.y + 27.0f);
		jd.Initialize(tail4,tail5, anchor);
		m_world->CreateJoint(&jd);

/*!
 * name -tail6 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object6<br>
 */
		//Tailobject6
		b2Body* tail6;
		tail6 = m_world->CreateBody(&bd2);
		bd2.angle=-90*DEGTORAD;
		bd2.position.Set(ref.x + 26.0f, ref.y + 22.2f);
		tail6 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.70f, 1.4f);
		vrec.SetAsBox(1.3f, 0.50f);
		fd2->shape = &hrec;
		tail6->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail6->CreateFixture(fd2);

		anchor.Set(ref.x  + 25.5f, ref.y + 24.5f);
		jd.Initialize(tail5,tail6, anchor);
		m_world->CreateJoint(&jd);

/*!
 * name -tail7 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object7<br>
 */
		//Tailobject7
		b2Body* tail7;
		tail7 = m_world->CreateBody(&bd2);
		bd2.angle=-80*DEGTORAD;
		bd2.position.Set(ref.x + 26.0f, ref.y + 19.2f);
		tail7 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.60f, 1.3f);
		vrec.SetAsBox(1.2f, 0.50f);
		fd2->shape = &hrec;
		tail7->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail7->CreateFixture(fd2);

		anchor.Set(ref.x + 25.5f, ref.y + 20.5f);
		jd.Initialize(tail6,tail7, anchor);
		m_world->CreateJoint(&jd);

/*!
 * name -tail8 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object8<br>
 */
		//Tailobject8
		b2Body* tail8;
		tail8 = m_world->CreateBody(&bd2);
		bd2.angle=-55*DEGTORAD;
		bd2.position.Set(ref.x + 27.0f, ref.y + 17.2f);
		tail8 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.50f, 1.2f);
		vrec.SetAsBox(1.1f, 0.44f);
		fd2->shape = &hrec;
		tail8->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail8->CreateFixture(fd2);

		anchor.Set(ref.x + 26.3f, ref.y + 19.0f);
		jd.Initialize(tail7,tail8, anchor);
		m_world->CreateJoint(&jd);
		
/*!
 * name -tail9 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object9<br>
 */

		//Tailobject9
		b2Body* tail9;
		tail9 = m_world->CreateBody(&bd2);
		bd2.angle=-25*DEGTORAD;
		bd2.position.Set(ref.x + 28.5f, ref.y + 16.2f);
		tail9 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.50f, 0.9f);
		vrec.SetAsBox(1.6f, 0.30f);
		fd2->shape = &hrec;
		tail9->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail9->CreateFixture(fd2);

		anchor.Set(ref.x + 27.5f, ref.y + 17.0f);
		jd.Initialize(tail8,tail9, anchor);
		m_world->CreateJoint(&jd);

/*!
 * name -tail10 <br>
 * datatype -b2Body* <br>
 * operation -Pointer to b2Body object. <br>
 * value - This contains pointer to the tail object10<br>
 */
		//Tailobject10
		b2Body* tail10;
		tail10 = m_world->CreateBody(&bd2);
		bd2.angle=15*DEGTORAD;
		bd2.position.Set(ref.x + 31.0f, ref.y + 16.1f);
		tail10 = m_world->CreateBody(&bd2);
		hrec.SetAsBox(0.80f, 0.3f);
		vrec.SetAsBox(1.3f, 0.20f);
		fd2->shape = &hrec;
		tail10->CreateFixture(fd2);
		fd2->shape = &vrec;
		tail10->CreateFixture(fd2);

		anchor.Set(ref.x + 29.5f, ref.y + 16.0f);
		jd.Initialize(tail9,tail10, anchor);
		m_world->CreateJoint(&jd);
	  }
		
		
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
