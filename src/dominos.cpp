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

  dominos_t::dominos_t()
  {
	  
	  //GROUND
	  {
		  b2Body* ground;
		  b2EdgeShape shape;
		  shape.Set(b2Vec2(-90.0f, 5.0f), b2Vec2(90.0f, 5.0f));
		  b2BodyDef bd; 
		  ground = m_world->CreateBody(&bd); 
		  ground->CreateFixture(&shape, 0.0f);
	  }
	  //A part of tail
	  {
		  b2RevoluteJointDef jd;
		  b2Vec2 anchor;
		  
		  //Tail object1
		  b2Body* tail1;
		  b2PolygonShape hrec,vrec;
		  hrec.SetAsBox(1.0f, 2.0f);
		  vrec.SetAsBox(2.0f, 1.0f);
		  b2BodyDef bd2;
		  bd2.angle=-10*DEGTORAD;
		  bd2.position.Set(20.0f, 35.5f);
		  
		  tail1 = m_world->CreateBody(&bd2);
		  b2FixtureDef* fd2 = new b2FixtureDef;
		  fd2->density = 1.f;
		  fd2->shape = new b2PolygonShape;
		  fd2->shape = &hrec;
		  tail1->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail1->CreateFixture(fd2);
		 bd2.type = b2_dynamicBody;
		  //Tail object2
		  b2Body* tail2;
		  tail2 = m_world->CreateBody(&bd2);
		  bd2.angle=-35*DEGTORAD;
		  bd2.position.Set(23.7f, 34.5f);
		  tail2 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.9f, 1.9f);
		  vrec.SetAsBox(1.9f, 0.9f);
		  fd2->shape = &hrec;
		  tail2->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail2->CreateFixture(fd2);
		  
		  anchor.Set(22.0f, 35.0f);
		  jd.Initialize(tail1,tail2, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject3
		  b2Body* tail3;
		  tail3 = m_world->CreateBody(&bd2);
		  bd2.angle=-60*DEGTORAD;
		  bd2.position.Set(26.0f, 32.0f);
		  tail3 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.85f, 1.8f);
		  vrec.SetAsBox(1.8f, 0.85f);
		  fd2->shape = &hrec;
		  tail3->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail3->CreateFixture(fd2);
		  
		  anchor.Set(24.1f, 33.0f);
		  jd.Initialize(tail2,tail3, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject4
		  b2Body* tail4;
		  tail4 = m_world->CreateBody(&bd2);
		  bd2.angle=-80*DEGTORAD;
		  bd2.position.Set(27.0f, 28.3f);
		  tail4 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.7f);
		  vrec.SetAsBox(1.7f, 0.80f);
		  fd2->shape = &hrec;
		  tail4->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail4->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 30.0f);
		  jd.Initialize(tail3,tail4, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject5
		  b2Body* tail5;
		  tail5 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
		  bd2.position.Set(27.0f, 25.2f);
		  tail5 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail5->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail5->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail4,tail5, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject6
		  b2Body* tail6;
		  tail6 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
		  bd2.position.Set(27.0f, 22.2f);
		  tail6 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.70f, 1.4f);
		  vrec.SetAsBox(1.3f, 0.50f);
		  fd2->shape = &hrec;
		  tail6->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail6->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 24.5f);
		  jd.Initialize(tail5,tail6, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject7
		  b2Body* tail7;
		  tail7 = m_world->CreateBody(&bd2);
		  bd2.angle=-80*DEGTORAD;
		  bd2.position.Set(27.0f, 19.2f);
		  tail7 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.60f, 1.3f);
		  vrec.SetAsBox(1.2f, 0.50f);
		  fd2->shape = &hrec;
		  tail7->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail7->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 20.5f);
		  jd.Initialize(tail6,tail7, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject8
		  b2Body* tail8;
		  tail8 = m_world->CreateBody(&bd2);
		  bd2.angle=-55*DEGTORAD;
		  bd2.position.Set(28.0f, 17.2f);
		  tail8 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.50f, 1.2f);
		  vrec.SetAsBox(1.1f, 0.44f);
		  fd2->shape = &hrec;
		  tail8->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail8->CreateFixture(fd2);
		  
		  anchor.Set(27.3f, 19.0f);
		  jd.Initialize(tail7,tail8, anchor);
		  m_world->CreateJoint(&jd);
		 
		  //Tailobject9
		  b2Body* tail9;
		  tail9 = m_world->CreateBody(&bd2);
		  bd2.angle=-25*DEGTORAD;
		  bd2.position.Set(29.5f, 16.2f);
		  tail9 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.50f, 0.9f);
		  vrec.SetAsBox(1.6f, 0.30f);
		  fd2->shape = &hrec;
		  tail9->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail9->CreateFixture(fd2);
		  
		  anchor.Set(28.5f, 17.0f);
		  jd.Initialize(tail8,tail9, anchor);
		  m_world->CreateJoint(&jd);
	/*	  
		  //Tailobject10
		  b2Body* tail10;
		  tail10 = m_world->CreateBody(&bd2);
		  bd2.angle=10*DEGTORAD;
		  bd2.position.Set(27.0f, 25.2f);
		  tail10 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail10->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail10->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail9,tail10, anchor);
		  m_world->CreateJoint(&jd);*/
	  }
	  
	  /**
	   * The part below is made by aditya kumar akash
	   * This is testing . donot bother about it
	   */
	  /* b2Body* body1;
	   b2Body* body2;
	   {
		   b2BodyDef bd;
		   bd.type=b2_dynamicBody;
		   bd.position.Set(0.0f, 17.0f);
		   
		   b2PolygonShape dynamicBox;
		   dynamicBox.SetAsBox(1.0f, 1.0f);
		   
		   b2FixtureDef fd;
		   fd.shape=&dynamicBox;
		   fd.density=1.0;
		   
		   body2=m_world->CreateBody(&bd);
		   body2->CreateFixture(&fd);

		}
		// the second box
	   {
		   b2BodyDef bd;
		   bd.type=b2_dynamicBody;
		   bd.position.Set(0.0f, 27.0f);
		   
		   b2PolygonShape dynamicBox;
		   dynamicBox.SetAsBox(1.0f, 1.0f);
		   
		   b2FixtureDef fd;
		   fd.shape=&dynamicBox;
		   fd.density=0.2;
		   
		   body1=m_world->CreateBody(&bd);
		   body1->CreateFixture(&fd);

		}
		// two distance joints acting as springs
		{b2DistanceJointDef jointDef;
		const b2Vec2 a(0.4f, 17.0f);
		const b2Vec2 b(0.4f, 27.0f);
		jointDef.Initialize(body1, body2, b, a);
		//jointDef.bodyA=body1;
		//jointDef.bodyB=body2;
		
		
		//jointDef.localAnchorA = a;
		//jointDef.localAnchorB = b;
		
		jointDef.collideConnected = true;
		jointDef.frequencyHz = 0.5f;
		jointDef.dampingRatio =0.0f;
	   
		m_world->CreateJoint(&jointDef);}
		{
		b2DistanceJointDef jointDef;
		const b2Vec2 a(-0.4f, 17.0f);
		const b2Vec2 b(-0.4f, 27.0f);
		jointDef.Initialize(body1, body2, b, a);
		//jointDef.bodyA=body1;
		//jointDef.bodyB=body2;
		
		
		//jointDef.localAnchorA = a;
		//jointDef.localAnchorB = b;
		
		jointDef.collideConnected = true;
		jointDef.frequencyHz = 0.25f;
		jointDef.dampingRatio =0.0f;
	   
		m_world->CreateJoint(&jointDef);}
	  */
	  
	  {
		   b2BodyDef bd;
		   bd.type=b2_dynamicBody;
		   bd.position.Set(12.5f, 11.0f);
		   
		   b2PolygonShape dynamicBox;
		   dynamicBox.SetAsBox(0.5f, 0.5f);
		   
		   b2FixtureDef fd;
		   fd.shape=&dynamicBox;
		   fd.density=0.2;
		   
		   b2Body* body1=m_world->CreateBody(&bd);
		   body1->CreateFixture(&fd);

		}

		// creating a horizaontal platform to serve as temp cheetah body
		b2Body* cheetahBody;
		{
			b2BodyDef cheetahBodyDef;
			cheetahBodyDef.position.Set(-10.0f, 35.0f);
			
			b2PolygonShape cheetahBodyShape;
			cheetahBodyShape.SetAsBox(20.0f, 1.0f);
			
			b2FixtureDef cheetahBodyFixture;
			cheetahBodyFixture.shape=&cheetahBodyShape;
			
			cheetahBody=m_world->CreateBody(&cheetahBodyDef);
			cheetahBody->CreateFixture(&cheetahBodyFixture);
		}
		
		// creating one leg for the body
		{
			// first part of leg
			b2BodyDef legDef1;
			legDef1.position.Set(6.0f, 28.5f);
			legDef1.type=b2_dynamicBody;
			legDef1.angle = -0.1f * b2_pi; 
			
			b2PolygonShape legShape1;
			legShape1.SetAsBox(0.5, 7.0f);
			
			b2FixtureDef legFixture1;
			legFixture1.shape=&legShape1;
			legFixture1.density=1.0f;
			
			b2Body* backLeg1;
			backLeg1=m_world->CreateBody(&legDef1);
			backLeg1->CreateFixture(&legFixture1);
			
			// second bony part of leg
			b2BodyDef legDef2;
			legDef2.position.Set(8.5f, 17.0f);
			legDef2.type=b2_dynamicBody;
			legDef2.angle = 0.2f * b2_pi; 
			
			b2PolygonShape legShape2;
			legShape2.SetAsBox(0.5, 8.0f);
			
			b2FixtureDef legFixture2;
			legFixture2.shape=&legShape2;
			legFixture2.density=1.0f;
			
			b2Body* backLeg2;
			backLeg2=m_world->CreateBody(&legDef2);
			backLeg2->CreateFixture(&legFixture2);
			
			// theird bony part of leg
			b2BodyDef legDef3;
			legDef3.position.Set(11.0f, 9.5f);
			legDef3.type=b2_dynamicBody;
			legDef3.angle = -0.25f * b2_pi; 
			
			b2PolygonShape legShape3;
			legShape3.SetAsBox(0.5, 6.0f);
			
			b2FixtureDef legFixture3;
			legFixture3.shape=&legShape3;
			legFixture3.density=1.0f;
			legFixture3.friction=1.0f;
			
			b2Body* backLeg3;
			backLeg3=m_world->CreateBody(&legDef3);
			backLeg3->CreateFixture(&legFixture3);
			
			// now joining joint1
			b2RevoluteJointDef joint1;
			const b2Vec2 point1(8.0f, 35.0f);
			joint1.Initialize(cheetahBody, backLeg1, point1);
			m_world->CreateJoint(&joint1);
			
			// now joining joint2
			b2RevoluteJointDef joint2;
			const b2Vec2 point2(4.5f, 23.0f);
			joint2.Initialize(backLeg1, backLeg2, point2);
			m_world->CreateJoint(&joint2);
			
			// now joining joint3
			b2RevoluteJointDef joint3;
			const b2Vec2 point3(12.5f, 11.0f);
			joint3.Initialize(backLeg2, backLeg3, point3);
			m_world->CreateJoint(&joint3);
			
			
		}
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
