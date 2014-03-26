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
		  bd2.type = b2_dynamicBody;
		  tail1 = m_world->CreateBody(&bd2);
		  b2FixtureDef* fd2 = new b2FixtureDef;
		  fd2->density = 1.f;
		  fd2->shape = new b2PolygonShape;
		  fd2->shape = &hrec;
		  tail1->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail1->CreateFixture(fd2);
		 
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
		  bd2.position.Set(27.0f, 25.2f);
		  tail6 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail6->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail6->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail5,tail6, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject7
		  b2Body* tail7;
		  tail7 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
		  bd2.position.Set(27.0f, 25.2f);
		  tail7 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail7->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail7->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail6,tail7, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject8
		  b2Body* tail8;
		  tail8 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
		  bd2.position.Set(27.0f, 25.2f);
		  tail8 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail8->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail8->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail7,tail8, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject9
		  b2Body* tail9;
		  tail9 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
		  bd2.position.Set(27.0f, 25.2f);
		  tail9 = m_world->CreateBody(&bd2);
		  hrec.SetAsBox(0.80f, 1.6f);
		  vrec.SetAsBox(1.6f, 0.70f);
		  fd2->shape = &hrec;
		  tail9->CreateFixture(fd2);
		   fd2->shape = &vrec;
		  tail9->CreateFixture(fd2);
		  
		  anchor.Set(26.5f, 27.0f);
		  jd.Initialize(tail8,tail9, anchor);
		  m_world->CreateJoint(&jd);
		  
		  //Tailobject10
		  b2Body* tail10;
		  tail10 = m_world->CreateBody(&bd2);
		  bd2.angle=-90*DEGTORAD;
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
		  m_world->CreateJoint(&jd);
	  }
    
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
