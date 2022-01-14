// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "settings.h"
#include "test.h"

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

// A basic slider crank created for GDC tutorial: Understanding Constraints
class SliderCrankCL : public Test
{
public:
	SliderCrankCL()
	{
		dir_change_timer_ = steady_clock::now();

		m_world->SetGravity(b2Vec2_zero);

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 17.0f);
			ground = m_world->CreateBody(&bd);
		}

		{
			b2Body* prevBody = ground;

			// Define crank.
			{
				b2PolygonShape shape;
				const float crank_width = 4.0;
				shape.SetAsBox(crank_width * 2, 1.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-8.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&shape, 0.10f);
				body->SetFixedRotation(false);

				b2MassData md;
				body->GetMassData(&md);
				md.center.x -= crank_width;
				body->SetMassData(&md);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(-12.0f, 20.0f));

				//rjd.enableMotor = true;
				//rjd.motorSpeed = 3.141;
				//rjd.maxMotorTorque = 2000.0;
				m_motor = static_cast<b2RevoluteJoint*>(m_world->CreateJoint(&rjd));

				prevBody = body;
			}

			// Define connecting rod
			{
				b2PolygonShape shape;
				shape.SetAsBox(8.0f, 1.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;

				bd.position.Set(4.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 0.10f);


				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(-4.0f, 20.0f));
				m_world->CreateJoint(&rjd);

				prevBody = body;
			}

			// Define piston
			{
				// Unsere Masse
				b2PolygonShape shape;
				shape.SetAsBox(3.0f, 3.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.fixedRotation = true;
				bd.position.Set(12.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);
				m_mass = body->CreateFixture(&shape, 2.0f);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(12.0f, 20.0f));
				m_world->CreateJoint(&rjd);

				b2PrismaticJointDef pjd;
				pjd.Initialize(ground, body, b2Vec2(12.0f, 17.0f), b2Vec2(1.0f, 0.0f));
				m_world->CreateJoint(&pjd);
			}
		}
	}

	b2RevoluteJoint* m_motor{ nullptr };
	b2Fixture* m_mass{ nullptr };
	float torque_{ -1000.0 };
	float m_mass_x_offset{ 0.0 }; // könnte man nutzen, um die null in die mitte zu legen. der wert müsste sich irgendwie aus den beiden kurbeln ergeben

	steady_clock::time_point dir_change_timer_;

	void Step(Settings& settings) override
	{
		float position = m_mass->GetBody()->GetPosition().x;

		//std::cout << "position: " << position << "\n";

		if (steady_clock::now() - dir_change_timer_ > 10s) {
			std::cout << "Change Direction\n";
			//m_motor->SetMotorSpeed(-m_motor->GetMotorSpeed());
			std::cout << "Reaction Torque: " << m_motor->GetMotorTorque(settings.m_hertz) << "Nm\n";
			torque_ = -torque_;
			dir_change_timer_ = steady_clock::now();

			b2MassData md;
			m_mass->GetMassData(&md);
			auto velo = m_mass->GetBody()->GetLinearVelocity();
			std::cout << "Mass: " << md.mass << "kg\n" << "Velo: " << velo.x << "m/s\n";

		}

		m_motor->GetBodyB()->ApplyTorque(torque_, true);
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new SliderCrankCL;
	}
};

static int testIndex = RegisterTest("905", "Slider Crank Closed Loop", SliderCrankCL::Create);
