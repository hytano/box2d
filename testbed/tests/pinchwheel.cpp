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
#include "imgui/imgui.h"

#include <chrono>
#include <iostream>
using namespace std;
using namespace std::chrono;

// Test the wheel joint with motor, spring, and limit options.
class PinchWheel : public Test
{
public:
	PinchWheel()
	{
		b2Vec2 no_gravity(0.0, 0.0);
		m_world->SetGravity(no_gravity);

		const float STEEL = 8000.0;

		b2Body* rail = nullptr;
		{
			b2BodyDef rail_body;
			rail_body.position.Set(10.0, 0.0);

			rail = m_world->CreateBody(&rail_body);


			b2PolygonShape rail_shape;
			rail_shape.SetAsBox(rail_lenght_, rail_thickness_ / 2);

			b2FixtureDef rail_fix;

			rail_fix.filter.categoryBits = pinch_drive_col_grp;
			rail_fix.filter.maskBits = pinch_drive_col_grp;

			rail_fix.shape = &rail_shape;
			rail_fix.density = STEEL;

			rail->CreateFixture(&rail_fix);
		}

		b2Body* chassis = nullptr;
		{
			b2BodyDef cbd;

			cbd.position.Set(0.0, 0.0);
			cbd.type = b2_dynamicBody;

			chassis = m_world->CreateBody(&cbd);

			b2PolygonShape cs;
			cs.SetAsBox(chassis_half_length, chassis_half_width);

			b2FixtureDef cfd;
			cfd.shape = &cs;
			cfd.density = STEEL;

			cfd.filter.categoryBits = chassis_col_grp;
			cfd.filter.maskBits = chassis_col_grp;

			chassis->CreateFixture(&cs, STEEL);

			// This will fuck up the joint calculation
		//	b2MassData cmd;
			//cmd.mass = 20.0 * 1000.0;
		//	chassis->SetMassData(&cmd);
		}

		b2Body* fl_sus_rod = nullptr;
		{
			b2BodyDef cbd;

			cbd.position.Set(chassis_half_length + fl_sus_rod_length/2 -0.01, 0.0);
			cbd.type = b2_dynamicBody;

			fl_sus_rod = m_world->CreateBody(&cbd);

			b2PolygonShape cs;
			cs.SetAsBox(fl_sus_rod_length / 2, fl_sus_rod_width / 2);

			b2FixtureDef cfd;
			cfd.shape = &cs;
			cfd.density = STEEL;

			cfd.filter.categoryBits = chassis_col_grp;
			cfd.filter.maskBits = chassis_col_grp;

			fl_sus_rod->CreateFixture(&cs, STEEL);

			// Connect it to the chassis
			b2RevoluteJointDef rjd;
			rjd.Initialize(fl_sus_rod, chassis, b2Vec2(chassis_half_length - 0.005, 0.0));
			rjd.enableLimit = true;
			rjd.upperAngle = 0.93 * b2_pi;
			rjd.lowerAngle = -0.01 * b2_pi;
			m_world->CreateJoint(&rjd);
		}

		b2Body* fr_sus_rod = nullptr;
		{
			b2BodyDef cbd;

			cbd.position.Set(chassis_half_length + fr_sus_rod_length / 2 - 0.01, 0.0);
			cbd.type = b2_dynamicBody;

			fr_sus_rod = m_world->CreateBody(&cbd);

			b2PolygonShape cs;
			cs.SetAsBox(fr_sus_rod_length / 2, fr_sus_rod_width / 2);

			b2FixtureDef cfd;
			cfd.shape = &cs;
			cfd.density = STEEL;

			cfd.filter.categoryBits = chassis_col_grp;
			cfd.filter.maskBits = chassis_col_grp;

			fr_sus_rod->CreateFixture(&cs, STEEL);

			// Connect it to the chassis
			b2RevoluteJointDef rjd;
			rjd.Initialize(fr_sus_rod, chassis, b2Vec2(chassis_half_length - 0.005, 0.0));
			rjd.enableLimit = true;
			rjd.upperAngle = 0.01 * b2_pi;
			rjd.lowerAngle = -0.93 * b2_pi;
			m_world->CreateJoint(&rjd);
		}

		b2Body* wheel1 = nullptr;
		{
			b2BodyDef wheel_body_def;
			wheel_body_def.type = b2_dynamicBody;
			wheel_body_def.position.Set(chassis_half_length - 0.01 + fl_sus_rod_length - 0.01, 0.0001);

			b2CircleShape wheel_shape;
			wheel_shape.m_radius = pinch_wheel_radius_;

			wheel1 = m_world->CreateBody(&wheel_body_def);

			b2FixtureDef wheel_fixture_def;
			wheel_fixture_def.shape = &wheel_shape;
			wheel_fixture_def.density = pinch_wheel_pu_density_;
			wheel_fixture_def.friction = 1.0f;
			wheel_fixture_def.filter.categoryBits = pinch_drive_col_grp;
			wheel_fixture_def.filter.maskBits = pinch_drive_col_grp;

			wheel1->CreateFixture(&wheel_fixture_def);

			// Attach to fl rod
			b2RevoluteJointDef rjd;
			rjd.Initialize(wheel1, fl_sus_rod, wheel1->GetWorldCenter());
			m_world->CreateJoint(&rjd);
		}

		b2Body* wheel2 = nullptr;
		{
			b2BodyDef wheel_body_def;
			wheel_body_def.type = b2_dynamicBody;
			wheel_body_def.position.Set(chassis_half_length - 0.01 + fr_sus_rod_length - 0.01, -0.0001);

			b2CircleShape wheel_shape;
			wheel_shape.m_radius = pinch_wheel_radius_;

			wheel2 = m_world->CreateBody(&wheel_body_def);

			b2FixtureDef wheel_fixture_def;
			wheel_fixture_def.shape = &wheel_shape;
			wheel_fixture_def.density = pinch_wheel_pu_density_;
			wheel_fixture_def.friction = 1.0f;
			wheel_fixture_def.filter.categoryBits = pinch_drive_col_grp;
			wheel_fixture_def.filter.maskBits = pinch_drive_col_grp;

			wheel2->CreateFixture(&wheel_fixture_def);

			// Attach to fl rod
			b2RevoluteJointDef rjd;
			rjd.Initialize(wheel2, fr_sus_rod, wheel2->GetWorldCenter());
			m_world->CreateJoint(&rjd);
		}

		b2DistanceJoint* spring_joint = nullptr;
		{
			b2DistanceJointDef sjd;
			sjd.bodyA = wheel1;
			sjd.bodyB = wheel2;

			sjd.length = rail_thickness_ -0.11;
			float frequencyHz = 30.0f;
			float dampingRatio = 10000.0f;
			b2LinearStiffness(sjd.stiffness, sjd.damping, frequencyHz, dampingRatio, sjd.bodyA, sjd.bodyB);
			spring_joint = (b2DistanceJoint*)m_world->CreateJoint(&sjd);
		}

		
		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 10.0f;

		//{
		//	b2CircleShape shape;
		//	shape.m_radius = 2.0f;

		//	b2BodyDef bd;
		//	bd.type = b2_dynamicBody;
		//	bd.position.Set(0.0f, 10.0f);
		//	bd.allowSleep = false;
		//	b2Body* body = m_world->CreateBody(&bd);
		//	body->CreateFixture(&shape, 5.0f);

		//	b2WheelJointDef jd;

		//	// Horizontal
		//	jd.Initialize(ground, body, bd.position, b2Vec2(0.0f, 1.0f));

		//	jd.motorSpeed = m_motorSpeed;
		//	jd.maxMotorTorque = 10000.0f;
		//	jd.enableMotor = m_enableMotor;
		//	jd.lowerTranslation = -3.0f;
		//	jd.upperTranslation = 3.0f;
		//	jd.enableLimit = m_enableLimit;

		//	float hertz = 1.0f;
		//	float dampingRatio = 0.7f;
		//	b2LinearStiffness(jd.stiffness, jd.damping, hertz, dampingRatio, ground, body);

		//	m_joint = (b2WheelJoint*)m_world->CreateJoint(&jd);
		//}
	}

	void Step(Settings& settings) override
	{
		auto t = high_resolution_clock::now();
		Test::Step(settings);
		auto st = high_resolution_clock::now() - t;
		cout << duration_cast<microseconds>(st).count() << "\n";

		/*	float torque = m_joint->GetMotorTorque(settings.m_hertz);
			g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f", torque);
			m_textLine += m_textIncrement;

			b2Vec2 F = m_joint->GetReactionForce(settings.m_hertz);
			g_debugDraw.DrawString(5, m_textLine, "Reaction Force = (%4.1f, %4.1f)", F.x, F.y);
			m_textLine += m_textIncrement;*/
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		/*if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			m_joint->EnableLimit(m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			m_joint->EnableMotor(m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -100.0f, 100.0f, "%.0f"))
		{
			m_joint->SetMotorSpeed(m_motorSpeed);
		}*/

		ImGui::End();
	}

	static Test* Create()
	{
		return new PinchWheel;
	}

	b2WheelJoint* m_joint;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;

	float rail_lenght_ = 20.0;
	float rail_thickness_ = 0.04;

	float chassis_length = 4900.0 / 1000.0;
	float chassis_half_length = chassis_length / 2.0;
	float chassis_width = 4300.0 / 1000.0;
	float chassis_half_width = chassis_width / 2.0;

	float fl_sus_rod_length = 785.0 / 1000.0;
	float fl_sus_rod_width = 100.0 / 1000.0;

	float fr_sus_rod_length = 785.0 / 1000.0;
	float fr_sus_rod_width = 100.0 / 1000.0;


	short pinch_drive_col_grp = 2;
	short chassis_col_grp = 4;


	float pinch_wheel_radius_ = 0.35;
	float pinch_wheel_pu_density_ = 1000.0; //Die Dichte von ungeschäumtem Polyurethan variiert zwischen rund 1000 und 1250 kg/m³
};

static int testIndex = RegisterTest("905", "Pinchwheel", PinchWheel::Create);
