#include "cclDancer.h"
#include "framework.h"

Dancer::Dancer()
{
	memset(this, 0, sizeof(*this));
}

void Dancer::randomize()
{
	memset(this, 0, sizeof(*this));
	
	numJoints = rand() % kMaxJoints;
	
	dampeningPerSecond = 0.9;
	
	for (int i = 0; i < numJoints; ++i)
	{
		for (int j = i + 1; j < numJoints; ++j)
		{
			DancerSpring & s = springs[numSprings++];
			
			s.jointIndex1 = i;
			s.jointIndex2 = j;
		}
	}
	
	for (int i = 0; i < numJoints; ++i)
	{
		DancerJoint & j = joints[i];
		
		j.x = random(0.0, 1000.0);
		j.y = random(0.0, 700.0);
	}
	
	randomizeSpringFactors();
	
	finalize();
}

void Dancer::randomizeSpringFactors()
{
	for (int i = 0; i < numJoints; ++i)
	{
		for (int j = i + 1; j < numJoints; ++j)
		{
			DancerSpring & s = springs[numSprings++];
			
			s.spasmFrequency = 1.0;
			//s.springFactor = random(0.0, 10000.0);
			s.springFactor = random(0.0, 100.0);
		}
	}
}

void Dancer::constructFromPoints(const float * points, const int numPoints)
{
	numJoints = numPoints;
	
	dampeningPerSecond = 0.9;
	
	for (int i = 0; i < numJoints; ++i)
	{
		DancerJoint & j = joints[i];
		
		j.x = points[i * 3 + 0];
		j.y = points[i * 3 + 0];
	}
	
	for (int i = 0; i < numJoints; ++i)
	{
		for (int j = i + 1; j < numJoints; ++j)
		{
			DancerJoint & j1 = joints[i];
			DancerJoint & j2 = joints[j];
			
			const double dx = j2.x - j1.x;
			const double dy = j2.y - j1.y;
			const double ds = std::hypot(dx, dy);
			
			if (ds < 1.0 || true)
			{
				DancerSpring & s = springs[numSprings++];
				
				s.jointIndex1 = i;
				s.jointIndex2 = j;
			}
		}
	}
	
	finalize();
}

void Dancer::finalize()
{
	for (int i = 0; i < numSprings; ++i)
	{
		DancerSpring & s = springs[i];
		
		DancerJoint & j1 = joints[s.jointIndex1];
		DancerJoint & j2 = joints[s.jointIndex2];
		
		const double dx = j2.x - j1.x;
		const double dy = j2.y - j1.y;
		const double ds = std::hypot(dx, dy);
		
		s.desiredDistance = ds;
	}
}

void Dancer::tick(const double dt)
{
	if (dt <= 0.f)
		return;
	
	const double eps = 0.00001;
	const double dtMax = 1.0 / 1000.0;
	
	const int numSteps = int(std::ceil(dt / dtMax));
	const double dtReal = dt / numSteps;
	
	const double dampeningPerStep = std::pow(1.0 - dampeningPerSecond, dtReal);
	
	if (mouse.wentDown(BUTTON_LEFT))
	{
		const int index = rand() % numJoints;
		DancerJoint & j = joints[index];
		
		j.vx += random(-1.0, +1.0) * 100.0;
		j.vy += random(-1.0, +1.0) * 100.0;
	}
	
	for (int i = 0; i < numSteps; ++i)
	{
		for (int j = 0; j < numJoints; ++j)
		{
			DancerJoint & jt = joints[j];
			
			jt.ax = 0.0;
			jt.ay = 0.0;
		}
		
		for (int j = 0; j < numSprings; ++j)
		{
			const DancerSpring & s = springs[j];
			DancerJoint & j1 = joints[s.jointIndex1];
			DancerJoint & j2 = joints[s.jointIndex2];
			
			const double dx = j2.x - j1.x;
			const double dy = j2.y - j1.y;
			const double ds = std::hypot(dx, dy) + eps;
			const double nx = dx / ds;
			const double ny = dy / ds;
			
			const double dd = ds - s.desiredDistance;
			const double a = dd * s.springFactor;
			
			j1.ax += nx * a;
			j1.ay += ny * a;
			
			j2.ax -= nx * a;
			j2.ay -= ny * a;
		}
		
		for (int j = 0; j < numJoints; ++j)
		{
			DancerJoint & jt = joints[j];
			
			jt.vx += jt.ax * dtReal;
			jt.vy += jt.ay * dtReal;
			
			jt.vx *= dampeningPerStep;
			jt.vy *= dampeningPerStep;
			
			jt.x += jt.vx * dtReal;
			jt.y += jt.vy * dtReal;
		}
	}
}

void Dancer::draw() const
{
	for (int i = 0; i < numSprings; ++i)
	{
		const DancerSpring & s = springs[i];
		const DancerJoint & j1 = joints[s.jointIndex1];
		const DancerJoint & j2 = joints[s.jointIndex2];
		
		hqBegin(HQ_LINES);
		{
			setColor(colorWhite);
			hqLine(j1.x, j1.y, 2.f, j2.x, j2.y, 2.f);
		}
		hqEnd();
	}
	
	hqBegin(HQ_FILLED_CIRCLES);
	{
		for (int i = 0; i < numJoints; ++i)
		{
			const DancerJoint & j = joints[i];
			
			setColor(colorYellow);
			hqFillCircle(j.x, j.y, 4.f);
		}
	}
	hqEnd();
}
