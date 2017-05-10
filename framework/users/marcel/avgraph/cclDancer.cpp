#include "cclDancer.h"
#include "framework.h"

static const double eps = 0.00001;

DancerEnv env;

static double Mix(const double a, const double b, const double t)
{
	const double t1 = 1.0 - t;
	const double t2 = t;
	
	return a * t1 + b * t2;
}

const char * getFitnessFunctionName(FitnessFunction f)
{
	if (f == kFitnessFunction_GetSmall)
		return "getSmall";
	else if (f == kFitnessFunction_GetLarge)
		return "getLarge";
	else if (f == kFitnessFunction_MatchPose)
		return "matchPose";
	else
	{
		Assert(false);
		return "n/a";
	}
}

//

Dancer::Dancer()
{
	memset(this, 0, sizeof(*this));
}

void Dancer::calculateMinMax(double * min, double * max) const
{
	min[0] = 0.0;
	min[1] = 0.0;
	max[0] = 0.0;
	max[1] = 0.0;
	
	for (int i = 0; i < numJoints; ++i)
	{
		const DancerJoint & j = joints[i];
		
		if (i == 0)
		{
			min[0] = j.x;
			min[1] = j.y;
			max[0] = j.x;
			max[1] = j.y;
		}
		else
		{
			min[0] = std::min(min[0], j.x);
			min[1] = std::min(min[1], j.y);
			max[0] = std::max(max[0], j.x);
			max[1] = std::max(max[1], j.y);
		}
	}
}

double Dancer::calculateFitness(const FitnessFunction fitnessFunction) const
{
    if (fitnessFunction == kFitnessFunction_GetSmall)
    {
		const double sy = max[1] - min[1] + eps;
		
        if (sy == 0.0)
            return eps;
        else
            return 1.0 / sy;
    }
    else if (fitnessFunction == kFitnessFunction_GetLarge)
    {
		const double sy = max[1] - min[1] + eps;
		
        return sy;
    }
	else if (fitnessFunction == kFitnessFunction_MatchPose)
	{
		double totalDistance = 0.0;
		
		const int numPoints = std::min(numJoints, env.liveData.numPoints);
		
		if (numPoints > 0)
		{
			double py1[kMaxJoints];
			double py2[kMaxJoints];
			
			double px1[kMaxJoints];
			double px2[kMaxJoints];
			
			for (int i = 0; i < numPoints; ++i)
			{
				px1[i] = joints[i].x;
				py1[i] = joints[i].y;
				
				px2[i] = env.liveData.x[i];
				py2[i] = env.liveData.y[i];
			}
			
			std::sort(px1, px1 + numPoints);
			std::sort(py1, py1 + numPoints);
			
			std::sort(px2, px2 + numPoints);
			std::sort(py2, py2 + numPoints);
			
			double totalDxSq = 0.0;
			double totalDySq = 0.0;
			
			for (int i = 0; i < numPoints; ++i)
			{
				const double dx = std::abs(px2[i] - px1[i]) * env.xFactor;
				const double dy = std::abs(py2[i] - py1[i]) * env.yFactor;
				
				totalDxSq += dx * dx;
				totalDySq += dy * dy;
			}
			
			//totalDistance += totalDxSq;
			totalDistance += std::sqrt(totalDySq);
			
			//
			
			const double sx1 = max[0] - min[0];
			const double sx2 = env.liveData.max[0] - env.liveData.min[0];
			//const double dsx = std::abs(sx2 - sx1);
			const double dsx = std::abs(0.0 - sx1) * env.xFactor;
			
			totalDistance += std::sqrt(dsx * dsx) * numPoints;
		}
		
		totalDistance += 1.0;
		
		return 1.0 / totalDistance;
	}
	else
	{
		Assert(false);
		return 1.0;
	}
}


void Dancer::randomize()
{
#if 1
	const int numPoints = (rand() % (kMaxJoints - 3)) + 3;
	
	float points[numPoints * 3];
	
	for (int i = 0; i < numPoints; ++i)
	{
		points[i * 3 + 0] = random(-100.0, +100.0);
		points[i * 3 + 1] = random(-200.0, +200.0);
		points[i * 3 + 2] = 0.0;
	}
	
	constructFromPoints(points, numPoints);
#else
	memset(this, 0, sizeof(*this));
	
	numJoints = (rand() % (kMaxJoints - 3)) + 3;
	
	dampeningPerSecond = 0.8;
	
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
		
		j.x = random(-100.0, +100.0);
		j.y = random(-200.0, +200.0);
	}
	
	randomizeSpringFactors();
	
	finalize();
#endif
}

void Dancer::randomizeSpringFactors()
{
	for (int i = 0; i < numSprings; ++i)
	{
		DancerSpring & s = springs[i];
		
		s.spasmFrequency = random(0.05, 0.1) * 2.0 * M_PI;
		s.spasmPhase = random(0.0, 1.0) * 2.0 * M_PI;
		s.springFactor = random(0.0, 200.0);
		//s.springFactor = random(100.0, 10000.0);
		//s.springFactor = random(100000.0, 100000.0);
	}
	
	accelTowardsOtherDancer = 100.0;
}

void Dancer::constructFromPoints(const float * points, const int numPoints)
{
	memset(this, 0, sizeof(*this));
	
	dampeningPerSecond = 0.9;
	
	for (int i = 0; i < numPoints; ++i)
	{
		const double x = points[i * 3 + 0];
		const double y = points[i * 3 + 1];
		
		if (std::isnan(x) ||
			std::isnan(y))
		{
			continue;
		}
		
		DancerJoint & j = joints[numJoints++];
		
		j.x = x;
		j.y = y;
		
		Assert(!std::isnan(j.x));
		Assert(!std::isnan(j.y));
		Assert(!std::isinf(j.x));
		Assert(!std::isinf(j.y));
	}
	
	calculateMinMax(min, max);
	
	const double sx = max[0] - min[0];
	const double sy = max[1] - min[1];
	const double diagonalSize = std::hypot(sx, sy);
	const double connectionTreshold = diagonalSize * 0.3;
	
	struct Connection
	{
		int index1;
		int index2;
		
		double distance;
		
		bool operator<(const Connection & other) const
		{
			if (index1 != other.index1)
				return index1 < other.index1;
			return index2 < other.index2;
		}
	};
	
	std::set<Connection> connections;
	
	for (int i = 0; i < numJoints; ++i)
	{
		std::vector<Connection> candidates;
		
		for (int j = 0; j < numJoints; ++j)
		{
			if (i == j)
				continue;
			
			// already connected ?
			
			Connection c;
			c.index1 = i;
			c.index2 = j;
			
			if (connections.count(c) != 0)
				continue;
			
			DancerJoint & j1 = joints[i];
			DancerJoint & j2 = joints[j];
			
			const double dx = j2.x - j1.x;
			const double dy = j2.y - j1.y;
			const double ds = std::hypot(dx, dy);
			
			c.distance = ds;
			
			candidates.push_back(c);
		}
		
		std::sort(candidates.begin(), candidates.end(), [](const Connection & c1, const Connection & c2) { return c1.distance < c2.distance; });
		
		DancerJoint & jt = joints[i];
		
		for (int i = 0; jt.numConnections < env.numConnectedJoints && i < candidates.size(); ++i)
		{
			Connection & c = candidates[i];
			
			if (c.distance < connectionTreshold || true)
			{
				jt.numConnections++;
				
				connections.insert(c);
			
				DancerSpring & s = springs[numSprings++];
				
				s.jointIndex1 = c.index1;
				s.jointIndex2 = c.index2;
			}
		}
	}
	
	randomizeSpringFactors();
	
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
		s.maximumDistance = ds * env.maxDistanceFactor;
	}
}

void Dancer::tick(const double dt, const FitnessFunction fitnessFunction)
{
	if (dt <= 0.f)
		return;
	
	const double dtMax = 1.0 / 100.0;
	
	const int numSteps = int(std::ceil(dt / dtMax));
	
	if (numSteps <= 0)
		return;
	
	const double dtReal = dt / numSteps;
	
	const double dampeningPerStep = std::pow(1.0 - dampeningPerSecond, dtReal);
	
	if (mouse.wentDown(BUTTON_LEFT))
	{
		for (int i = 0; i < numJoints; ++i)
		{
			//const int index = rand() % numSprings;
			const int index = i;
			
			DancerSpring & s = springs[index];
			DancerJoint & j1 = joints[s.jointIndex1];
			DancerJoint & j2 = joints[s.jointIndex2];
			const double dx = j2.x - j1.x;
			const double dy = j2.y - j1.y;
			const double ds = std::hypot(dx, dy) + eps;
			const double nx = dx / ds;
			const double ny = dy / ds;
			
			const double speed = 200.0 / 2.0;
			
			j1.vx += nx * speed;
			j1.vy += ny * speed;
			j2.vx -= nx * speed;
			j2.vy -= ny * speed;
		}
	}
	
	for (int i = 0; i < numSteps; ++i)
	{
    #if 0
		if (env.useSpasms)
		{
			// apply spasms
			
			for (int j = 0; j < numSprings; ++j)
			{
				DancerSpring & s = springs[j];
				DancerJoint & j1 = joints[s.jointIndex1];
				DancerJoint & j2 = joints[s.jointIndex2];
				const double dx = j2.x - j1.x;
				const double dy = j2.y - j1.y;
				const double ds = std::hypot(dx, dy) + eps;
				const double nx = dx / ds;
				const double ny = dy / ds;
				
				s.spasmPhase += s.spasmFrequency * dtReal;
				
				const double speed = std::sin(s.spasmPhase) * 5.0 / 2.0;
				
				j1.vx += nx * speed;
				j1.vy += ny * speed;
				j2.vx -= nx * speed;
				j2.vy -= ny * speed;
			}
		}
    #endif
	
	#if 0
		{
			// apply acceleration towards dancer
			
			const double centerX = (min[0] + max[0]) / 2.0;
			const double dancerCenterX = (env.liveData.min[0] + env.liveData.max[0]) / 2.0;
			const double dx = dancerCenterX - centerX;
			
			for (int i = 0; i < numJoints; ++i)
			{
				DancerJoint & j = joints[i];
				
				j.x += dx;
			}
		}
	#endif
		
		for (int j = 0; j < numJoints; ++j)
		{
			DancerJoint & jt = joints[j];
			
			jt.ax = 0.0;
			jt.ay = 0.0;
			
			// gravity
			
			jt.ay += env.gravityY;
			
			// collision
			
			if (jt.y > env.collisionY)
				jt.y = env.collisionY;
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
			
			if (env.useDistanceConstraint)
			{
				const double fitting = 1.0 / 2.0 / 2.0;
				j1.x += dd * nx * fitting * env.xFactor;
				j1.y += dd * ny * fitting * env.yFactor;
				j2.x -= dd * nx * fitting * env.xFactor;
				j2.y -= dd * ny * fitting * env.yFactor;
			}
			
			if (env.useSprings)
			{
				j1.ax += nx * a;
				j1.ay += ny * a;
				
				j2.ax -= nx * a;
				j2.ay -= ny * a;
			}
		}
		
		for (int j = 0; j < numJoints; ++j)
		{
			DancerJoint & jt = joints[j];
			
			jt.vx += jt.ax * dtReal;
			jt.vy += jt.ay * dtReal;
			
			jt.vx *= dampeningPerStep;
			jt.vy *= dampeningPerStep;
			
			jt.x += jt.vx * dtReal * env.xFactor;
			jt.y += jt.vy * dtReal * env.yFactor;
		}
	}
	
	calculateMinMax(min, max);
	
	totalFitnessValue += calculateFitness(fitnessFunction) * dt;
}

void Dancer::draw() const
{
	if (env.debugDraw)
	{
		setColor(colorBlue);
		drawRectLine(min[0], min[1], max[0], max[1]);
	}
	
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

void Dancer::blendTo(const Dancer & target, const double amount)
{
	for (int i = 0; i < numSprings; ++i)
	{
		DancerSpring & dst = springs[i];
		const DancerSpring & src = target.springs[i];
		
		dst.desiredDistance = Mix(dst.desiredDistance, src.desiredDistance, amount);
		dst.spasmFrequency = Mix(dst.spasmFrequency, src.spasmFrequency, amount);
		dst.springFactor = Mix(dst.springFactor, src.springFactor, amount);
	}
}
