#pragma once

struct DancerJoint
{
	double x;
	double y;
	double vx;
	double vy;
	double ax;
	double ay;
};

struct DancerSpring
{
	int jointIndex1;
	int jointIndex2;
	
	double desiredDistance;
	double springFactor;
	double spasmFrequency;
};

struct Dancer
{
	static const int kMaxJoints = 16;
	static const int kMaxSprings = 256;
	
	DancerJoint joints[kMaxJoints];
	DancerSpring springs[kMaxSprings];
	
	int numJoints;
	int numSprings;
	
	double dampeningPerSecond;
	
	Dancer();
	
	void randomize();
	void randomizeSpringFactors();
	void constructFromPoints(const float * xyz, const int numPoints);
	void finalize();
	
	void tick(const double dt);
	void draw() const;
};
