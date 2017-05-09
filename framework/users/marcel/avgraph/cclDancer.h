#pragma once

struct DancerJoint
{
	double x;
	double y;
	double vx;
	double vy;
	double ax;
	double ay;
	
	int numConnections;
};

struct DancerSpring
{
	int jointIndex1;
	int jointIndex2;
	
	double desiredDistance;
	double springFactor;
	double spasmFrequency;
	
	//
	
	double spasmPhase;
};

struct DancerEnv
{
	int numConnectedJoints;
	
	double gravityY;
	double collisionY;
	bool useDistanceConstraint;
	bool useSprings;
	bool useSpasms;
	double xFactor;
	double yFactor;
	
	DancerEnv()
		: numConnectedJoints(5)
		, gravityY(100.0)
		, collisionY(250.0)
		, useDistanceConstraint(true)
		, useSprings(false)
		, useSpasms(false)
		, xFactor(0.0)
		, yFactor(1.0)
	{
	}
};

struct Dancer
{
	static const int kMaxJoints = 32;
	static const int kMaxSprings = 1024;
	
	DancerJoint joints[kMaxJoints];
	DancerSpring springs[kMaxSprings];
	
	int numJoints;
	int numSprings;
	
	double dampeningPerSecond;
	
	double min[2];
	double max[2];
	
	double totalFitnessValue;
	
	Dancer();
	
	void calculateMinMax(double * min, double * max) const;
	
	double calculateFitness() const;
	
	void randomize();
	void randomizeSpringFactors();
	void constructFromPoints(const float * xyz, const int numPoints);
	void finalize();
	
	void tick(const double dt);
	void draw() const;
	
	void blendTo(const Dancer & target, const double amount);
};
