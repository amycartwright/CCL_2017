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
};
