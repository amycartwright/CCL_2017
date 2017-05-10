#pragma once

static const int kMaxJoints = 128;
static const int kMaxSprings = 1024;

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
	
	double maximumDistance;
	
	//
	
	double spasmPhase;
};

struct DancerEnv
{
	struct LiveData
	{
		int numPoints;
		
		double x[kMaxJoints];
		double y[kMaxJoints];
		double min[2];
		double max[2];
		
		LiveData()
			: numPoints(0)
			, x()
			, y()
			, min()
			, max()
		{
		}
	};
	
	int numConnectedJoints;
	float maxDistanceFactor;
	
	double gravityY;
	double collisionY;
	bool useDistanceConstraint;
	bool useSprings;
	bool useSpasms;
	double xFactor;
	double yFactor;
	LiveData liveData;
	
	DancerEnv()
		: numConnectedJoints(5)
		, maxDistanceFactor(2.f)
		, gravityY(100.0)
		, collisionY(250.0)
		, useDistanceConstraint(true)
		, useSprings(false)
		, useSpasms(false)
		, xFactor(1.0)
		, yFactor(1.0)
		, liveData()
	{
	}
};

struct Dancer
{
	DancerJoint joints[kMaxJoints];
	DancerSpring springs[kMaxSprings];
	
	int numJoints;
	int numSprings;
	
	double dampeningPerSecond;
	
	double accelTowardsOtherDancer;
	
	double min[2];
	double max[2];
	
	double totalFitnessValue;
	
	Dancer();
	
	void calculateMinMax(double * min, double * max) const;
	
	double calculateFitness(const int fitnessFunction) const;
	
	void randomize();
	void randomizeSpringFactors();
	void constructFromPoints(const float * xyz, const int numPoints);
	void finalize();
	
	void tick(const double dt, const int fitnessFunction);
	void draw() const;
	
	void blendTo(const Dancer & target, const double amount);
};

extern DancerEnv env;
