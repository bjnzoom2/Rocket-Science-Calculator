#pragma once
#include <vector>

class Rocket {
private:
	float thrust = 0.0f; // N
	float massFlowRate = 0.0f; // kg/s
	float exhaustVelocity = 0.0f; // m/s
	float exhaustPressure = 0.0f; // Pa
	float ambientPressure = 0.0f; // Pa
	float exitArea = 0.0f; // m2

	float dryMass = 0.0f; // kg
	float propellantMass = 0.0f; // kg
	float mass = 0.0f; // kg
	float netForce = 0.0f; // N

public:
	Rocket(float mfr, float ev, float ep = 101325, float ap = 101325, float eA = 1.0f, float dm = 1.0f, float pm = 0.0f) : massFlowRate(mfr), exhaustVelocity(ev), exhaustPressure(ep), ambientPressure(ap), exitArea(eA), dryMass(dm), propellantMass(pm) {
		thrust = (massFlowRate * exhaustVelocity) + (exhaustPressure - ambientPressure) * exitArea;
		mass = dryMass + propellantMass;

		float gForce = mass * 9.81;
		netForce = thrust - gForce;
	}

	const float getThrust() const {
		return thrust;
	}

	const float getNetForce() const {
		return netForce;
	}

	const float getAccel() const {
		return netForce / mass;
	}

	const float getBurnTime() const {
		return propellantMass / massFlowRate;
	}

	const float getMass() const {
		return mass;
	}
};