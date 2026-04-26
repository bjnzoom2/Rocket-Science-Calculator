#pragma once
#include <vector>
#include <iostream>
#include "rocketMath.h"

class WaterRocket {
private:
	// Engine Variables
	float thrust = 0.0f; // N
	float massFlowRate = 0.0f; // kg/s
	float exhaustVelocity = 0.0f; // m/s
	float exhaustPressure = 0.0f; // Pa
	float ambientPressure = 0.0f; // Pa
	float exitArea = 0.0f; // m2

	// Mass Variables
	float dryMass = 0.0f; // kg
	float propellantMass = 0.0f; // kg
	float mass = 0.0f; // kg
	float netForce = 0.0f; // N

	// Decay Variables
	float initPa = 0.0f; // Pa
	float initAirVolume = 0.0f; // m3
	float currentAirVolume = 0.0f; // m3
	float liquidDensity = 0.0f; // kg/m3

	// Position Variables
	float velocity = 0.0f;
	float height = 0.0f;
	float maxAltitude = 0.0f;

	// Other Forces
	float drag = 0.0f;
	float dragCoeff = 0.0f;
	float referenceArea = 0.0f; // m2

	void calculateEngineState() {
		if (exhaustPressure > ambientPressure && propellantMass > 0.0f) {
			exhaustVelocity = RocketMath::getExhaustVeloLiquid(exhaustPressure, ambientPressure, liquidDensity);
			massFlowRate = RocketMath::getMFR(liquidDensity, exitArea, exhaustVelocity);
			thrust = massFlowRate * exhaustVelocity;
		}
		else {
			exhaustVelocity = 0.0f;
			massFlowRate = 0.0f;
			thrust = 0.0f;
		}
	}

	void updateForces() {
		mass = dryMass + propellantMass;
		float gForce = mass * 9.80665f;

		if (velocity > 0) {
			netForce = thrust - gForce - drag;
		}
		else {
			netForce = thrust - gForce + drag;
		}
	}

public:
	WaterRocket(float liquidD, float initP = 101325, float ambientP = 101325, float exitA = 1.0f, float dryM = 1.0f, float propellantM = 0.0f, float bottleV = 0.0f, float dragC = 0.0f, float referenceA = 0.0f) : exhaustPressure(initP), initPa(initP), ambientPressure(ambientP), exitArea(exitA), dryMass(dryM), propellantMass(propellantM), liquidDensity(liquidD), dragCoeff(dragC), referenceArea(referenceA) {
		float waterVolume = propellantMass / liquidDensity;
		initAirVolume = bottleV - waterVolume;
		currentAirVolume = initAirVolume;

		calculateEngineState();
		updateForces();
	}

	void update(float dt) {
		if (propellantMass <= 0.0f) {
			thrust = 0.0f;
			exhaustVelocity = 0.0f;
			massFlowRate = 0.0f;
			propellantMass = 0.0f;
		}
		else {
			float massLost = massFlowRate * dt;
			if (massLost > propellantMass) massLost = propellantMass;
			propellantMass -= massLost;

			float volumeLost = massLost / liquidDensity;
			currentAirVolume += volumeLost;

			exhaustPressure = RocketMath::getBoyleLawPressure(initAirVolume, initPa, currentAirVolume);
			calculateEngineState();
		}

		drag = 0.5f * 1.225f * (velocity * velocity) * dragCoeff * referenceArea;
		updateForces();

		velocity += netForce / mass * dt;
		height += velocity * dt;

		if (height > maxAltitude) maxAltitude = height;
	}

	const float getThrust() const { return thrust; }
	const float getNetForce() const { return netForce; }
	const float getAccel() const { return netForce / mass; }
	const float getMass() const { return mass; }
	const float getPropellantMass() const { return propellantMass; }
	const float getExhaustPressure() const { return exhaustPressure; }
	const float getVelocity() const { return velocity; }
	const float getHeight() const { return height; }
	const float getMaxAltitude() const { return maxAltitude; }
	const float getDragCoeff() const { return dragCoeff; }

	void setInitialState(float v, float h) {
		velocity = v;
		height = h;
		maxAltitude = h;
	}

	void setDragCoeff(float dc) {
		dragCoeff = dc;
	}
};

class WaterRocketMulti {
private:
	std::vector<WaterRocket*> stages = {};
	WaterRocket* active;
	int stageIndex = 0;

	float startCd = 0.0f;

	float getCurrentCd() {
		int remainingStages = stages.size() - stageIndex;
		if (remainingStages <= 0) return 0.0f;

		float cd = startCd;

		cd += remainingStages * 0.05f;

		if (remainingStages > 1) {
			cd += (remainingStages - 1) * 0.08f;
		}

		return cd;
	}
public:
	WaterRocketMulti(std::vector<WaterRocket*> _stages) : stages(_stages) {
		if (!stages.empty()) {
			startCd = stages[stages.size() - 1]->getDragCoeff();
		}
		else return;

		stages[0]->setDragCoeff(getCurrentCd());
	}

	void update(float dt) {
		if (stageIndex >= stages.size()) return;
		active = stages[stageIndex];
		active->update(dt);

		if (active->getPropellantMass() <= 0 && stageIndex < stages.size() - 1) {
			float velo = active->getVelocity();
			float height = active->getHeight();

			stageIndex++;

			stages[stageIndex]->setInitialState(velo, height);
			stages[stageIndex]->setDragCoeff(getCurrentCd());

			std::cout << "Stage " << stageIndex + 1 << " Ignition\n";
		}
	}

	const WaterRocket* getActive() const { return stages[stageIndex]; }
};