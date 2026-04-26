#pragma once
#include <cmath>

#include "rocketMath.h"

class SolidRocket {
private:
	// Engine Variables
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

	float propellantDensity = 0.0f; // kg/m3
	float burnArea = 0.0f; // m2
	float burnRate = 0.0f; // m/s

	float initCoreRadius = 0.0f; // m
	float currentCoreRadius = 0.0f; // m
	float outerRadius = 0.0f; // m
	float grainLength = 0.0f; // m

	float burnRateCoeff = 0.0f;
	float combustionPressure = 0.0f; // Pa
	float pressureExponent = 0.0f;

	float velocity = 0.0f; // m/s
	float height = 0.0f; // m
	float maxAltitude = 0.0f; // m

	float drag = 0.0f;
	float dragCoeff = 0.0f;
	float referenceArea = 0.0f; // m2
	float dynamicDragCoeff = 0.0f;

	float chamberTemp = 0.0f;
	float gasConstant = 0.0f;
	float heatRatio = 0.0f;
	float throatArea = 0.0f;

	float exitTemp = 0.0f;

	float airDensity = 0.0f;

	float noseconeLength = 0.0f; // m
	float bodyLength = 0.0f; // m

	float finenessRatio = 0.0f;

	void calculateEngineState() {
		if (propellantMass > 0.0f) {
			burnArea = RocketMath::getProgressiveBurnArea(currentCoreRadius, outerRadius, grainLength);

			float exitMach = RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio);
			float gasFlow = RocketMath::getGasFlowabitilty(chamberTemp, gasConstant, heatRatio);
			combustionPressure = RocketMath::getCombustionPressure(burnArea, throatArea, propellantDensity, burnRateCoeff, pressureExponent, gasFlow);

			burnRate = RocketMath::getBurnRateSolid(burnRateCoeff, combustionPressure, pressureExponent);
			massFlowRate = RocketMath::getMFRSolid(propellantDensity, burnArea, burnRate);
			exhaustPressure = RocketMath::getExhaustPressureSolid(combustionPressure, heatRatio, exitMach);

			exitTemp = RocketMath::getExitTempSolid(chamberTemp, heatRatio, exitMach);
			exhaustVelocity = RocketMath::getExhaustVeloSolid(heatRatio, exitMach, gasConstant, exitTemp);

			thrust = massFlowRate * exhaustVelocity + (exhaustPressure - ambientPressure) * exitArea;
		}
		else {
			exhaustVelocity = 0.0f;
			massFlowRate = 0.0f;
			thrust = 0.0f;
			burnRate = 0.0f;
		}
	}

	void updateForces() {
		mass = dryMass + propellantMass;
		float gForce = mass * 9.80665f;

		if (velocity > 0) netForce = thrust - gForce - drag;
		else netForce = thrust - gForce + drag;
	}

public:
	SolidRocket(float dryM, float propellantM, float propellantD = 0.0f, float initCR = 0.0f, float outerR = 0.0f, float grainL = 0.0f, float burnRC = 0.0f, float throatA = 0.0f, float pressureEx = 0.0f, float exhaustP = 1.0f, float ambientP = 1.0f, float exitA = 0.0f, float referenceA = 0.0f, float heatRa = 0.0f,
		float chamberT = 0.0f, float gasC = 0.0f, float noseconeL = 0.0f, float bodyL = 0.0f) :
		dryMass(dryM), propellantMass(propellantM), propellantDensity(propellantD), initCoreRadius(initCR), currentCoreRadius(initCR), outerRadius(outerR), grainLength(grainL), burnRateCoeff(burnRC), throatArea(throatA), pressureExponent(pressureEx),
		exhaustPressure(exhaustP), ambientPressure(ambientP), exitArea(exitA), referenceArea(referenceA), heatRatio(heatRa), chamberTemp(chamberT), gasConstant(gasC), noseconeLength(noseconeL), bodyLength(bodyL) {

		dragCoeff = RocketMath::getDragCD(noseconeLength, bodyLength, referenceArea, finenessRatio);

		calculateEngineState();
	}

	void update(float dt) {
		ambientPressure = 101325.0f * std::exp(-height / 8500.0f);
		float airTempK = (32.0f - 0.0065f * height) + 273.15;
		airDensity = ambientPressure / (287.058f * airTempK);

		float soundSpeed = std::sqrt(1.4f * 287.058f * airTempK);
		float mach = std::abs(velocity) / soundSpeed;

		dynamicDragCoeff = RocketMath::getWaveDragCD(mach, dragCoeff, finenessRatio);

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

			currentCoreRadius += burnRate * dt;

			calculateEngineState();
		}

		drag = 0.5f * airDensity * (velocity * velocity) * dynamicDragCoeff * referenceArea;
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
	const float getDynamicDragCoeff() const { return dynamicDragCoeff; }
};