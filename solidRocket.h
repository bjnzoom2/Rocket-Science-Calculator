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

    float initPropellantMass = 0.0f; // kg
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
    float highestVelo = 0.0f; // m/s

    float drag = 0.0f;
    float dragCoeff = 0.0f;
    float referenceArea = 0.0f; // m2
    float dynamicDragCoeff = 0.0f;
    float mach = 0.0f;

    float chamberTemp = 0.0f;
    float gasConstant = 0.0f;
    float heatRatio = 0.0f;
    float throatArea = 0.0f;

    float exitTemp = 0.0f;

    float airDensity = 0.0f;

    float noseconeLength = 0.0f; // m
    float bodyLength = 0.0f; // m
    int numFins = 0;
    float finRootChord = 0.0f; // m
    float finTipChord = 0.0f; // m
    float finSpan = 0.0f; // m
    float finThickness = 0.0f; // m

    float finenessRatio = 0.0f;
    
    int numSegments = 0;

    void calculateEngineState() {
        if (propellantMass > 0.0f) {
            burnArea = RocketMath::getBATESBurnArea(currentCoreRadius, grainLength, numSegments, outerRadius);

            float exitMach = RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio);
            float gasFlow = RocketMath::getGasFlowabitilty(chamberTemp, gasConstant, heatRatio);
            combustionPressure = RocketMath::getCombustionPressure(burnArea, throatArea, propellantDensity, burnRateCoeff, pressureExponent, gasFlow);

            burnRate = RocketMath::getBurnRateSolid(burnRateCoeff, combustionPressure, pressureExponent);
            massFlowRate = RocketMath::getMFRSolid(propellantDensity, burnArea, burnRate);
            exhaustPressure = RocketMath::getExhaustPressureSolid(combustionPressure, heatRatio, exitMach);

            exitTemp = RocketMath::getExitTempSolid(chamberTemp, heatRatio, exitMach);
            exhaustVelocity = RocketMath::getExhaustVeloSolid(heatRatio, exitMach, gasConstant, exitTemp);

            thrust = massFlowRate * exhaustVelocity + (exhaustPressure - ambientPressure) * exitArea;

            if (thrust < 0.0f) thrust = 0.0f;
        }
        else {
            exhaustVelocity = 0.0f;
            massFlowRate = 0.0f;
            thrust = 0.0f;
            burnRate = 0.0f;
            exhaustPressure = 0.0f;
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
        float chamberT = 0.0f, float gasC = 0.0f, float noseconeL = 0.0f, float bodyL = 0.0f, int numF = 0.0f, float finRC = 0.0f, float finTC = 0.0f, float finS = 0.0f, float finT = 0.0f, int numSeg = 0) :
        dryMass(dryM), initPropellantMass(propellantM), propellantMass(propellantM), propellantDensity(propellantD), initCoreRadius(initCR), currentCoreRadius(initCR), outerRadius(outerR), grainLength(grainL), burnRateCoeff(burnRC), throatArea(throatA), pressureExponent(pressureEx),
        exhaustPressure(exhaustP), ambientPressure(ambientP), exitArea(exitA), referenceArea(referenceA), heatRatio(heatRa), chamberTemp(chamberT), gasConstant(gasC), noseconeLength(noseconeL), bodyLength(bodyL), numFins(numF), finRootChord(finRC), finTipChord(finTC), finSpan(finS), 
        finThickness(finT), numSegments(numSeg) {

        finenessRatio = (noseconeLength + bodyLength) / (2.0f * outerRadius);
        dragCoeff = RocketMath::getDragCD(noseconeLength, bodyLength, referenceArea, finenessRatio,
            numFins, finRootChord, finTipChord, finSpan, finThickness);

        calculateEngineState();
    }

    void update(float dt) {
        ambientPressure = 101325.0f * std::exp(-height / 8500.0f);
        float groundTempK = 32.0f + 273.15f;
        float airTempK = groundTempK - (0.0065f * height);

        if (airTempK < 216.65f) airTempK = 216.65f; // Tropopause temperature
        airDensity = ambientPressure / (287.058f * airTempK);

        float soundSpeed = std::sqrt(1.4f * 287.058f * airTempK);
        mach = std::abs(velocity) / soundSpeed;

        if (propellantMass > 0.0f && currentCoreRadius < outerRadius) {
            float massToBurn = massFlowRate * dt;

            if (massToBurn >= propellantMass || (currentCoreRadius + burnRate * dt) >= outerRadius) {
                propellantMass = 0.0f;
                currentCoreRadius = outerRadius;
                calculateEngineState();
            }
            else {
                propellantMass -= massToBurn;
                float regression = burnRate * dt;
                currentCoreRadius += regression;
                grainLength -= 2.0f * regression;
                calculateEngineState();
            }
        }
        else {
            propellantMass = 0.0f;
            calculateEngineState();
        }

        dynamicDragCoeff = RocketMath::getWaveDragCD(mach, dragCoeff, finenessRatio);

        drag = 0.5f * airDensity * (velocity * velocity) * dynamicDragCoeff * referenceArea;
        updateForces();

        velocity += netForce / mass * dt;
        height += velocity * dt;

        if (height > maxAltitude) maxAltitude = height;
        if (velocity > highestVelo) highestVelo = velocity;
    }

    const float getThrust() const { return thrust; }
    const float getNetForce() const { return netForce; }
    const float getMFR() const { return massFlowRate; }
    const float getAccel() const { return netForce / mass; }
    const float getMass() const { return mass; }
    const float getPropellantMass() const { return propellantMass; }
    const float getExhaustPressure() const { return exhaustPressure; }
    const float getCombustionPressure() const { return combustionPressure; }
    const float getEffExhaustVelo() const { return thrust / massFlowRate; }
    const float getVelocity() const { return velocity; }
    const float getHeight() const { return height; }
    const float getMaxAltitude() const { return maxAltitude; }
    const float getHighestVelo() const { return highestVelo; }
    const float getDrag() const { return drag; }
    const float getDragCoeff() const { return dragCoeff; }
    const float getDynamicDragCoeff() const { return dynamicDragCoeff; }
    const float getMach() const { return mach; }
};