#pragma once
#include <cmath>
#include <iostream>
#include "rocketMath.h"

class LiquidRocket {
private:
    // Engine Variables
    float thrust = 0.0f; // N
    float massFlowRate = 0.0f; // kg/s
    float exhaustVelocity = 0.0f; // m/s
    float exhaustPressure = 101325.0f; // Pa
    float ambientPressure = 101325.0f; // Pa
    float throatArea = 0.0f; // m2
    float exitArea = 0.0f; // m2

    float airDensity = 0.0f; // kg/m3

    // Mass Variables
    float propellantMass = 0.0f;
    float dryMass = 0.0f;
    float mass = 0.0f;
    float netForce = 0.0f; // N

    float Isp = 0.0f;
    float heatRatio = 0.0f;
    float gasConstant = 0.0f;
    float chamberTemp = 0.0f;

    // Fluid Variables
    float fuelDensity = 0.0f; // kg/m3
    float oxidizerDensity = 0.0f; // kg/m3
    float mixtureRatio = 0.0f;
    float bulkDensity = 0.0f; // kg/m3

    // Tank and Pump Variables
    float tankPressure = 0.0f; // Pa
    float dischargePressure = 0.0f; // Pa
    float pumpPowerReq = 0.0f; // W
    float shaftVelocity = 0.0f; // RPM
    float shaftTorque = 0.0f; // Nm
    float shaftInertia = 0.0f; // kg * m2
    float pumpEfficiency = 0.0f;
    float turbineEfficiency = 0.0f;
    float turbinePower = 0.0f;
    float injectorArea = 0.0f; // m2
    float injectorDischargeCoeff = 0.0f;
    float pumpConstant = 0.0f;
    float flowResistanceCoeff = 0.0f;

    float turbineInletTemp = 0.0f;

    float turbineBleedRato = 0.0f;
    float massFlowRateTurbine = 0.0f; // kg/s

    float chamberPressure = 101325.0f; // Pa
    float characteristicVelocity = 0.0f; // m/s
    float SHCP = 0.0f; // J/(kg * K)

    // Position Variables
    float velocity = 0.0f; // m/s
    float height = 0.0f; // m
    float maxAltitude = 0.0f; // m
    float highestVelo = 0.0f; // m/s

    // Drag
    float drag = 0.0f;
    float dragCoeff = 0.0f;
    float referenceArea = 0.0f; // m2
    float dynamicDragCoeff = 0.0f;
    float mach = 0.0f;
    float finenessRatio = 0.0f;

    float noseconeLength = 0.0f; // m
    float bodyLength = 0.0f; // m
    int numFins = 0;
    float finRootChord = 0.0f; // m
    float finTipChord = 0.0f; // m
    float finSpan = 0.0f; // m
    float finThickness = 0.0f; // m

    float time = 0.0f;

    void calculateEngineState(float dt) {
        time += dt;

        if (propellantMass > 0.0f) {
            bulkDensity = RocketMath::getBulkPropellantDensity(fuelDensity, oxidizerDensity, mixtureRatio);
            dischargePressure = RocketMath::getPumpDischargePressure(tankPressure, pumpConstant, bulkDensity, shaftVelocity);

            chamberPressure = RocketMath::getChamberPressure(injectorDischargeCoeff, injectorArea, characteristicVelocity, throatArea, bulkDensity, dischargePressure);
            massFlowRate = RocketMath::getMFRLiquid(injectorDischargeCoeff, injectorArea, bulkDensity, dischargePressure, chamberPressure);

            massFlowRateTurbine = massFlowRate * turbineBleedRato;

            float totalMassFlow = massFlowRate + massFlowRateTurbine;

            float exitMach = RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio);
            exhaustPressure = RocketMath::getExhaustPressure(chamberPressure, heatRatio, exitMach);

            exhaustVelocity = RocketMath::getExhaustVeloLiquid(heatRatio, gasConstant, chamberTemp, exhaustPressure, chamberPressure);

            thrust = massFlowRate * exhaustVelocity + ((exhaustPressure - ambientPressure) * exitArea);

            pumpPowerReq = RocketMath::getPumpPowerReq(totalMassFlow, dischargePressure, tankPressure, bulkDensity, pumpEfficiency);
            SHCP = RocketMath::getSHCP(heatRatio, gasConstant);

            float turbineInletPressure = dischargePressure - RocketMath::getFrictionLoss(flowResistanceCoeff, bulkDensity, massFlowRateTurbine);

            turbinePower = RocketMath::getTurbinePower(massFlowRateTurbine, SHCP, turbineInletTemp, turbineInletPressure, ambientPressure, heatRatio, turbineEfficiency);

            float starterTorque = (time < 0.5f) ? 50.0f : 0.0f;
            shaftTorque = ((turbinePower - pumpPowerReq) / std::max(shaftVelocity, 1.0f)) + starterTorque;

            float angularAccel = shaftTorque / shaftInertia;
            shaftVelocity += angularAccel * dt;
        }
        else {
            exhaustVelocity = 0.0f;
            massFlowRate = 0.0f;
            massFlowRateTurbine = 0.0f;
            thrust = 0.0f;
            exhaustPressure = 0.0f;
            shaftTorque = 0.0f;

            shaftVelocity = std::max(0.0f, shaftVelocity - (shaftVelocity * 0.1f * dt));
        }
    }

    void updateForces() {
        mass = dryMass + propellantMass;
        float gForce = mass * 9.80665f;

        if (velocity > 0) netForce = thrust - gForce - drag;
        else netForce = thrust - gForce + drag;
    }

public:
    LiquidRocket(float throatA = 0.0f, float exitA = 0.0f, float injectorA = 0.0f, float heatRa = 0.0f, float gasC = 0.0f, float chamberT = 0.0f,
        float fuelD = 0.0f, float oxidizerD = 0.0f, float mixtureRa = 0.0f, float shaftI = 0.0f, float pumpEff = 0.0f, float turbineEff = 0.0f, float tankP = 0.0f,
        float propellantM = 0.0f, float dryM = 0.0f, float injectorDC = 0.0f, float pumpC = 0.0f, float turbineBRa = 0.0f, float turbineIT = 0.0f, float flowRC = 0.0f,
        float referenceA = 0.0f, float noseconeL = 0.0f, float bodyL = 0.0f, int numF = 0, float finRC = 0.0f, float finTC = 0.0f, float finS = 0.0f, float finT = 0.0f) : throatArea(throatA), exitArea(exitA), injectorArea(injectorA), heatRatio(heatRa), gasConstant(gasC), chamberTemp(chamberT),
        fuelDensity(fuelD), oxidizerDensity(oxidizerD), mixtureRatio(mixtureRa), shaftInertia(shaftI), pumpEfficiency(pumpEff), turbineEfficiency(turbineEff), tankPressure(tankP), propellantMass(propellantM), dryMass(dryM), injectorDischargeCoeff(injectorDC),
        pumpConstant(pumpC), turbineBleedRato(turbineBRa), turbineInletTemp(turbineIT), flowResistanceCoeff(flowRC), referenceArea(referenceA), noseconeLength(noseconeL), bodyLength(bodyL), numFins(numF), finRootChord(finRC), finTipChord(finTC), finSpan(finS),
        finThickness(finT) {
    
        characteristicVelocity = RocketMath::getCharacteristicVelocity(heatRatio, gasConstant, chamberTemp);
        shaftVelocity = 5000.0f;

        finenessRatio = (noseconeLength + bodyLength) / (2.0f * std::sqrt(referenceA / 3.14159f));
        dragCoeff = RocketMath::getDragCD(noseconeLength, bodyLength, referenceArea, finenessRatio,
            numFins, finRootChord, finTipChord, finSpan, finThickness);

        calculateEngineState(0.0f);
        updateForces();
    }

    void update(float dt) {
        float groundTempK = 32.0f + 273.15f;
        float airTempK = groundTempK;

        ambientPressure = RocketMath::getAmbientPressure(groundTempK, airTempK, height);
        airDensity = ambientPressure / (287.058f * airTempK);

        float soundSpeed = std::sqrt(1.4f * 287.058f * airTempK);
        mach = std::abs(velocity) / soundSpeed;

        if (propellantMass > 0.0f) {
            float totalMFR = massFlowRate + massFlowRateTurbine;
            float massToBurn = totalMFR * dt;
            propellantMass = std::max(0.0f, propellantMass - massToBurn);
        }
        else {
            propellantMass = 0.0f;
        }

        const int subSteps = 10;
        float subDt = dt / subSteps;

        for (int i = 0; i < subSteps; i++) {
            calculateEngineState(subDt);
        }

        dynamicDragCoeff = RocketMath::getWaveDragCD(mach, dragCoeff, finenessRatio);

        drag = 0.5f * airDensity * (velocity * velocity) * dynamicDragCoeff * referenceArea;
        updateForces();

        if (height <= 0.0f && netForce <= 0.0f) {
            netForce = 0.0f;
            velocity = 0.0f;
            height = 0.0f;
        }

        velocity += netForce / mass * dt;
        height += velocity * dt;

        if (height < 0.0f) {
            height = 0.0f;
            velocity = 0.0f;
        }

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
    const float getCHamberPressure() const { return chamberPressure; }
    const float getEffExhaustVelo() const { return thrust / massFlowRate; }
    const float getVelocity() const { return velocity; }
    const float getHeight() const { return height; }
    const float getMaxAltitude() const { return maxAltitude; }
    const float getHighestVelo() const { return highestVelo; }
    const float getDrag() const { return drag; }
    const float getDragCoeff() const { return dragCoeff; }
    const float getDynamicDragCoeff() const { return dynamicDragCoeff; }
    const float getMach() const { return mach; }
    const float getCharVelo() const { return characteristicVelocity; }
};