#pragma once
#include <cmath>
#include "rocketMath.h"

class LiquidRocket {
private:
    // Engine Variables
    float thrust = 0.0f; // N
    float massFlowRate = 0.0f; // kg/s
    float exhaustVelocity = 0.0f; // m/s
    float exhaustPressure = 0.0f; // Pa
    float ambientPressure = 0.0f; // Pa
    float throatArea = 0.0f; // m2
    float exitArea = 0.0f; // m2

    // Mass Variables
    float propellantMass = 0.0f;
    float dryMass = 0.0f;
    float mass = 0.0f;

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

    float chamberPressure = 0.0f; // Pa
    float characteristicVelocity = 0.0f; // m/s

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

    void calculateEngineState(float dt) {
        bulkDensity = RocketMath::getBulkPropellantDensity(fuelDensity, oxidizerDensity, mixtureRatio);
        dischargePressure = RocketMath::getPumpDischargePressure(tankPressure, pumpConstant, bulkDensity, shaftVelocity);

        chamberPressure = RocketMath::getChamberPressure(injectorDischargeCoeff, injectorArea, characteristicVelocity, throatArea, bulkDensity, dischargePressure);
        massFlowRate = RocketMath::getMFRLiquid(injectorDischargeCoeff, injectorArea, bulkDensity, dischargePressure, chamberPressure);

        float exitMach = RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio);
        exhaustPressure = RocketMath::getExhaustPressure(chamberPressure, heatRatio, exitMach);

        exhaustVelocity = RocketMath::getExhaustVeloLiquid(heatRatio, gasConstant, chamberTemp, exhaustPressure, chamberPressure);
        thrust = massFlowRate * exhaustVelocity + ((exhaustPressure - ambientPressure) * exitArea);
    }

    void updateForces() {

    }

public:
    LiquidRocket(float throatA = 0.0f, float exitA = 0.0f, float injectorA = 0.0f, float heatRa = 0.0f, float gasC = 0.0f, float chamberT = 0.0f,
        float fuelD = 0.0f, float oxidizerD = 0.0f, float mixtureRa = 0.0f, float shaftI = 0.0f, float pumpEff = 0.0f, float turbineEff = 0.0f, float tankP = 0.0f,
        float propellantM = 0.0f, float dryM = 0.0f, float injectorDC = 0.0f, float pumpC = 0.0f) : throatArea(throatArea), exitArea(exitA), injectorArea(injectorA), heatRatio(heatRa), gasConstant(gasC), chamberTemp(chamberT),
        fuelDensity(fuelD), oxidizerDensity(oxidizerD), mixtureRatio(mixtureRa), shaftInertia(shaftI), pumpEfficiency(pumpEff), turbineEfficiency(turbineEff), tankPressure(tankP), propellantMass(propellantM), dryMass(dryM), injectorDischargeCoeff(injectorDC),
        pumpConstant(pumpC) {
    
        characteristicVelocity = RocketMath::getCharacteristicVelocity(heatRatio, gasConstant, chamberTemp);
    }

    void update(float dt) {

    }
};