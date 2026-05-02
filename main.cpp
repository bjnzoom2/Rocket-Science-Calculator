#include <iostream>
#include <iomanip>

#include "waterRocket.h"
#include "solidRocket.h"
#include "liquidRocket.h"

#include "rocketMath.h"

#ifdef _WIN32
#include <windows.h>
#endif

int main() {
    #ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);
    #endif

    float seaLevelPa = 101325.0f;
    /*float exitArea = 0.00006362f;

    float stage1DryMass = 0.056f + 0.005f;
    float stage2DryMass = 0.056f + 0.0056f + 0.005f;

    float stage1PropellantMass = 1.2f;
    float stage2PropellantMass = 0.7f;

    float stage2WaterVol = stage2PropellantMass / 1000.0f;
    float stage2AirVol = 0.002f - stage2WaterVol;


    float stage1CombinedDryMass = stage1DryMass + stage2DryMass + stage2PropellantMass;
    float stage1VirtualBM = 0.002f + stage2WaterVol;

    float dragCoeff = 0.4f;

    WaterRocket stage1(1000, 450000, seaLevelPa, exitArea, stage1CombinedDryMass, stage1PropellantMass, stage1VirtualBM, 0.0f, 0.00916f);
    WaterRocket stage2(1000, 450000, seaLevelPa, exitArea, stage2DryMass, stage2PropellantMass, 0.002f, dragCoeff, 0.00916f);
    std::vector<WaterRocket*> stages = { &stage1, &stage2 };

    WaterRocketMulti waterRocket(stages);*/

/*    float casingMass = 13.1f;
    float airframeMass = 6.15f; // Fins, Nosecone and body
    float electronicsMass = 0.25f;
    float recoveryMass = 1.85f; // Parachute and cord
    float dryMass = casingMass + airframeMass + electronicsMass + recoveryMass;

    float propellantDensity = 1750.0f; // kg/m3
    float coreRadius = 0.003f; // m
    float outerRadius = 0.076f; // m
    float grainLength = 0.32f; // m
    float burnRateCoeff = 0.000008f; // Normal (0.000052)
    float throatArea = 0.00024f; // m2
    float pressureExponent = 0.41f;
    float exitArea = 0.115f; // m2
    float heatRatio = 1.22f;
    float chamberTemp = 2850.0f; // K
    float gasConstant = 290.0f; // J/(kg * K)
    float noseconeLength = 1.25f; // m
    float bodyLength = 4.5f; // m
    int numFins = 4;
    float finRootChord = 0.12f; // m
    float finTipChord = 0.02f; // m
    float finSpan = 0.04f; // m
    float finThickness = 0.004f; // m
    int numSegments = 8;

    float totalPropellantLength = numSegments * grainLength;
    float propellantVolume = 3.14159f * totalPropellantLength * ((outerRadius * outerRadius) - (coreRadius * coreRadius));
    float propellantMass = propellantDensity * propellantVolume;

    float referenceArea = 3.14159f * (outerRadius * outerRadius);

    SolidRocket solidRocket(dryMass, propellantMass, propellantDensity, coreRadius, outerRadius, grainLength, burnRateCoeff, throatArea, pressureExponent, seaLevelPa, seaLevelPa, exitArea, referenceArea, heatRatio, chamberTemp, gasConstant, noseconeLength, bodyLength,
        numFins, finRootChord, finTipChord, finSpan, finThickness, numSegments); */

    float engineMass = 90.0f;
    float tankageMass = 120.0f;
    float airframeMass = 80.0f;
    float avionicsRecoveryMass = 30.0f;
    float dryMass = engineMass + tankageMass + airframeMass + avionicsRecoveryMass;

    float fuelDensity = 810.0f;      // RP-1 (kg/m3)
    float oxidizerDensity = 1141.0f; // LOX (kg/m3)
    float mixtureRatio = 2.56f;      // O/F Ratio for RP-1/LOX
    float propellantMass = 500.0f;  // kg

    float throatArea = 0.004f;     // m2
    float exitArea = 0.033f;       // m2
    float injectorArea = 0.0025f;    // m2
    float heatRatio = 1.24f;         
    float gasConstant = 360.0f;      // J/(kg * K)
    float chamberTemp = 3500.0f;     // K

    float tankPressure = 500000.0f;  // 5 Bar (Pa)
    float pumpConstant = 0.000045f;
    float shaftInertia = 0.045f;     // kg*m2
    float pumpEfficiency = 0.70f;
    float turbineEfficiency = 0.75f;
    float turbineInletTemp = 850.0f; // K
    float turbineBleedRatio = 0.022f;
    float injectorDischargeCoeff = 0.85f;
    float flowResistanceCoeff = 0.05f;

    float noseconeLength = 1.5f;     // m
    float bodyLength = 6.5f;         // m
    float referenceArea = 0.0314f;   // m2
    int numFins = 4;
    float finRootChord = 0.25f;      // m
    float finTipChord = 0.10f;       // m
    float finSpan = 0.15f;           // m
    float finThickness = 0.006f;     // m

    LiquidRocket keroloxRocket(throatArea, exitArea, injectorArea, heatRatio, gasConstant, chamberTemp, fuelDensity, oxidizerDensity, mixtureRatio, shaftInertia, pumpEfficiency, turbineEfficiency, tankPressure, propellantMass, dryMass, injectorDischargeCoeff,
        pumpConstant, turbineBleedRatio, turbineInletTemp, flowResistanceCoeff, referenceArea, noseconeLength, bodyLength, numFins, finRootChord, finTipChord, finSpan, finThickness);

    float dt = 0.001f;
    float time = 0.0f;

    float engineCutTime = 0.0f;
    bool engineCutoff = false;

    std::cout << "Ideal Delta-V: " << RocketMath::getIdealDeltaV(keroloxRocket.getCharVelo() * 1.6f, dryMass + propellantMass, dryMass) << "m/s\n";
    std::cout << "TWR: " << keroloxRocket.getThrust() / ((dryMass + propellantMass) * 9.80665f) << "\n";
    std::cout << "Exit Mach: " << RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio) << "\n";

    std::cout << '\n';

    std::cout << "Start Program (y/n): ";
    std::string proceed = "";
    std::cin >> proceed;

    if (proceed != "y") return -1;
    std::cout << '\n';

    std::cout << "Time(s) | Thrust(N) | Accel(m/s2) | Velo(m/s) | Fuel(kg) | Height(m) | Mach | Drag\n";
    std::cout << "----------------------------------------------------------------------------------\n";

    while (keroloxRocket.getHeight() > 0.0f || time < 0.1f) {
        keroloxRocket.update(dt);
        time += dt;

        if (keroloxRocket.getPropellantMass() <= 0.0f && !engineCutoff) {
            engineCutTime = time;
            engineCutoff = true;
        }

        if (std::fmod(time, 0.05f) < dt) {
            std::cout << std::fixed << std::setprecision(3)
                << std::setw(7) << time << " | "
                << std::setw(9) << keroloxRocket.getThrust() << " | "
                << std::setw(11) << keroloxRocket.getAccel() << " | "
                << std::setw(9) << keroloxRocket.getVelocity() << " | "
                << std::setw(8) << keroloxRocket.getPropellantMass() << " | "
                << std::setw(9) << keroloxRocket.getHeight() << " | "
                << std::setw(4) << keroloxRocket.getMach() << " | "
                << std::setw(6) << keroloxRocket.getDrag() << "\n";
        }
    }

    std::cout << "\nTotal Burn Time: " << engineCutTime << " s\n";
    std::cout << "\nMax Altitude: " << keroloxRocket.getMaxAltitude() << " m\n";
    std::cout << "\nHighest Velocity: " << keroloxRocket.getHighestVelo() << " m/s\n";

    return 0;
}