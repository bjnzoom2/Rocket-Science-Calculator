#include <iostream>
#include <iomanip>

#include "waterRocket.h"
#include "solidRocket.h"
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

    float casingMass = 1.8f;
    float airframeMass = 1.5f; // Tube + Fins + Nosecone
    float electronicsMass = 0.15f; // Altimeter + Battery
    float recoveryMass = 0.5f; // Parachute + Cord
    float dryMass = casingMass + airframeMass + electronicsMass + recoveryMass;

    float propellantDensity = 1750.0f; // kg/m3
    float coreRadius = 0.006f; // m
    float outerRadius = 0.038f; // m
    float grainLength = 0.16f; // m
    float burnRateCoeff = 0.000022f; // Normal (0.000052)
    float throatArea = 0.0004f; // m2
    float pressureExponent = 0.41f;
    float exitArea = 0.0075f; // m2
    float referenceArea = 0.004536f; // m2
    float heatRatio = 1.22f;
    float chamberTemp = 2850.0f; // K
    float gasConstant = 290.0f; // J/(kg * K)
    float noseconeLength = 0.25f; // m
    float bodyLength = 1.3f; // m
    int numFins = 4;
    float finRootChord = 0.05f; // m
    float finTipChord = 0.00f; // m
    float finSpan = 0.05f; // m
    float finThickness = 0.002f; // m
    int numSegments = 4;

    float totalPropellantLength = numSegments * grainLength;
    float propellantVolume = 3.14159f * totalPropellantLength * ((outerRadius * outerRadius) - (coreRadius * coreRadius));
    float propellantMass = propellantDensity * propellantVolume;

    SolidRocket solidRocket(dryMass, propellantMass, propellantDensity, coreRadius, outerRadius, grainLength, burnRateCoeff, throatArea, pressureExponent, seaLevelPa, seaLevelPa, exitArea, referenceArea, heatRatio, chamberTemp, gasConstant, noseconeLength, bodyLength,
        numFins, finRootChord, finTipChord, finSpan, finThickness, numSegments);

    float dt = 0.001f;
    float time = 0.0f;

    float engineCutTime = 0.0f;
    bool engineCutoff = false;

    std::cout << "Ideal Delta-V: " << RocketMath::getIdealDeltaV(solidRocket.getEffExhaustVelo(), dryMass + propellantMass, dryMass) << "m/s\n";
    std::cout << "TWR: " << solidRocket.getThrust() / ((dryMass + propellantMass) * 9.80665f) << "\n";
    std::cout << "Exit Mach: " << RocketMath::getExitMachApproximation(exitArea / throatArea, heatRatio) << "\n";

    std::cout << '\n';

    std::cout << "Start Program (y/n): ";
    std::string proceed = "";
    std::cin >> proceed;

    if (proceed != "y") return -1;
    std::cout << '\n';

    std::cout << "Time(s) | Thrust(N) | Accel(m/s2) | Velo(m/s) | Fuel(kg) | Height(m) | Mach | Drag\n";
    std::cout << "----------------------------------------------------------------------------------\n";

    while (solidRocket.getHeight() >= 0.0f) {
        solidRocket.update(dt);
        time += dt;

        if (solidRocket.getPropellantMass() <= 0.0f && !engineCutoff) {
            engineCutTime = time;
            engineCutoff = true;
        }

        if (std::fmod(time, 0.05f) < dt) {
            std::cout << std::fixed << std::setprecision(3)
                << std::setw(7) << time << " | "
                << std::setw(9) << solidRocket.getThrust() << " | "
                << std::setw(11) << solidRocket.getAccel() << " | "
                << std::setw(9) << solidRocket.getVelocity() << " | "
                << std::setw(8) << solidRocket.getPropellantMass() << " | "
                << std::setw(9) << solidRocket.getHeight() << " | "
                << std::setw(4) << solidRocket.getMach() << " | "
                << std::setw(6) << solidRocket.getDrag() << "\n";
        }
    }

    std::cout << "\nTotal Burn Time: " << engineCutTime << " s\n";
    std::cout << "\nMax Altitude: " << solidRocket.getMaxAltitude() << " m\n";
    std::cout << "\nHighest Velocity: " << solidRocket.getHighestVelo() << " m/s\n";

    return 0;
}