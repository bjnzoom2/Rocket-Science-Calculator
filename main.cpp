#include <iostream>
#include <iomanip>
#include <vector>

#include "waterRocket.h"
#include "solidRocket.h"

int main() {
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
	float dryMass = 0.47f; // kg
	float propellantDensity = 1889.0f; // kg/m3
	float coreRadius = 0.006f; // m
	float outerRadius = 0.018f; // m
	float grainLength = 0.15f; // m
	float burnRateCoeff = 0.0000082f;
	float throatArea = 0.00005f; // m2
	float pressureExponent = 0.32f;
	float exitArea = 0.000314f; // m2
	float referenceArea = 0.00125f; // m2
	float heatRatio = 1.13f;
	float chamberTemp = 1720.0f; // K
	float gasConstant = 198.0f; // J/(mol * K)
	float noseconeLength = 0.05f; // m
	float bodyLength = 0.15f; // m

	float propellantVolume = 3.14159f * grainLength * ((outerRadius * outerRadius) - (coreRadius * coreRadius));
	float propellantMass = propellantDensity * propellantVolume;

	SolidRocket solidRocket(dryMass, propellantMass, propellantDensity, coreRadius, outerRadius, grainLength, burnRateCoeff, throatArea, pressureExponent, seaLevelPa, seaLevelPa, exitArea, referenceArea, heatRatio, chamberTemp, gasConstant, noseconeLength, bodyLength);

	float dt = 0.001f;
	float time = 0.0f;

	float engineCutTime = 0.0f;
	bool engineCutoff = false;

	std::cout << "Time(s) | Thrust(N) | Accel(m/s2) | Velo(m/s) | Fuel(kg) | Height(m)\n";
	std::cout << "--------------------------------------------------------------------\n";

	while (solidRocket.getHeight() >= 0.0f || time < 0.05f) {
		solidRocket.update(dt);

		time += dt;

		if (std::fmod(time, 0.05f) < dt) {
			std::cout << std::fixed << std::setprecision(3)
				<< std::setw(7) << time << " | "
				<< std::setw(9) << solidRocket.getThrust() << " | "
				<< std::setw(11) << solidRocket.getAccel() << " | "
				<< std::setw(9) << solidRocket.getVelocity() << " | "
				<< std::setw(8) << solidRocket.getPropellantMass() << " | "
				<< std::setw(9) << solidRocket.getHeight() << "\n";
		}

		if (solidRocket.getPropellantMass() <= 0.0f && !engineCutoff) {
			engineCutTime = time;
			engineCutoff = true;
		}
	}

	std::cout << "\nTotal Burn Time: " << engineCutTime << " s\n";
	std::cout << "\nMax Altitude: " << solidRocket.getMaxAltitude() << " m\n";

	return 0;
}