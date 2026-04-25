#include <iostream>
#include <iomanip>
#include <vector>

#include "waterRocket.h"

int main() {
	float seaLevelPa = 101325.0f;
	float exitArea = 0.00006362f;

	float stage1DryMass = 0.028f + 0.0056f;
	float stage2DryMass = 0.028f + 0.0056f + 0.005f;

	float stage2WaterVol = 0.4f / 1000.0f;
	float stage2AirVol = 0.001f - stage2WaterVol;

	float stage1CombinedDryMass = stage1DryMass + stage2DryMass + 0.6f;
	float stage1VirtualBM = 0.001f + stage2WaterVol;

	float dragCoeff = 0.4f;

	WaterRocket stage1(1000, 450000, seaLevelPa, exitArea, stage1CombinedDryMass, 0.6f, stage1VirtualBM, 0.0f, 0.00502f);
	WaterRocket stage2(1000, 450000, seaLevelPa, exitArea, stage2DryMass, 0.4f, 0.001f, dragCoeff, 0.00502f);
	std::vector<WaterRocket*> stages = { &stage1, &stage2 };

	WaterRocketMulti waterRocket(stages);

	float dt = 0.001f;
	float time = 0.0f;

	float engineCutTime = 0.0f;
	bool engineCutoff = false;

	std::cout << "Time(s) | Thrust(N) | Accel(m/s2) | Fuel(kg) | Height(m)\n";
	std::cout << "--------------------------------------------------------\n";

	while (waterRocket.getActive()->getHeight() >= 0.0f || time < 0.1f) {
		waterRocket.update(dt);

		time += dt;

		if (std::fmod(time, 0.05f) < dt) {
			std::cout << std::fixed << std::setprecision(3)
				<< std::setw(7) << time << " | "
				<< std::setw(9) << waterRocket.getActive()->getThrust() << " | "
				<< std::setw(11) << waterRocket.getActive()->getAccel() << " | "
				<< std::setw(8) << waterRocket.getActive()->getPropellantMass() << " | "
				<< std::setw(9) << waterRocket.getActive()->getHeight() << "\n";
		}

		if (waterRocket.getActive()->getPropellantMass() <= 0.0f && !engineCutoff) {
			engineCutTime = time;
			engineCutoff = true;
		}
	}

	std::cout << "\nTotal Burn Time: " << engineCutTime << " s\n";
	std::cout << "\nMax Altitude: " << waterRocket.getActive()->getMaxAltitude() << " m\n";

	return 0;
}