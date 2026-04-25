#include <iostream>
#include <iomanip>

#include "rocket.h"

int main() {
	float seaLevelPa = 101325.0f;
	float exitArea = 0.00006362f;

	float dryMass = 0.028f + 0.0056f + 0.005f;
	WaterRocket waterRocket(1000, 400000, seaLevelPa, exitArea, dryMass, 0.4f, 0.001f, 0.4f, 0.00502f);

	float dt = 0.001f;
	float time = 0.0f;

	float engineCutTime = 0.0f;
	bool engineCutoff = false;

	std::cout << "Time(s) | Thrust(N) | Accel(m/s2) | Fuel(kg) | Height(m)\n";
	std::cout << "--------------------------------------------------------\n";

	while (waterRocket.getHeight() >= 0.0f || time < 0.1f) {
		waterRocket.update(dt);

		time += dt;

		if (std::fmod(time, 0.05f) < dt) {
			std::cout << std::fixed << std::setprecision(3)
				<< std::setw(7) << time << " | "
				<< std::setw(9) << waterRocket.getThrust() << " | "
				<< std::setw(11) << waterRocket.getAccel() << " | "
				<< std::setw(8) << waterRocket.getPropellantMass() << " | "
				<< std::setw(9) << waterRocket.getHeight() << "\n";
		}

		if (waterRocket.getPropellantMass() <= 0.0f && !engineCutoff) {
			engineCutTime = time;
			engineCutoff = true;
		}
	}

	std::cout << "\nTotal Burn Time: " << engineCutTime << " s\n";
	std::cout << "\nMax Altitude: " << waterRocket.getMaxAltitude() << " m\n";

	return 0;
}