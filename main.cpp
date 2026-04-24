#include <iostream>
#include "rocket.h"
#include "rocketMath.h"

int main() {
	float seaLevelPa = 101325.0f;
	float exitArea = 0.000616f;

	float exhaustVelo = RocketMath::getExhaustVeloLiquid(400000, seaLevelPa, 1000.0f);
	float mfr = RocketMath::getMFR(1000.0f, exitArea, exhaustVelo);

	float dryMass = 0.015f + 0.0056 + 0.005;
	Rocket waterRocket(mfr, exhaustVelo, seaLevelPa, seaLevelPa, exitArea, dryMass, 0.3f);

	std::cout << "Net Force: " << waterRocket.getNetForce() << " N\n";
	std::cout << "Acceleration: " << waterRocket.getAccel() << " m/s2\n";
	std::cout << "Burn Time: " << waterRocket.getBurnTime() << " s\n";

	return 0;
}