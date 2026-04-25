#pragma once
#include <cmath>

namespace RocketMath {
	float getExhaustVeloLiquid(float internalPa, float externalPa, float density) {
		float exhaustVeloSquared = 2.0f * (internalPa - externalPa) / density;
		return std::sqrt(exhaustVeloSquared);
	}

	float getBoyleLawPressure(float initVolume, float initPressure, float currentVolume) {
		if (currentVolume <= 0.000001f) return 0.0f;
		return (initVolume * initPressure) / currentVolume;
	}

	float getMFR(float p, float A, float v) {
		return p * A * v;
	}
}