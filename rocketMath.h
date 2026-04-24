#pragma once
#include <cmath>

namespace RocketMath {
	float getExhaustVeloLiquid(float internalPa, float externalPa, float density) {
		float exhaustVeloSquared = 2 * (internalPa - externalPa) / density;
		return std::sqrt(exhaustVeloSquared);
	}

	float getMFR(float p, float A, float v) {
		return p * A * v;
	}
}