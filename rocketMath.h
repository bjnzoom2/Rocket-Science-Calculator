#pragma once
#include <cmath>

namespace RocketMath {
	float getMFR(float p, float A, float v) {
		return p * A * v;
	}

	float getMFRSolid(float p, float A, float r) {
		return p * A * r;
	}

	float getBurnRateSolid(float a, float P, float n) {
		return a * std::pow(P, n);
	}

	float getExhaustVeloLiquid(float internalPa, float externalPa, float density) {
		float exhaustVeloSquared = 2.0f * (internalPa - externalPa) / density;
		return std::sqrt(exhaustVeloSquared);
	}

	float getExhaustVeloSolid(float heatRatio, float MachE, float gasConstant, float exitTemp) {
		return MachE * std::sqrt(heatRatio * gasConstant * exitTemp);
	}

	float getExitTempSolid(float Tc, float heatRatio, float MachE) {
		return Tc / (1.0f + (heatRatio - 1) / 2 * (MachE * MachE));
	}

	float getBoyleLawPressure(float initVolume, float initPressure, float currentVolume) {
		if (currentVolume <= 0.000001f) return 0.0f;
		return (initVolume * initPressure) / currentVolume;
	}

	float getExhaustPressureSolid(float Pc, float heatRatio, float MachE) {
		return Pc * std::pow(1.0f + ((heatRatio - 1.0f) / 2.0f) * (MachE * MachE), -(heatRatio / (heatRatio - 1)));
	}

	float getCombustionPressure(float burnA, float throatA, float propellantDensity, float a, float n, float G) {
		return std::pow((burnA / throatA) * (propellantDensity * a / G), 1.0f / (1.0f - n));
	}

	float getGasFlowabitilty(float chamberTemp, float gasConstant, float heatRatio) {
		float vp = std::sqrt(heatRatio * (std::pow(2.0f / (heatRatio + 1.0f), (heatRatio + 1.0f) / (heatRatio - 1.0f))));
		return vp / std::sqrt(gasConstant * chamberTemp);
	}

	float getExitMachApproximation(float epsilon, float gamma) {
		if (epsilon <= 1.001f) return 1.0f;

		float g = gamma;
		float p = (g + 1.0f) / (2.0f * (g - 1.0f));
		float a = std::pow((g + 1.0f) / 2.0f, -p);

		float m_est = std::sqrt(2.0f / (g + 1.0f) * std::pow(epsilon, (g - 1.0f)));

		float term1 = (g - 1.0f) / 2.0f;
		float machSq = (std::pow(epsilon / a, 1.0f / p) - 1.0f) / term1;

		return std::sqrt(std::max(1.0f, machSq));
	}

	float getProgressiveBurnArea(float currentCR, float outerR, float grainL) {
		if (currentCR >= outerR) return 0.0f;
		return 2.0f * 3.14159f * currentCR * grainL;
	}

	float getRegressiveBurnArea(float R) {
		return 3.14159f * (R * R);
	}

	float getDiameter(float referenceArea) {
		return 2.0f * std::sqrt(referenceArea / 3.14159f);
	}

	float getDragCD(float noseLength, float bodyLength, float referenceArea, float& finenessRatio) {
		float totalLength = noseLength + bodyLength;
		float diameter = getDiameter(referenceArea);

		float noseSurface = 3.14159f * (diameter / 2.0f) * std::sqrt(std::pow(diameter / 2.0f, 2) + std::pow(noseLength, 2));
		float bodySurface = 3.14159f * diameter * bodyLength;
		float wettedArea = noseSurface + bodySurface;

		float Cf = 0.003f;

		finenessRatio = totalLength / diameter;
		float formFactor = 1.0f + (1.5f / std::pow(finenessRatio, 1.5f)) + (0.5f / finenessRatio);

		float skinDrag = Cf * (wettedArea / referenceArea) * formFactor;

		return skinDrag + 0.1f;
	}

	float getWaveDragCD(float mach, float baseCD, float finenessRatio) {
		float peakMagnitude = 1.0f / std::sqrt(finenessRatio);

		if (mach < 0.8f) {
			return baseCD;
		}

		if (mach < 1.05f) {
			float t = (mach - 0.8f) / 0.25f;
			float smoothStep = t * t * (3.0f - 2.0f * t);
			return baseCD + (smoothStep * peakMagnitude);
		}

		float beta = std::sqrt(mach * mach - 1.0f);
		float supersonicCD = (peakMagnitude * 1.2f) / beta;

		return std::max(baseCD, supersonicCD);
	}
}