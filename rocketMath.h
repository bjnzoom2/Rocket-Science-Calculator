#pragma once
#include <cmath>

namespace RocketMath {
	float getMFR(float p, float A, float v) {
		return p * A * v;
	}

	float getMFRSolid(float p, float A, float r) {
		return p * A * r;
	}

	float getMFRLiquid(float injectorDischargeCoeff, float injectorArea, float bulkDensity, float dischargePressure, float chamberPressure) {
		return injectorDischargeCoeff * injectorArea * std::sqrt(2.0f * bulkDensity * (dischargePressure - chamberPressure));
	}

	float getBurnRateSolid(float a, float P, float n) {
		return a * std::pow(P, n);
	}

	float getExhaustVeloWater(float internalPa, float externalPa, float density) {
		float exhaustVeloSquared = 2.0f * (internalPa - externalPa) / density;
		return std::sqrt(exhaustVeloSquared);
	}

	float getExhaustVeloSolid(float heatRatio, float MachE, float gasConstant, float exitTemp) {
		return MachE * std::sqrt(heatRatio * gasConstant * exitTemp);
	}

	float getExhaustVeloLiquid(float heatRatio, float gasConstant, float chamberTemp, float exhaustPressure, float chamberPressure) {
		float x = (2.0f * heatRatio / (heatRatio - 1.0f)) * gasConstant * chamberTemp;
		float y = 1.0f - std::pow(exhaustPressure / chamberPressure, (heatRatio - 1.0f) / heatRatio);

		return std::sqrt(x * y);
	}

	float getExitTempSolid(float Tc, float heatRatio, float MachE) {
		return Tc / (1.0f + (heatRatio - 1) / 2 * (MachE * MachE));
	}

	float getBoyleLawPressure(float initVolume, float initPressure, float currentVolume) {
		if (currentVolume <= 0.000001f) return 0.0f;
		return (initVolume * initPressure) / currentVolume;
	}

	float getExhaustPressure(float Pc, float heatRatio, float MachE) {
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

	float getBATESBurnArea(float currentCR, float grainL, int numSeg, float outerR) {
		float areaCore = 2.0f * 3.14159f * currentCR * grainL * numSeg;
		float areaEnds = 2.0f * numSeg * 3.14159f * (pow(outerR, 2.0f) - pow(currentCR, 2.0f));
		float totalArea = areaCore + areaEnds;

		return totalArea;
	}

	float getDiameter(float referenceArea) {
		return 2.0f * std::sqrt(referenceArea / 3.14159f);
	}

	float getDragCD(float noseLength, float bodyLength, float referenceArea, float finenessRatio, int numFins, float finRootChord, float finTipChord, float finSpan, float finThickness) {
		float totalLength = noseLength + bodyLength;
		float diameter = getDiameter(referenceArea);

		float noseSurface = 3.14159f * (diameter / 2.0f) * std::sqrt(std::pow(diameter / 2.0f, 2.0f) + std::pow(noseLength, 2.0f));
		float bodySurface = 3.14159f * diameter * bodyLength;
		float wettedArea = noseSurface + bodySurface;

		float Cf = 0.003f;

		float formFactor = 1.0f + (1.5f / std::pow(finenessRatio, 1.5f)) + (0.5f / finenessRatio);
		float skinDrag = Cf * (wettedArea / referenceArea) * formFactor;

		float singleFinArea = 0.5f * (finRootChord + finTipChord) * finSpan;

		float finWettedArea = 2.0f * singleFinArea * numFins;
		float meanChord = (finRootChord + finTipChord) / 2.0f;

		float finFormFactor = 1.0f + (60.0f * std::pow(finThickness / meanChord, 4.0f)) + (0.8f * (finThickness / meanChord));

		float interferenceFactor = 1.0f + (diameter / (2.0f * finSpan));

		float finDrag = Cf * (finWettedArea / referenceArea) * finFormFactor * interferenceFactor;

		return 0.1f + skinDrag + finDrag;
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
		float supersonicCD = (peakMagnitude * 1.5f) / beta;

		return std::max(baseCD * 1.1f, std::min(supersonicCD, peakMagnitude * 1.2f));
	}

	float getIdealDeltaV(float initExhaustVelo, float initMass, float dryMass) {
		return initExhaustVelo * std::log(initMass / dryMass);
	}

	float getCharacteristicVelocity(float heatRatio, float gasConstant, float chamberTemp) {
		float chamberSpeedOfSound = std::sqrt(heatRatio * gasConstant * chamberTemp);
		float vf = heatRatio * std::sqrt(std::pow(2.0f / (heatRatio + 1), (heatRatio + 1) / (heatRatio - 1)));

		return chamberSpeedOfSound / vf;
	}

	float getBulkPropellantDensity(float fuelDensity, float oxidizerDensity, float mixtureRatio) {
		return (mixtureRatio + 1.0f) / ((1.0f / fuelDensity) + (mixtureRatio / oxidizerDensity));
	}

	float getPumpDischargePressure(float tankPressure, float pumpConstant, float bulkDensity, float shaftVelocity) {
		return tankPressure + (pumpConstant * bulkDensity * (shaftVelocity * shaftVelocity));
	}
	
	float getChamberPressure(float injectorDischargeCoeff, float injectorArea, float characteristicVelo, float throatArea, float bulkDensity, float dischargePressure) {
		float flowResistanceConstant = 2.0f * bulkDensity * ((injectorDischargeCoeff * injectorArea * characteristicVelo / throatArea) * (injectorDischargeCoeff * injectorArea * characteristicVelo / throatArea));
		return (-flowResistanceConstant + std::sqrt(flowResistanceConstant * flowResistanceConstant + 4.0f * flowResistanceConstant * dischargePressure)) / 2.0f;
	}

	float getPumpPowerReq(float mfr, float dischargePressure, float tankPressure, float bulkDensity, float pumpEfficiency) {
		return (mfr * (dischargePressure - tankPressure)) / (bulkDensity * pumpEfficiency);
	}

	float getSHCP(float heatRatio, float gasConstant) {
		return heatRatio * gasConstant / (heatRatio - 1);
	}

	float getFrictionLoss(float flowResistanceCoeff, float bulkDensity, float massFlowRateTurbine) {
		return (flowResistanceCoeff * (massFlowRateTurbine * massFlowRateTurbine)) / (2.0f * bulkDensity);
	}

	float getTurbinePower(float mfrTurbine, float SHCP, float turbineInlentTemp, float P_in, float P_out, float heatRatio, float turbineEfficiency) {
		if (P_in <= P_out || P_in <= 0.0f) {
			return 0.0f;
		}
		return mfrTurbine * SHCP * turbineInlentTemp * (1.0f - std::pow(P_out / P_in, (heatRatio - 1.0f) / heatRatio)) * turbineEfficiency;
	}

	float getShaftVeloChange(float shaftTorque, float shaftInertia, float dt) {
		float accel = shaftTorque / shaftInertia;
		return accel * dt;
	}

	float getAmbientPressure(float groundTempK, float& airTempK, float height) {
		float ambientPressure = 0.0f;
		if (height <= 11000.0f) { // Troposphere
			airTempK = 288.15f - (0.0065f * height);
			ambientPressure = 101325.0f * std::pow((airTempK / 288.15f), 5.25588f);
		}
		else if (height <= 20000.0f) { // Lower Stratosphere
			airTempK = 216.65f;
			ambientPressure = 22632.10f * std::exp(-0.000157696f * (height - 11000.0f));
		}
		else if (height <= 32000.0f) { // Upper Stratosphere
			airTempK = 216.65f + (0.001f * (height - 20000.0f));
			ambientPressure = 5474.89f * std::pow((airTempK / 216.65f), -34.1632f);
		}
		else if (height <= 47000.0f) { // Stratopause
			airTempK = 228.65f + (0.0028f * (height - 32000.0f));
			ambientPressure = 868.02f * std::pow((airTempK / 228.65f), -12.2011f);
		}
		else if (height <= 51000.0f) { // Lower Mesosphere
			airTempK = 270.65f;
			ambientPressure = 110.91f * std::exp(-0.0001262f * (height - 47000.0f));
		}
		else if (height <= 71000.0f) { // Upper Mesosphere
			airTempK = 270.65f - (0.0028f * (height - 51000.0f));
			ambientPressure = 66.94f * std::pow((airTempK / 270.65f), 12.2011f);
		}
		else if (height <= 84852.0f) { // Mesopause
			airTempK = 214.65f - (0.002f * (height - 71000.0f));
			ambientPressure = 3.96f * std::pow((airTempK / 214.65f), 17.0816f);
		}
		else { // Exosphere / Space
			airTempK = 186.87f;
			ambientPressure = 0.00001f;
		}
		return std::max(0.0f, ambientPressure);
	}
}