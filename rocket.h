#pragma once

class Rocket {
private:
	float thrust = 0.0f;
	float massFlowRate = 0.0f;
	float exitVelocity = 0.0f;
	float exitPressure = 0.0f;
	float ambientPressure = 0.0f;
	float exitArea = 0.0f;
public:
	Rocket(float mfr, float ev, float ep, float ap, float eA) : massFlowRate(mfr), exitVelocity(ev), exitPressure(ep), ambientPressure(ap), exitArea(eA) {
		thrust = (massFlowRate * exitVelocity) + (exitPressure - ambientPressure) * exitArea;
	}
};