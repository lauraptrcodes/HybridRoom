#pragma once
#include "Export.h"
#include <vector>

namespace fdtd {

	FDTD_API float estimateT60(const float* ir, int length, int fs);
	FDTD_API double schroederFrequency(double T60, double volume);

	// Hybrid-IR: LR4-Tiefpass(FDTD-IR) + LR4-Hochpass(synthetischer Nachhall), Übergang bei fc.
	// directDelaySec = Laufzeit des Direktschalls (Abstand / c); ab dort beginnt der Rauschnachhall.
	FDTD_API std::vector<float> makeHybrid(const float* lowIr, int length, int fs, float fc, float T60,
		float directDelaySec, float tailGain = 1.0f, unsigned seed = 1);

	FDTD_API void applyDecay(float* ir, int length, int fs, float T60);
}
