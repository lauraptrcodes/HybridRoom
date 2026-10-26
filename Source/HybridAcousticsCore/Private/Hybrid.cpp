
#include <algorithm>
#include <cmath>
#include <random>

#include "Hybrid.h"
#include "Constants.h"

namespace fdtd {

    // Biquad, transponierte Direktform II
    namespace {
        struct Biquad {
            double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
            double process(double x) {
                const double y = b0 * x + z1;
                z1 = b1 * x - a1 * y + z2;
                z2 = b2 * x - a2 * y;
                return y;
            }
        };

        // Butterworth 2. Ordnung (RBJ-Cookbook, Q = 1/sqrt(2))
        Biquad butterworth(double fc, int fs, bool highpass) {
            const double w0 = 2.0 * kPi * fc / fs;
            const double cw = std::cos(w0);
            const double alpha = std::sin(w0) / (2.0 * 0.7071067811865476);
            const double a0 = 1.0 + alpha;
            Biquad q;
            if (highpass) { q.b0 = (1 + cw) / 2; q.b1 = -(1 + cw); q.b2 = (1 + cw) / 2; }
            else { q.b0 = (1 - cw) / 2; q.b1 = (1 - cw); q.b2 = (1 - cw) / 2; }
            q.a1 = -2.0 * cw;
            q.a2 = 1.0 - alpha;
            q.b0 /= a0; q.b1 /= a0; q.b2 /= a0; q.a1 /= a0; q.a2 /= a0;
            return q;
        }

        // Linkwitz-Riley 4. Ordnung = zwei Butterworth-Stufen hintereinander;
        // Tief- und Hochpass addieren sich zu flachem Betragsfrequenzgang
        void linkwitzRiley4(std::vector<float>& x, double fc, int fs, bool highpass) {
            Biquad s1 = butterworth(fc, fs, highpass), s2 = butterworth(fc, fs, highpass);
            for (float& v : x) v = (float)s2.process(s1.process(v));
        }

        double rms(const std::vector<float>& x, int a, int b) {
            a = std::max(a, 0);
            b = std::min(b, (int)x.size());
            if (b <= a) return 0.0;
            double e = 0.0;
            for (int i = a; i < b; ++i) e += (double)x[i] * x[i];
            return std::sqrt(e / (b - a));
        }
    }
    

    float estimateT60(const float* ir, int length, int fs) {
        if (length < 2) return -1.f;
        std::vector<double> E(length);
        double acc = 0.0;
        for (int i = length - 1; i >= 0; --i) { acc += (double)ir[i] * ir[i]; E[i] = acc; }
        if (acc <= 0.0) return -1.f;

        int i5 = -1, i25 = -1;
        for (int i = 0; i < length; ++i) {
            const double L = 10.0 * std::log10(E[i] / E[0] + 1e-30);
            if (i5 < 0 && L < -5.0) i5 = i;
            if (L < -25.0) { i25 = i; break; }
        }
        if (i5 < 0 || i25 <= i5 + 1) return -1.f;

        // lineare Regression L(t) = k*t + b
        double st = 0, sL = 0, stt = 0, stL = 0;
        const int n = i25 - i5;
        for (int i = i5; i < i25; ++i) {
            const double t = (double)i / fs;
            const double L = 10.0 * std::log10(E[i] / E[0] + 1e-30);
            st += t; sL += L; stt += t * t; stL += t * L;
        }
        const double kSlope = (n * stL - st * sL) / (n * stt - st * st);   // dB/s
        return kSlope < 0.0 ? (float)(-60.0 / kSlope) : -1.f;
    }

    double schroederFrequency(double T60, double volume)
    {
        return 0.0;
    }

    std::vector<float> makeHybrid(const float* lowIr, int length, int fs, float fc, float T60,
        float directDelaySec, float tailGain, unsigned seed) {
        // 1) Tiefton: FDTD-IR, LR4-Tiefpass bei fc
        std::vector<float> low(lowIr, lowIr + length);
        linkwitzRiley4(low, fc, fs, false);

        // 2) Hochton: Rauschen * exp(-3 ln10 t / T60) ab Direktschall, LR4-Hochpass bei fc
        std::vector<float> tail(length, 0.f);
        std::mt19937 rng(seed);
        std::normal_distribution<float> nd(0.f, 1.f);
        const int n0 = std::clamp((int)(directDelaySec * fs), 0, length);
        const double k = 3.0 * std::log(10.0) / T60;
        for (int n = n0; n < length; ++n)
            tail[n] = nd(rng) * (float)std::exp(-k * (n - n0) / fs);
        linkwitzRiley4(tail, fc, fs, true);

        // 3) Pegelanpassung (Heuristik): gleicher RMS im Fenster 20..100 ms nach Direktschall
        const int wa = n0 + fs / 50, wb = n0 + fs / 10;
        const double rTail = rms(tail, wa, wb);
        const float g = rTail > 1e-12 ? (float)(tailGain * rms(low, wa, wb) / rTail) : 0.f;

        for (int n = 0; n < length; ++n) low[n] += g * tail[n];
        return low;
    }

    void applyDecay(float* ir, int length, int fs, float T60) {
        if (T60 <= 0.f) return;
        const double k = 3.0 * std::log(10.0) / T60;     // = sigma/2
        for (int n = 0; n < length; ++n)
            ir[n] *= (float)std::exp(-k * n / fs);
    }
}