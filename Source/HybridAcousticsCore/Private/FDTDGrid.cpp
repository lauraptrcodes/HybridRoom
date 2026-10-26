#include "FDTDGrid.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace fdtd {
	FDTDGrid::FDTDGrid(const SimulationConfig& cfg) : cfg_(cfg) {
		cfg_.dx = cfg_.c / (cfg_.fmax * cfg_.pointsPerLambda);

		// zeitschritt berechnen, 0.99 = 2nd Order
		const double cflFactor = 0.99 / std::sqrt(3.0);
		cfg_.dt = cfg_.dx * cflFactor / cfg_.c;

		const int pad = cfg_.pmlThickness;
		
		cfg_.Nx = static_cast<int>(std::lround(cfg_.Lx / cfg_.dx)) + 2 * pad + 2;
		cfg_.Ny = static_cast<int>(std::lround(cfg_.Ly / cfg_.dx)) + 2 * pad + 2;
		cfg_.Nz = static_cast<int>(std::lround(cfg_.Lz / cfg_.dx)) + 2 * pad + 2;
		//const size_t N = static_cast<size_t>(cfg_.Nx * cfg_.Ny * cfg_.Nz);
		const size_t N = static_cast<size_t>(cfg_.Nx) * cfg_.Ny * cfg_.Nz;
		for (auto& buf : p_)     buf.assign(N, 0.0);
		sigma_.assign(N, 0.0);
		//absorption_.assign(N, 1.0);
		wallMask_.assign(N, 1.0);
	}
	void FDTDGrid::setWallAbsorption(double alpha)
	{
		alpha = std::clamp(alpha, 0.0, 1.0);
		const double R = std::sqrt(1.0 - alpha);
		const double sig = (R < 1.0) ? -std::log(R) / cfg_.dt : 0.0;

		const int t = 2;  // 2-cell thick absorption layer inside rigid boundary
		for (int i = 0; i < cfg_.Nx; ++i)
			for (int j = 0; j < cfg_.Ny; ++j)
				for (int k = 0; k < cfg_.Nz; ++k) {
					bool nearWall = (i < t || i >= cfg_.Nx - t ||
						j < t || j >= cfg_.Ny - t ||
						k < t || k >= cfg_.Nz - t);
					if (nearWall)
						sigma_[idx(i, j, k)] += sig;
				}
	}
	void FDTDGrid::setupPML()
	{
		const int    d = cfg_.pmlThickness;
		const double dx = cfg_.dx;
		const double c = cfg_.c;
		// σ_max chosen so round-trip attenuation ≈ -80 dB
		const double sigMax = -3.0 * c * std::log(1e-4) / (2.0 * d * dx);

		auto profile = [&](int dist) -> double {
			if (dist >= d) return 0.0;
			double t = static_cast<double>(d - dist) / d;
			return sigMax * t * t;   // quadratic growth toward boundary
			};

		for (int i = 0; i < cfg_.Nx; ++i)
		for (int j = 0; j < cfg_.Ny; ++j)
		for (int k = 0; k < cfg_.Nz; ++k) {
			double s = profile(i) + profile(cfg_.Nx - 1 - i)
				+ profile(j) + profile(cfg_.Ny - 1 - j)
				+ profile(k) + profile(cfg_.Nz - 1 - k);
			sigma_[idx(i, j, k)] = s;
		}
	}

	void FDTDGrid::injectSource(int index, double value)
	{
		p_[now_][index] += value;
	}

	Vec3i FDTDGrid::toGrid(Vec3 posM) const {
		const int pad = cfg_.pmlThickness;
		return {
			static_cast<int>(std::lround(posM.x / cfg_.dx + 0.5)) + pad,
			static_cast<int>(std::lround(posM.y / cfg_.dx + 0.5)) + pad,
			static_cast<int>(std::lround(posM.z / cfg_.dx + 0.5)) + pad
		};
	}

	void FDTDGrid::setGlobalDamping(double T60)
	{
		if (T60 <= 0.0) return;
		const double sig = 6.0 * std::log(10.0) / T60;      // sigma = 6 ln10 / T60
		for (double& s : sigma_) s += sig;
	}

	// FDTDGrid.cpp
	int FDTDGrid::indexOf(Vec3 posM) const {
		const Vec3i g = toGrid(posM);
		const int lo = std::max(1, cfg_.pmlThickness + 1);          // erster Knoten im Raum
		auto inside = [&](int v, int n) { return std::clamp(v, lo, n - 1 - lo); };
		return idx(inside(g.x, cfg_.Nx), inside(g.y, cfg_.Ny), inside(g.z, cfg_.Nz));
	}

	void FDTDGrid::setWallRegion(Vec3 minCorner, Vec3 maxCorner) {
		// Statt sigma zu erhöhen: Zellen als Wand markieren (material = 1)
		// Diese Zellen werden in jedem Update auf 0 gesetzt
		auto gc0 = toGrid(minCorner);
		auto gc1 = toGrid(maxCorner);

		int x0 = std::clamp(gc0.x, 0, cfg_.Nx - 1);
		int y0 = std::clamp(gc0.y, 0, cfg_.Ny - 1);
		int z0 = std::clamp(gc0.z, 0, cfg_.Nz - 1);
		int x1 = std::clamp(gc1.x, 0, cfg_.Nx - 1);
		int y1 = std::clamp(gc1.y, 0, cfg_.Ny - 1);
		int z1 = std::clamp(gc1.z, 0, cfg_.Nz - 1);

		// Harte Wand: Druckwerte in dieser Region werden nach jedem Schritt auf 0 gesetzt
		int count = 0;
		for (int i = x0; i <= x1; ++i)
			for (int j = y0; j <= y1; ++j)
				for (int k = z0; k <= z1; ++k) {
					wallMask_[idx(i, j, k)] = 0.0;
					++count;
				}
		//std::cout << "[FDTDGrid] setAbsorptionRegion: " << count << " Zellen auf 0 gesetzt\n";
	}

	void FDTDGrid::step()
	{
		/*
		if (cfg_.scheme == Scheme::Order4)
			update4thOrder();
		else*/
		update2ndOrder();

		applyRigidBoundary();

		// Rotate buffer indices (zero allocation)
		int tmp = past_;
		past_ = now_;
		now_ = future_;
		future_ = tmp;
		++stepCount_;
	}

	void FDTDGrid::update2ndOrder()
	{
		const double lam = cfg_.c * cfg_.dt / cfg_.dx;
		const double r2 = lam * lam;
		const int Nx = cfg_.Nx, Ny = cfg_.Ny, Nz = cfg_.Nz;
		const int sx = Ny * Nz, sy = Nz, sz = 1;          // Schrittweiten passend zu idx()

		const auto& pn = p_[now_];
		const auto& pp = p_[past_];
		auto& pf = p_[future_];
		const auto& m = wallMask_;                       // 1 = Luft, 0 = Wand

		// Eine x-Ebene (i fest); schreibt nur in pf dieser Ebene -> parallel sicher
		auto slab = [&](int i) {
			for (int j = 1; j < Ny - 1; ++j)
				for (int k = 1; k < Nz - 1; ++k) {
					const int c0 = idx(i, j, k);
					if (m[c0] == 0.0) { pf[c0] = 0.0; continue; }   // Zelle liegt in einer Wand

					// Nachbar in einer Wand -> Term entfällt -> schallharte Innenwand
					const double pc = pn[c0];
					const double lap =
						m[c0 + sx] * (pn[c0 + sx] - pc) + m[c0 - sx] * (pn[c0 - sx] - pc) +
						m[c0 + sy] * (pn[c0 + sy] - pc) + m[c0 - sy] * (pn[c0 - sy] - pc) +
						m[c0 + sz] * (pn[c0 + sz] - pc) + m[c0 - sz] * (pn[c0 - sz] - pc);

					// zentrierte Dämpfung: stabil für jedes sigma
					const double a = 0.5 * sigma_[c0] * cfg_.dt;
					pf[c0] = (2.0 * pc - (1.0 - a) * pp[c0] + r2 * lap) / (1.0 + a); // *absorption_[c0];
				}
			};

		if (pfor_) pfor_(Nx - 2, [&](int n) { slab(n + 1); });   // i = 1 .. Nx-2
		else for (int i = 1; i < Nx - 1; ++i) slab(i);
	}

	void FDTDGrid::applyRigidBoundary() {
		auto& p = p_[future_];
		const int Nx = cfg_.Nx, Ny = cfg_.Ny, Nz = cfg_.Nz;

		for (int j = 0; j < Ny; ++j)
			for (int k = 0; k < Nz; ++k) {
				p[idx(0, j, k)] = p[idx(1, j, k)];
				p[idx(Nx - 1, j, k)] = p[idx(Nx - 2, j, k)];
			}
		for (int i = 0; i < Nx; ++i)
			for (int k = 0; k < Nz; ++k) {
				p[idx(i, 0, k)] = p[idx(i, 1, k)];
				p[idx(i, Ny - 1, k)] = p[idx(i, Ny - 2, k)];
			}
		for (int i = 0; i < Nx; ++i)
			for (int j = 0; j < Ny; ++j) {
				p[idx(i, j, 0)] = p[idx(i, j, 1)];
				p[idx(i, j, Nz - 1)] = p[idx(i, j, Nz - 2)];
			}
	}

	void FDTDGrid::reset()
	{
		for (auto& buf : p_) std::fill(buf.begin(), buf.end(), 0.0);
		stepCount_ = 0;
		past_ = 0; now_ = 1; future_ = 2;
	}
}