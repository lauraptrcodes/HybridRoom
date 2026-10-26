

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "Misc/ScopeLock.h"
#include "Convolver.h"
#include <atomic>
#include <memory>

/**
 * 
 */
class HYBRIDACOUSTICS_API HybridIrStore
{
public:
	static HybridIrStore& Get()
	{
		static HybridIrStore Instance;
		return Instance;
	}

	// Game-Thread
	void Publish(std::shared_ptr<const fdtd::PartitionedIr> Ir)
	{
		{
			FScopeLock Lock(&Cs);
			Current = MoveTemp(Ir);
		}
		Version.fetch_add(1, std::memory_order_release);
	}

	void SetGains(float Dry, float Wet)
	{
		DryGain.store(Dry, std::memory_order_relaxed);
		WetGain.store(Wet, std::memory_order_relaxed);
	}

	void Clear() { Publish(nullptr); }

	// Audio-Thread: Lock nur, wenn sich die Version geändert hat
	uint32 GetVersion() const { return Version.load(std::memory_order_acquire); }

	std::shared_ptr<const fdtd::PartitionedIr> GetCurrent()
	{
		FScopeLock Lock(&Cs);
		return Current;
	}

	float GetDryGain() const { return DryGain.load(std::memory_order_relaxed); }
	float GetWetGain() const { return WetGain.load(std::memory_order_relaxed); }

	static constexpr int32 BlockSize = 512; // Partitionsgröße = Latenz der Faltung in Samples

private:
	FCriticalSection Cs;
	std::shared_ptr<const fdtd::PartitionedIr> Current;
	std::atomic<uint32> Version{ 0 };
	std::atomic<float> DryGain{ 1.f };
	std::atomic<float> WetGain{ 0.5f };
};
