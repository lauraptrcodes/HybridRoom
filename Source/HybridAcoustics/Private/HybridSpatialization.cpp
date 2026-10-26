
#include "HybridSpatialization.h"
#include "HybridIrStore.h"

namespace
	{
	// EmitterPosition ist relativ zum Hörer; +Y wird als "rechts" angenommen.
		// Kommt der Sound von der falschen Seite, hier -1.f eintragen.
		constexpr float kPanSign = 1.f;
	}

void HybridSpatialization::Initialize(const FAudioPluginInitializationParams InitParams)
{
    Sources.SetNum(InitParams.NumSources);
    InitialBufferLength = InitParams.BufferLength;
    for (FSourceState& S : Sources)
    {
        S.Mono.SetNumZeroed(InitialBufferLength);
        S.Wet.SetNumZeroed(InitialBufferLength);
    }
}

void HybridSpatialization::OnReleaseSource(const uint32 SourceId)
{
    if (Sources.IsValidIndex(SourceId) && Sources[SourceId].Conv)
    {
        Sources[SourceId].Conv->reset(); // nächste Quelle in diesem Slot startet ohne Nachhall-Rest
    }
	if (!Sources.IsValidIndex(SourceId)) return;
	FSourceState & S = Sources[SourceId];
	if (S.Conv) S.Conv->reset();
	S.bGainsInit = false;
}

void HybridSpatialization::ProcessAudio(const FAudioPluginSourceInputData& In, FAudioPluginSourceOutputData& Out)
{
	const int32 InCh = FMath::Max(1, In.NumChannels);
	const int32 Frames = In.AudioBuffer->Num() / InCh;
	const int32 OutCh = Frames > 0 ? Out.AudioBuffer.Num() / Frames : 2;
	if (Frames == 0 || !Sources.IsValidIndex(In.SourceId)) return;

	FSourceState& S = Sources[In.SourceId];
	HybridIrStore& Store = HybridIrStore::Get();

	// 1) Neue IR übernehmen (nur wenn sich die Version geändert hat)
	const uint32 Version = Store.GetVersion();
	if (Version != S.IrVersion)
	{
		S.IrVersion = Version;
		std::shared_ptr<const fdtd::PartitionedIr> Ir = Store.GetCurrent();
		if (Ir)
		{
			// Einmalige Allokation pro Quellen-Slot (Demo-Abkürzung, siehe Hinweise)
			if (!S.Conv) S.Conv = std::make_unique<fdtd::Convolver>(HybridIrStore::BlockSize);
			S.Conv->setIr(MoveTemp(Ir));
		}
		else
		{
			S.Conv.reset();   // unique_ptr::reset -> Convolver freigeben; danach nur noch Dry
		}
	}

	// 2) Mono-Downmix
	if (S.Mono.Num() < Frames) { S.Mono.SetNumZeroed(Frames); S.Wet.SetNumZeroed(Frames); }
	const float* Src = In.AudioBuffer->GetData();
	for (int32 i = 0; i < Frames; ++i)
	{
		float Sum = 0.f;
		for (int32 c = 0; c < InCh; ++c) Sum += Src[i * InCh + c];
		S.Mono[i] = Sum / (float)InCh;
	}

	// 3) Faltung
	if (S.Conv) S.Conv->process(S.Mono.GetData(), S.Wet.GetData(), Frames);
	else FMemory::Memzero(S.Wet.GetData(), Frames * sizeof(float));

	/* 4) Dry + Wet auf alle Ausgangskanäle(noch ohne Richtung->HRTF kommt später)
	const float Dry = Store.GetDryGain(), WetG = Store.GetWetGain();
	float* Dst = Out.AudioBuffer.GetData();
	for (int32 i = 0; i < Frames; ++i)
	{
		const float V = Dry * S.Mono[i] + WetG * S.Wet[i];
		for (int32 c = 0; c < OutCh; ++c) Dst[i * OutCh + c] = V;
	}*/
	// 4) Richtung des Direktschalls: Equal-Power-Panning aus der hörerrelativen Position
	float TargetL = 0.70710678f, TargetR = 0.70710678f;
	if (In.SpatializationParams){
		const FVector Rel = In.SpatializationParams->EmitterPosition;
		const double Len = Rel.Size();
		if (Len > UE_KINDA_SMALL_NUMBER){
			const float Pan = FMath::Clamp((float)(kPanSign * Rel.Y / Len), -1.f, 1.f); // -1 links, +1 rechts
			const float Theta = (Pan + 1.f) * 0.25f * UE_PI;                              // 0 .. pi/2
			TargetL = FMath::Cos(Theta);
			TargetR = FMath::Sin(Theta);
		}
	}
	if (!S.bGainsInit) { S.GainL = TargetL; S.GainR = TargetR; S.bGainsInit = true; }
	
		// 5) Ausgabe: Dry mit Richtung, Wet (Nachhall) auf beiden Ohren gleich;
		//    Panning-Pegel über den Block überblenden -> kein Knacken bei Bewegung
	const float Dry = Store.GetDryGain(), WetG = Store.GetWetGain();
	float* Dst = Out.AudioBuffer.GetData();
	for (int32 i = 0; i < Frames; ++i){
		const float T = (float)(i + 1) / (float)Frames;
		const float gL = S.GainL + (TargetL - S.GainL) * T;
		const float gR = S.GainR + (TargetR - S.GainR) * T;
		const float W = WetG * S.Wet[i];
		
			if (OutCh >= 2){
				Dst[i * OutCh + 0] = Dry * gL * S.Mono[i] + W;
				Dst[i * OutCh + 1] = Dry * gR * S.Mono[i] + W;
				for (int32 c = 2; c < OutCh; ++c) Dst[i * OutCh + c] = 0.f;
			}
			else{
				Dst[i] = Dry * S.Mono[i] + W;
			}
	}
	S.GainL = TargetL;
	S.GainR = TargetR;
}
