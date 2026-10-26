

#pragma once

#include "CoreMinimal.h"
#include <IAudioExtensionPlugin.h>
#include <memory>
#include "Convolver.h"
/**
 *  HYBRIDACOUSTICS_API 
 */
class HybridSpatialization : public IAudioSpatialization
{
public:
    HybridSpatialization() = default;
    HybridSpatialization(const HybridSpatialization&) = delete;
    HybridSpatialization & operator=(const HybridSpatialization&) = delete;
    virtual void Initialize(const FAudioPluginInitializationParams InitParams) override;
    virtual void OnReleaseSource(const uint32 SourceId) override;
    virtual void ProcessAudio(const FAudioPluginSourceInputData& In,
        FAudioPluginSourceOutputData& Out) override;

private:
    struct FSourceState
    {
        std::unique_ptr<fdtd::Convolver> Conv;   // std::unique_ptr: vollständiger Typ, kein Pimpl-Problem
        uint32 IrVersion = 0;
        TArray<float> Mono;
        TArray<float> Wet;
        float GainL = 0.70710678f;   // Panning-Pegel des letzten Blocks (für Überblendung)
        float GainR = 0.70710678f;
        bool  bGainsInit = false;
    };

    TArray<FSourceState> Sources;
    int32 InitialBufferLength = 1024;
};

class FHybridSpatializationFactory : public IAudioSpatializationFactory
{
public:
    virtual FString GetDisplayName() override { return TEXT("Hybrid Acoustics"); }
    virtual bool SupportsPlatform(const FString& PlatformName) override { return PlatformName == TEXT("Windows"); }
    virtual TAudioSpatializationPtr CreateNewSpatializationPlugin(FAudioDevice* OwningDevice) override
    {
        return TAudioSpatializationPtr(new HybridSpatialization());
    }
};
