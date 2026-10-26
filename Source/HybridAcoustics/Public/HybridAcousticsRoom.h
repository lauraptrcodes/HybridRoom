

#pragma once
#include "Types.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HybridAcousticsRoom.generated.h"

class UBoxComponent;
struct FHybridBakeResult;

UCLASS()
class HYBRIDACOUSTICS_API AHybridAcousticsRoom : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AHybridAcousticsRoom();

	// ================= CONFIG =================
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Hybrid Acoustics")
	TObjectPtr<UBoxComponent> RoomBounds;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Hybrid Acoustics")
	TObjectPtr<USceneComponent> SourcePoint;

	// Optional: Bake-Quelle = Position dieses Actors (z. B. dein Ambient Sound)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics")
	TObjectPtr<AActor> SoundSource;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics")
	bool bDrawDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics")
	bool bBakeOnBeginPlay = true;

	// Übergangsfrequenz FDTD <-> synthetischer Nachhall (Mock statt 2000*sqrt(T60/V))
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "50", ClampMax = "1000"))
	float MockSchroederHz = 200.f;

	// FDTD rechnet bis FmaxFactor * MockSchroederHz, damit der Übergang sauber aufgelöst ist
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "1.0", ClampMax = "4.0"))
	float FmaxFactor = 2.f;

	// Gitterpunkte pro Wellenlänge bei fmax
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "6.0", ClampMax = "20.0"))
	float PointsPerLambda = 10.f;

	// Absorptionsgrad der Wände (0 = schallhart, 1 = vollständig absorbierend)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WallAlpha = 0.17f;

	// Wird für den HF-Nachhall verwendet, falls T60 nicht aus der FDTD-IR gemessen werden kann
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "0.1", ClampMax = "5.0"))
	float FallbackT60 = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "0.1", ClampMax = "3.0"))
	float IrLengthSec = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "10"))
	float ReceiverSpacingCm = 50.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "0.0", ClampMax = "2.0"))
	float DryGain = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hybrid Acoustics", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float WetGain = 0.5f;

	UFUNCTION(BlueprintCallable, Category = "Hybrid Acoustics")
	void BakeAsync();
	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	//std::vector<fdtd::SourceDesc>   sources_;
	//std::vector<fdtd::ReceiverDesc> receivers_;

	FVector WorldToRoomMeters(const FVector& World) const;
	FVector RoomMetersToWorld(const FVector& Meters) const;
	float GetMeasuredT60() const;
	void PublishReceiver(int32 Receiver);

	TSharedPtr<FHybridBakeResult, ESPMode::ThreadSafe> Bake;
	std::atomic<bool> bBaking{ false };
	int32 CurrentReceiver = INDEX_NONE;

	// Mit dem Worker-Thread geteilt; überleben den Actor, falls der Bake noch läuft
	TSharedRef<std::atomic<bool>, ESPMode::ThreadSafe>  CancelFlag = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
	TSharedRef<std::atomic<float>, ESPMode::ThreadSafe> Progress = MakeShared<std::atomic<float>, ESPMode::ThreadSafe>(0.f);
	
};
