#include "HybridAcousticsRoom.h"

#include "Components/BoxComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Kismet/GameplayStatics.h"
#include "AudioDevice.h"

#include "Types.h"
#include "FDTDGrid.h"
#include "IrGrid.h"
#include "Baker.h"
#include "Hybrid.h"
#include "Convolver.h"
#include "HybridIrStore.h"

struct FHybridBakeResult
{
	fdtd::SimulationConfig Cfg;
	fdtd::IrGrid Grid;
	fdtd::BakeSettings Settings;
	float T60 = -1.f;   // gemessen aus der FDTD-IR
};

// Sets default values
AHybridAcousticsRoom::AHybridAcousticsRoom()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RoomBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("RoomBounds"));
	RoomBounds->SetBoxExtent(FVector(250.f, 200.f, 150.f)); // 5 x 4 x 3 m
	RoomBounds->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	RootComponent = RoomBounds;

	SourcePoint = CreateDefaultSubobject<USceneComponent>(TEXT("SourcePoint"));
	SourcePoint->SetupAttachment(RoomBounds);
	SourcePoint->SetRelativeLocation(FVector(-130.f, -100.f, 0.f));


}

// Called when the game starts or when spawned
void AHybridAcousticsRoom::BeginPlay()
{
	Super::BeginPlay();
	if (bBakeOnBeginPlay) BakeAsync();
}

void AHybridAcousticsRoom::EndPlay(const EEndPlayReason::Type Reason)
{
	CancelFlag->store(true);         // laufenden Bake abbrechen
	HybridIrStore::Get().Clear();   // Spatializer fällt auf "nur dry" zurück
	Super::EndPlay(Reason);
}

FVector AHybridAcousticsRoom::WorldToRoomMeters(const FVector& World) const
{
	const FVector Local = RoomBounds->GetComponentTransform().InverseTransformPosition(World);
	return (Local + RoomBounds->GetUnscaledBoxExtent()) * RoomBounds->GetComponentScale() / 100.0;
}

FVector AHybridAcousticsRoom::RoomMetersToWorld(const FVector& Meters) const
{
	const FVector Local = Meters * 100.0 / RoomBounds->GetComponentScale() - RoomBounds->GetUnscaledBoxExtent();
	return RoomBounds->GetComponentTransform().TransformPosition(Local);
}

float AHybridAcousticsRoom::GetMeasuredT60() const
{
	return Bake.IsValid() ? Bake->T60 : -1.f;
}

void AHybridAcousticsRoom::PublishReceiver(int32 Receiver)
{
	const fdtd::IrGrid& G = Bake->Grid;
	const fdtd::SimulationConfig& C = Bake->Cfg;
	const fdtd::Vec3& Src = Bake->Settings.source;

	float X, Y, Z;
	G.position(Receiver, X, Y, Z);
	const double Dist = FMath::Sqrt(FMath::Square(X - Src.x) + FMath::Square(Y - Src.y) + FMath::Square(Z - Src.z));

	// Gemessene Nachhallzeit, sonst Fallback
	const float T60 = Bake->T60 > 0.f ? Bake->T60 : FallbackT60;

	// FDTD (LR4-Tiefpass) + synthetischer Nachhall (LR4-Hochpass), Übergang bei MockSchroederHz
	std::vector<float> Ir = fdtd::makeHybrid(G.ir(Receiver), G.length, G.sampleRate,
		MockSchroederHz, T60, (float)(Dist / C.c));

	// Energie-Normierung: WetGain bleibt unabhängig von Raum und Empfänger vergleichbar
	double Energy = 0.0;
	for (float V : Ir) Energy += (double)V * V;
	const float Norm = Energy > 0.0 ? (float)(1.0 / FMath::Sqrt(Energy)) : 0.f;
	for (float& V : Ir) V *= Norm;

	HybridIrStore::Get().Publish(fdtd::partitionIr(Ir.data(), (int)Ir.size(), HybridIrStore::BlockSize));
}

// Called every frame
void AHybridAcousticsRoom::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	HybridIrStore::Get().SetGains(DryGain, WetGain);

	if (bDrawDebug)
	{
		const FVector Src = SoundSource ? SoundSource->GetActorLocation() : SourcePoint->GetComponentLocation();
		DrawDebugSphere(GetWorld(), Src, 20.f, 12, FColor::Orange);
	}

	if (!Bake.IsValid()) return;
	//get cam pos
	APlayerCameraManager* Cam = UGameplayStatics::GetPlayerCameraManager(this, 0);
	if (!Cam) return;

	const FVector P = WorldToRoomMeters(Cam->GetCameraLocation());
	const int32 R = Bake->Grid.nearest((float)P.X, (float)P.Y, (float)P.Z);

	if (bDrawDebug)
	{
		float X, Y, Z;
		Bake->Grid.position(R, X, Y, Z);
		DrawDebugPoint(GetWorld(), RoomMetersToWorld(FVector(X, Y, Z)), 12.f, FColor::Green);
	}

	if (R != CurrentReceiver)
	{
		CurrentReceiver = R;
		PublishReceiver(R);
	}
}


void AHybridAcousticsRoom::BakeAsync()
{
	if (bBaking.exchange(true)) return;
	CancelFlag->store(false);
	Progress->store(0.f);

	const FVector SizeM = 2.0 * RoomBounds->GetScaledBoxExtent() / 100.0;
	const FVector SrcWorld = SoundSource ? SoundSource->GetActorLocation() : SourcePoint->GetComponentLocation();
	const FVector SrcM = WorldToRoomMeters(SrcWorld);

	//für fdtd grid simulation
	fdtd::SimulationConfig Cfg;
	Cfg.Lx = SizeM.X; Cfg.Ly = SizeM.Y; Cfg.Lz = SizeM.Z;
	Cfg.fmax = MockSchroederHz * FmaxFactor;
	Cfg.pointsPerLambda = PointsPerLambda;
	Cfg.pmlThickness = 0; //geschlossener raum (keine pml)

	//für baking
	fdtd::BakeSettings Settings;
	Settings.source = fdtd::Vec3{ SrcM.X, SrcM.Y, SrcM.Z };
	Settings.irLength = IrLengthSec;
	Settings.receiverSpacing = ReceiverSpacingCm / 100.0;
	Settings.wallAlpha = WallAlpha;
	Settings.outRate = 48000;

	if (UWorld* World = GetWorld())
	{
		if (FAudioDeviceHandle Device = World->GetAudioDevice())
		{
			Settings.outRate = (int)Device->GetSampleRate();
		}
	}

	TWeakObjectPtr<AHybridAcousticsRoom> WeakThis(this);
	auto Cancel = CancelFlag;   // geteilte Flags in den Worker kopieren, nicht den Actor
	auto Prog = Progress;

	Async(EAsyncExecution::Thread, [WeakThis, Cfg, Settings, Cancel, Prog]()
	{
		auto PFor = [](int N, const std::function<void(int)>& Body){
			ParallelFor(N, [&Body](int32 i) { Body(i); });
		};
		auto OnProgress = [Cancel, Prog](float P){
			Prog->store(P);
			return !Cancel->load();   // false -> Bake bricht ab
		};
	

		fdtd::BakeStats Stats;
		TSharedPtr<FHybridBakeResult, ESPMode::ThreadSafe> Result = MakeShared<FHybridBakeResult, ESPMode::ThreadSafe>();

		Result->Settings = Settings;
		Result->Grid = fdtd::bake(Cfg, Settings, &Stats, PFor, OnProgress);

		if (!Stats.aborted && Result->Grid.numReceivers() > 0)
		{
			// Abgeleitete Werte aus dem Gitter übernehmen (für Quellposition/c in PublishReceiver)
			Result->Cfg = Cfg;
			Result->Cfg.dx = Stats.dx;
			Result->Cfg.dt = Stats.dt;
			Result->Cfg.Nx = Stats.Nx; Result->Cfg.Ny = Stats.Ny; Result->Cfg.Nz = Stats.Nz;

			// T60 am Empfänger in der Raummitte messen (für den synthetischen HF-Nachhall)
			const fdtd::IrGrid& G = Result->Grid;
			const int Center = G.nearest((float)Cfg.Lx * 0.5f, (float)Cfg.Ly * 0.5f, (float)Cfg.Lz * 0.5f);
			Result->T60 =  fdtd::estimateT60(G.ir(Center), G.length, G.sampleRate);

			UE_LOG(LogTemp, Log, TEXT("HybridAcoustics bake: grid %dx%dx%d, dx=%.3f m, courant=%.3f, %d steps, %d receivers, fmax=%.0f Hz, alpha=%.2f, T60=%.2f s"),
				Stats.Nx, Stats.Ny, Stats.Nz, Stats.dx, Stats.courant, Stats.steps, Stats.receivers,
				Cfg.fmax, Settings.wallAlpha, Result->T60);
		}
		else
		{
			UE_LOG(LogTemp, Log, TEXT("HybridAcoustics bake: abgebrochen"));
		}

		AsyncTask(ENamedThreads::GameThread, [WeakThis, Result, Aborted = Stats.aborted]()
		{
			AHybridAcousticsRoom* Self = WeakThis.Get();
			if (!Self) return;

			Self->bBaking = false;
			if (Aborted || Result->Grid.numReceivers() == 0) return;

			Self->Bake = Result;
			Self->CurrentReceiver = INDEX_NONE; // erzwingt Publish im nächsten Tick
			Self->Progress->store(1.f);
		});
	});
}

