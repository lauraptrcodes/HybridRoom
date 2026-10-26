using UnrealBuildTool;

public class HybridAcousticsCore : ModuleRules
{
    public HybridAcousticsCore(ReadOnlyTargetRules Target) : base(Target)
    {
        // Kein UE-PCH -> fehlende Includes fallen sofort auf, keine versteckte UE-Abhängigkeit
        PCHUsage = PCHUsageMode.NoPCHs;
        CppStandard = CppStandardVersion.Cpp20;

        bUseRTTI = false;
        bEnableExceptions = false;

        // Nur für die Modul-Boilerplate (IMPLEMENT_MODULE)
        PublicDependencyModuleNames.Add("Core");
    }
}
