using Unity.Mathematics;

public struct ClothData
{
    public float3 pos;
}

public enum ClothSimulationType
{
    PBD = 0,
    MassSpring = 1,
}

public enum ClothIntegrationMethod
{
    ExplicitEuler = 0,
    SemiImplicitEuler = 1,
    ImplicitEuler = 2,
    Verlet = 3,
}

public interface ICloth
{
    int Resolution { get; }
    ClothData[] GetClothDrawData();
    void UpdateStep(float3 colliderPosition);
}

public static class ClothFactory
{
    public static ICloth CreateCloth(int resolution, float3 startPos, float nodeStep, float density)
    {
        return CreateCloth(SimulationSettings.clothType, resolution, startPos, nodeStep, density, SimulationSettings.massSpringIntegrationMethod);
    }

    public static ICloth CreateCloth(ClothSimulationType clothType, int resolution, float3 startPos, float nodeStep, float density, ClothIntegrationMethod integrationMethod)
    {
        switch (clothType)
        {
            case ClothSimulationType.MassSpring:
                return new MassSpringCloth(resolution, startPos, nodeStep, density, integrationMethod);
            case ClothSimulationType.PBD:
            default:
                return new PBDCloth(resolution, startPos, nodeStep, density);
        }
    }
}
