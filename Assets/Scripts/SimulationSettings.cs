using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// Global simulation settings shared across cloth instances.
/// </summary>
public static class SimulationSettings
{
    public static int numIter = 2;
    public static float kStretch = 0.25f;

    public static float kSelfCollision = 0.5f;

    public static ClothSimulationType clothType = ClothSimulationType.PBD;
    public static int clothResolution = 32;
    public static float clothNodeStep = 0.2f;
    public static float clothDensity = 1.0f;

    public static float selfCollisionRadius = 0.15f;
    public static int maxSelfCollisionPairs = 4096;
    public static SelfCollisionPartitionType selfCollisionPartitionType = SelfCollisionPartitionType.UniformGrid;
    public static float selfCollisionHashCellScale = 1.0f;
    public static int selfCollisionMultiHashLevels = 3;
    public static float selfCollisionMultiHashScale = 2.0f;

    // Mass-spring parameters
    public static float massSpringStructuralK = 600.0f;
    public static float massSpringShearK = 600.0f;
    public static float massSpringBendK = 300.0f;
    public static float massSpringSpringDampingRatio = 0.25f;
    public static ClothIntegrationMethod massSpringIntegrationMethod = ClothIntegrationMethod.SemiImplicitEuler;
    public static float massSpringImplicitDamping = 0.1f;
    public static float massSpringMaxForce = 800.0f;
    public static float massSpringMaxStretch = 1.2f;
    public static float massSpringMinStretch = 0.85f;
    public static float massSpringCompressionRelaxation = 0.35f;
    public static int massSpringStrainIterations = 2;
    public static float massSpringDamping = 0.015f;
    public static float massSpringVelocityDamping = 0.99f;
    public static int massSpringSubsteps = 8;

    public static float kSpring = 0.1f;

    public static float3 gravity = new float3(0.0f, -0.98f, 0.0f);
    public static float timeStep = 1.0f / 60.0f;
    public static float globalDamping = 0.98f;

    public static float3 ballCenter = new float3(0.0f, -4.0f, 3.0f);
    public static float ballRadius = 2.0f;
    public static float sphereSpeed = 6.0f;
    public static float sphereRange = 5.0f;
}
