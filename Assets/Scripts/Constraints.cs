using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// Constraint system definitions.
/// </summary>
public interface IConstraint
{
    void Solve(ref float3[] nodePredPos, float[] nodeInvMass);
}

/// <summary>
/// Maintains the rest distance between two nodes.
/// </summary>
public class DistanceConstraint : IConstraint
{
    public DistanceConstraint(int i1, int i2, float3 position1, float3 position2, float stiffness)
    {
        index1 = i1;
        index2 = i2;

        restLength = math.length(position1 - position2);

        k = math.clamp(stiffness, 0.0f, 1.0f);
        kPrime = 1.0f - math.pow(1.0f - k, 1.0f / SimulationSettings.numIter);
    }

    public int index1;
    public int index2;
    public float restLength;
    float k;
    public float kPrime;

    public void Solve(ref float3[] nodePredPos, float[] nodeInvMass)
    {
        float3 dirVec = nodePredPos[index1] - nodePredPos[index2];
        float len = math.length(dirVec);
        if (len < 1e-6f) return;

        float w1 = nodeInvMass[index1];
        float w2 = nodeInvMass[index2];
        float wSum = w1 + w2;
        if (wSum <= 0.0f) return;

        float3 correction = (1.0f / wSum) * (len - restLength) * (dirVec / len) * kPrime;

        if (w1 > 0.0f)
            nodePredPos[index1] -= correction * w1;
        if (w2 > 0.0f)
            nodePredPos[index2] += correction * w2;
    }
}

/// <summary>
/// Adds longer-range bending support between nodes.
/// </summary>
public class SpringConstraint : IConstraint
{
    public SpringConstraint(int i1, int i2, float3 position1, float3 position2, float stiffness)
    {
        index1 = i1;
        index2 = i2;

        restLength = math.length(position1 - position2);

        k = math.clamp(stiffness, 0.0f, 1.0f);
        kPrime = 1.0f - math.pow(1.0f - k, 1.0f / SimulationSettings.numIter);
    }

    public int index1;
    public int index2;
    public float restLength;
    float k;
    public float kPrime;

    public void Solve(ref float3[] nodePredPos, float[] nodeInvMass)
    {
        float3 dirVec = nodePredPos[index1] - nodePredPos[index2];
        float len = math.length(dirVec);
        if (len < 1e-6f) return;

        float w1 = nodeInvMass[index1];
        float w2 = nodeInvMass[index2];
        float wSum = w1 + w2;
        if (wSum <= 0.0f) return;

        float3 correction = (len - restLength) / wSum * (dirVec / len) * kPrime;

        if (w1 > 0.0f)
            nodePredPos[index1] -= correction * w1;
        if (w2 > 0.0f)
            nodePredPos[index2] += correction * w2;
    }
}

