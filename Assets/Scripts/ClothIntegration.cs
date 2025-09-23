using Unity.Mathematics;

public static class ClothIntegration
{
    public static void ExplicitEuler(float dt, float3[] positions, float3[] velocities, float3[] forces, float[] invMass, float3[] predictedPositions)
    {
        for (int i = 0; i < positions.Length; i++)
        {
            if (invMass[i] <= 0.0f)
            {
                predictedPositions[i] = positions[i];
                velocities[i] = float3.zero;
                continue;
            }

            float3 acceleration = forces[i] * invMass[i];
            predictedPositions[i] = positions[i] + velocities[i] * dt;
            velocities[i] = velocities[i] + acceleration * dt;
        }
    }

    public static void SemiImplicitEuler(float dt, float3[] positions, float3[] velocities, float3[] forces, float[] invMass, float3[] predictedPositions)
    {
        for (int i = 0; i < positions.Length; i++)
        {
            if (invMass[i] <= 0.0f)
            {
                predictedPositions[i] = positions[i];
                velocities[i] = float3.zero;
                continue;
            }

            float3 acceleration = forces[i] * invMass[i];
            velocities[i] = velocities[i] + acceleration * dt;
            predictedPositions[i] = positions[i] + velocities[i] * dt;
        }
    }

    public static void ImplicitEuler(float dt, float implicitDamping, float3[] positions, float3[] velocities, float3[] forces, float[] invMass, float3[] predictedPositions)
    {
        for (int i = 0; i < positions.Length; i++)
        {
            if (invMass[i] <= 0.0f)
            {
                predictedPositions[i] = positions[i];
                velocities[i] = float3.zero;
                continue;
            }

            float3 acceleration = forces[i] * invMass[i];
            float3 velocity = velocities[i] + acceleration * dt;
            velocity /= (1.0f + implicitDamping * dt);
            velocities[i] = velocity;
            predictedPositions[i] = positions[i] + velocity * dt;
        }
    }

    public static void Verlet(float dt, float velocityDamping, float3[] positions, float3[] velocities, float3[] forces, float[] invMass, float3[] predictedPositions, float3[] previousPositions)
    {
        float dtSq = dt * dt;
        for (int i = 0; i < positions.Length; i++)
        {
            if (invMass[i] <= 0.0f)
            {
                predictedPositions[i] = positions[i];
                previousPositions[i] = positions[i];
                velocities[i] = float3.zero;
                continue;
            }

            float3 current = positions[i];
            float3 prev = previousPositions[i];
            float3 acceleration = forces[i] * invMass[i];
            float3 next = current + (current - prev) * velocityDamping + acceleration * dtSq;
            predictedPositions[i] = next;
        }
    }

    public static void ApplyVelocityUpdate(ClothIntegrationMethod method, float dt, float velocityDamping, float3[] positions, float3[] predictedPositions, float3[] velocities, float[] invMass, float3[] previousPositions)
    {
        float invTwoDt = dt > 1e-6f ? 1.0f / (2.0f * dt) : 0.0f;
        for (int i = 0; i < positions.Length; i++)
        {
            if (invMass[i] <= 0.0f)
            {
                velocities[i] = float3.zero;
                previousPositions[i] = positions[i];
                positions[i] = predictedPositions[i];
                continue;
            }

            switch (method)
            {
                case ClothIntegrationMethod.Verlet:
                {
                    float3 prev = previousPositions[i];
                    float3 current = positions[i];
                    float3 next = predictedPositions[i];
                    previousPositions[i] = current;
                    positions[i] = next;
                    if (invTwoDt > 0.0f)
                        velocities[i] = (next - prev) * invTwoDt * velocityDamping;
                    else
                        velocities[i] = float3.zero;
                    break;
                }
                default:
                {
                    float3 newVelocity = (predictedPositions[i] - positions[i]) / math.max(1e-6f, dt);
                    positions[i] = predictedPositions[i];
                    velocities[i] = newVelocity * velocityDamping;
                    if (previousPositions != null)
                        previousPositions[i] = positions[i];
                    break;
                }
            }
        }
    }
}
