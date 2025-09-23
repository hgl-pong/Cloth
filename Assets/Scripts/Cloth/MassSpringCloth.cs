using System.Collections.Generic;
using Unity.Mathematics;

public class MassSpringCloth : ICloth
{
    struct SpringLink
    {
        public int indexA;
        public int indexB;
        public float restLength;
        public float stiffness;

        public SpringLink(int indexA, int indexB, float restLength, float stiffness)
        {
            this.indexA = indexA;
            this.indexB = indexB;
            this.restLength = restLength;
            this.stiffness = stiffness;
        }
    }

    public int Resolution => N;
    public ClothIntegrationMethod IntegrationMethod => integrationMethod;

    readonly int N;
    readonly ClothIntegrationMethod integrationMethod;
    readonly ClothData[] dataForDraw;

    readonly List<SpringLink> springLinks = new List<SpringLink>();
    readonly List<SelfCollisionConstraint> selfCollisionConstraints = new List<SelfCollisionConstraint>();
    readonly List<NodePair> selfCollisionCandidates = new List<NodePair>();

    ISelfCollisionAccelerator selfCollisionAccelerator;
    SelfCollisionPartitionType currentAcceleratorType;

    readonly float3[] nodePos;
    readonly float3[] nodePrevPos;
    float3[] nodeVel;
    float3[] nodePredPos;
    float3[] nodeForce;
    float[] nodeMass;
    float[] nodeInvMass;
    readonly int numNode;

    public MassSpringCloth(int N, float3 startPos, float nodeStep, float density, ClothIntegrationMethod integrationMethod)
    {
        this.N = N;
        numNode = N * N;
        this.integrationMethod = integrationMethod;

        nodePos = new float3[numNode];
        nodePrevPos = new float3[numNode];
        nodeVel = new float3[numNode];
        nodePredPos = new float3[numNode];
        nodeForce = new float3[numNode];
        nodeMass = new float[numNode];
        nodeInvMass = new float[numNode];

        for (int index = 0; index < numNode; index++)
        {
            int i = index / N;
            int j = index - N * i;

            float3 initialPos = new float3(i * nodeStep, 0.0f, j * nodeStep) + startPos;
            nodePos[index] = initialPos;
            nodePrevPos[index] = initialPos;
            nodeVel[index] = float3.zero;
            nodePredPos[index] = initialPos;
            nodeForce[index] = float3.zero;
        }

        InitialiseMass(nodeStep, density);
        BuildSprings(nodeStep);

        currentAcceleratorType = SimulationSettings.selfCollisionPartitionType;
        selfCollisionAccelerator = SelfCollisionAcceleratorFactory.Create(currentAcceleratorType);

        dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
    }

    void InitialiseMass(float nodeStep, float density)
    {
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                float areaMass = nodeStep * nodeStep * density * 0.5f;
                nodeMass[index0] += areaMass * 2.0f / 3.0f;
                nodeMass[index1] += areaMass * 1.0f / 3.0f;
                nodeMass[index2] += areaMass * 2.0f / 3.0f;
                nodeMass[index3] += areaMass * 1.0f / 3.0f;
            }
        }

        for (int index = 0; index < numNode; index++)
        {
            float mass = math.max(nodeMass[index], 1e-6f);
            nodeMass[index] = mass;
            nodeInvMass[index] = 1.0f / mass;
        }

        nodeInvMass[0] = 0.0f;
        nodeInvMass[N - 1] = 0.0f;
        nodeVel[0] = float3.zero;
        nodeVel[N - 1] = float3.zero;
        nodePrevPos[0] = nodePos[0];
        nodePrevPos[N - 1] = nodePos[N - 1];
    }

    void BuildSprings(float nodeStep)
    {
        float structural = math.max(SimulationSettings.massSpringStructuralK, 0.0f);
        float shear = math.max(SimulationSettings.massSpringShearK, 0.0f);
        float bend = math.max(SimulationSettings.massSpringBendK, 0.0f);
        var addedPairs = new HashSet<long>();

        void TryAddSpring(int a, int b, float stiffness)
        {
            if (stiffness <= 0.0f)
                return;

            long key = SelfCollisionHelpers.EncodePair(a, b);
            if (!addedPairs.Add(key))
                return;

            float rest = math.length(nodePos[a] - nodePos[b]);
            if (rest < 1e-6f)
                rest = nodeStep;
            springLinks.Add(new SpringLink(a, b, rest, stiffness));
        }

        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                int index = j + i * N;

                if (i + 1 < N)
                    TryAddSpring(index, j + (i + 1) * N, structural);
                if (j + 1 < N)
                    TryAddSpring(index, j + 1 + i * N, structural);

                if (i + 1 < N && j + 1 < N)
                    TryAddSpring(index, j + 1 + (i + 1) * N, shear);
                if (i + 1 < N && j - 1 >= 0)
                    TryAddSpring(index, j - 1 + (i + 1) * N, shear);

                if (i + 2 < N)
                    TryAddSpring(index, j + (i + 2) * N, bend);
                if (j + 2 < N)
                    TryAddSpring(index, j + 2 + i * N, bend);
            }
        }
    }

    public ClothData[] GetClothDrawData()
    {
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index = j + i * (N - 1);
                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                dataForDraw[index * 12 + 0].pos = nodePos[index0];
                dataForDraw[index * 12 + 1].pos = nodePos[index1];
                dataForDraw[index * 12 + 2].pos = nodePos[index1];
                dataForDraw[index * 12 + 3].pos = nodePos[index2];
                dataForDraw[index * 12 + 4].pos = nodePos[index2];
                dataForDraw[index * 12 + 5].pos = nodePos[index3];
                dataForDraw[index * 12 + 6].pos = nodePos[index3];
                dataForDraw[index * 12 + 7].pos = nodePos[index0];
                dataForDraw[index * 12 + 8].pos = nodePos[index0];
                dataForDraw[index * 12 + 9].pos = nodePos[index2];
                dataForDraw[index * 12 + 10].pos = nodePos[index1];
                dataForDraw[index * 12 + 11].pos = nodePos[index3];
            }
        }
        return dataForDraw;
    }

    public void UpdateStep(float3 colliderPosition)
    {
        int substeps = math.max(1, SimulationSettings.massSpringSubsteps);
        float subDt = SimulationSettings.timeStep / substeps;
        int strainIterations = math.max(0, SimulationSettings.massSpringStrainIterations);

        for (int step = 0; step < substeps; step++)
        {
            ApplyExternalForces();
            ApplySpringForces();
            IntegratePredictor(subDt);

            for (int iter = 0; iter < strainIterations; iter++)
                ApplyStrainLimiting();

            RebuildSelfCollisionConstraints();
            ResolveSelfCollisions();
            ResolveSphereCollision(colliderPosition);
            IntegrateCorrector(subDt);
        }
    }

    void ApplyExternalForces()
    {
        float3 gravityForce = SimulationSettings.gravity;
        float damping = SimulationSettings.massSpringDamping;
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodeForce[i] = float3.zero;
                continue;
            }

            nodeForce[i] = gravityForce * nodeMass[i] - damping * nodeVel[i] * nodeMass[i];
        }
    }

    void ApplySpringForces()
    {
        float dampingRatio = math.max(0.0f, SimulationSettings.massSpringSpringDampingRatio);
        float maxForce = SimulationSettings.massSpringMaxForce;

        foreach (SpringLink spring in springLinks)
        {
            int a = spring.indexA;
            int b = spring.indexB;
            float3 delta = nodePos[b] - nodePos[a];
            float length = math.length(delta);
            if (length < 1e-6f)
                continue;

            float3 dir = delta / length;
            float displacement = length - spring.restLength;
            float3 relativeVelocity = nodeVel[b] - nodeVel[a];
            float relativeSpeed = math.dot(relativeVelocity, dir);

            float invA = nodeInvMass[a];
            float invB = nodeInvMass[b];
            if (invA + invB <= 0.0f)
                continue;

            float effectiveMass = (invA > 0.0f && invB > 0.0f) ? 1.0f / (invA + invB) : (invA > 0.0f ? 1.0f / invA : 1.0f / invB);
            float criticalDamping = 2.0f * math.sqrt(math.max(1e-6f, spring.stiffness) * math.max(1e-6f, effectiveMass));
            float damping = dampingRatio * criticalDamping;

            float forceScalar = spring.stiffness * displacement - damping * relativeSpeed;
            if (maxForce > 0.0f)
                forceScalar = math.clamp(forceScalar, -maxForce, maxForce);
            float3 force = forceScalar * dir;

            if (invA > 0.0f)
                nodeForce[a] += force;
            if (invB > 0.0f)
                nodeForce[b] -= force;
        }
    }

    void ApplyStrainLimiting()
    {
        float maxStretch = math.max(1.0f, SimulationSettings.massSpringMaxStretch);
        float minStretch = math.clamp(SimulationSettings.massSpringMinStretch, 0.0f, 1.0f);
        float compressionRelax = math.clamp(SimulationSettings.massSpringCompressionRelaxation, 0.0f, 1.0f);

        if (maxStretch <= 1.0f && minStretch <= 0.0f)
            return;

        foreach (SpringLink spring in springLinks)
        {
            int a = spring.indexA;
            int b = spring.indexB;
            float3 delta = nodePredPos[b] - nodePredPos[a];
            float length = math.length(delta);
            if (length < 1e-6f)
                continue;

            float rest = spring.restLength;
            float targetMin = minStretch > 0.0f ? rest * minStretch : 0.0f;
            float targetMax = maxStretch > 1.0f ? rest * maxStretch : rest;

            float clampedLength = length;
            if (maxStretch > 1.0f)
                clampedLength = math.min(clampedLength, targetMax);
            if (minStretch > 0.0f)
                clampedLength = math.max(clampedLength, targetMin);

            float correctionMagnitude = length - clampedLength;
            if (math.abs(correctionMagnitude) <= 1e-6f)
                continue;

            bool compression = correctionMagnitude < 0.0f;
            if (compression && compressionRelax <= 0.0f)
                continue;

            if (compression)
                correctionMagnitude *= compressionRelax;

            float3 dir = delta / length;
            float3 correction = dir * correctionMagnitude;

            float wA = nodeInvMass[a];
            float wB = nodeInvMass[b];
            float wSum = wA + wB;
            if (wSum <= 0.0f)
                continue;

            float weightA = wA / wSum;
            float weightB = wB / wSum;

            if (wA > 0.0f)
                nodePredPos[a] += correction * weightA;
            if (wB > 0.0f)
                nodePredPos[b] -= correction * weightB;

            if (compression)
            {
                float3 relVel = nodeVel[b] - nodeVel[a];
                float relAlong = math.dot(relVel, dir);
                if (relAlong < 0.0f)
                {
                    float impulse = (-relAlong * compressionRelax) / wSum;
                    if (wA > 0.0f)
                        nodeVel[a] += impulse * wA * dir;
                    if (wB > 0.0f)
                        nodeVel[b] -= impulse * wB * dir;
                }
            }
        }
    }

    void IntegratePredictor(float dt)
    {
        switch (integrationMethod)
        {
            case ClothIntegrationMethod.ExplicitEuler:
                ClothIntegration.ExplicitEuler(dt, nodePos, nodeVel, nodeForce, nodeInvMass, nodePredPos);
                break;
            case ClothIntegrationMethod.SemiImplicitEuler:
                ClothIntegration.SemiImplicitEuler(dt, nodePos, nodeVel, nodeForce, nodeInvMass, nodePredPos);
                break;
            case ClothIntegrationMethod.ImplicitEuler:
                ClothIntegration.ImplicitEuler(dt, SimulationSettings.massSpringImplicitDamping, nodePos, nodeVel, nodeForce, nodeInvMass, nodePredPos);
                break;
            case ClothIntegrationMethod.Verlet:
                ClothIntegration.Verlet(dt, SimulationSettings.massSpringVelocityDamping, nodePos, nodeVel, nodeForce, nodeInvMass, nodePredPos, nodePrevPos);
                break;
        }
    }

    void IntegrateCorrector(float dt)
    {
        ClothIntegration.ApplyVelocityUpdate(integrationMethod, dt, SimulationSettings.massSpringVelocityDamping, nodePos, nodePredPos, nodeVel, nodeInvMass, nodePrevPos);
    }

    void RebuildSelfCollisionConstraints()
    {
        selfCollisionConstraints.Clear();

        float radius = SimulationSettings.selfCollisionRadius;
        float stiffness = SimulationSettings.kSelfCollision;
        if (radius <= 0.0f || stiffness <= 0.0f)
            return;

        if (SimulationSettings.selfCollisionPartitionType != currentAcceleratorType)
        {
            currentAcceleratorType = SimulationSettings.selfCollisionPartitionType;
            selfCollisionAccelerator = SelfCollisionAcceleratorFactory.Create(currentAcceleratorType);
        }

        selfCollisionCandidates.Clear();
        selfCollisionAccelerator.Build(nodePredPos, radius);
        selfCollisionAccelerator.CollectPairs(nodePredPos, radius, nodeInvMass, N, SimulationSettings.maxSelfCollisionPairs, selfCollisionCandidates);

        for (int i = 0; i < selfCollisionCandidates.Count; i++)
        {
            if (SimulationSettings.maxSelfCollisionPairs > 0 && selfCollisionConstraints.Count >= SimulationSettings.maxSelfCollisionPairs)
                break;

            NodePair pair = selfCollisionCandidates[i];
            selfCollisionConstraints.Add(new SelfCollisionConstraint(pair.indexA, pair.indexB, radius, stiffness));
        }
    }

    void ResolveSelfCollisions()
    {
        foreach (SelfCollisionConstraint constraint in selfCollisionConstraints)
            constraint.Solve(ref nodePredPos, nodeInvMass);
    }

    void ResolveSphereCollision(float3 colliderPosition)
    {
        float radius = SimulationSettings.ballRadius;
        if (radius <= 0.0f)
            return;

        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
                continue;
            }

            float3 gap = nodePredPos[i] - colliderPosition;
            float gapLen = math.length(gap);
            if (gapLen < radius)
            {
                nodePredPos[i] += math.normalize(gap) * (radius - gapLen);
            }
        }
    }
}
