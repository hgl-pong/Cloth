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
    public int N;

    readonly float3[] nodePos;
    float3[] nodeVel;
    float3[] nodePredPos;
    float3[] nodeForce;
    float[] nodeMass;
    float[] nodeInvMass;
    readonly List<SpringLink> springLinks = new List<SpringLink>();
    readonly List<SelfCollisionConstraint> selfCollisionConstraints = new List<SelfCollisionConstraint>();
    readonly List<NodePair> selfCollisionCandidates = new List<NodePair>();
    ISelfCollisionAccelerator selfCollisionAccelerator;
    SelfCollisionPartitionType currentAcceleratorType;
    readonly ClothIntegrationMethod integrationMethod;
    float3[] nodePrevPos;

    readonly ClothData[] dataForDraw;

    readonly int numNode;

    public MassSpringCloth(int N, float3 startPos, float nodeStep, float density, ClothIntegrationMethod integrationMethod)
    {
        this.N = N;
        numNode = N * N;
        this.integrationMethod = integrationMethod;

        nodePos = new float3[numNode];
        nodeVel = new float3[numNode];
        nodePredPos = new float3[numNode];
        nodeForce = new float3[numNode];
        nodeMass = new float[numNode];
        nodeInvMass = new float[numNode];
        nodePrevPos = new float3[numNode];

        for (int index = 0; index < numNode; index++)
        {
            int i = index / N;
            int j = index - N * i;

            float3 tmpPos = new float3(i * nodeStep, 0.0f, j * nodeStep);
            float3 initialPos = tmpPos + startPos;
            nodePos[index] = initialPos;
            nodeVel[index] = float3.zero;
            nodePredPos[index] = initialPos;
            nodePrevPos[index] = initialPos;
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
        // Using same mass distribution as PBD cloth for consistency
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

        // anchor top corners
        nodeInvMass[0] = 0.0f;
        nodeInvMass[N - 1] = 0.0f;
        nodeVel[0] = float3.zero;
        nodeVel[N - 1] = float3.zero;
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
                {
                    int down = j + (i + 1) * N;
                    TryAddSpring(index, down, structural);
                }
                if (j + 1 < N)
                {
                    int right = j + 1 + i * N;
                    TryAddSpring(index, right, structural);
                }

                if (i + 1 < N && j + 1 < N)
                {
                    int diag = j + 1 + (i + 1) * N;
                    TryAddSpring(index, diag, shear);
                }
                if (i + 1 < N && j - 1 >= 0)
                {
                    int diagLeft = j - 1 + (i + 1) * N;
                    TryAddSpring(index, diagLeft, shear);
                }

                if (i + 2 < N)
                {
                    int bendDown = j + (i + 2) * N;
                    TryAddSpring(index, bendDown, bend);
                }
                if (j + 2 < N)
                {
                    int bendRight = j + 2 + i * N;
                    TryAddSpring(index, bendRight, bend);
                }
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
            {
                ApplyStrainLimiting();
            }

            RebuildSelfCollisionConstraints();
            ResolveSelfCollisions();
            ResolveSphereCollision(colliderPosition);
            IntegrateCorrector(subDt);
        }
    }

    void ApplyExternalForces()
    {
        float3 gravityForce = SimulationSettings.gravity;
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodeForce[i] = float3.zero;
                continue;
            }

            nodeForce[i] = gravityForce * nodeMass[i];

            // simple velocity damping
            nodeForce[i] += -SimulationSettings.massSpringDamping * nodeVel[i] * nodeMass[i];
        }
    }

    void ApplySpringForces()
    {
        foreach (SpringLink spring in springLinks)
        {
            int a = spring.indexA;
            int b = spring.indexB;
            float3 delta = nodePos[b] - nodePos[a];
            float length = math.length(delta);
            if (length < 1e-6f)
                continue;

            float displacement = length - spring.restLength;
            float3 dir = delta / length;

            float invMassA = nodeInvMass[a];
            float invMassB = nodeInvMass[b];
            float3 relativeVelocity = nodeVel[b] - nodeVel[a];
            float relativeSpeed = math.dot(relativeVelocity, dir);

            if (invMassA + invMassB <= 0.0f)
                continue;

            float effectiveMass;
            if (invMassA > 0.0f && invMassB > 0.0f)
                effectiveMass = 1.0f / (invMassA + invMassB);
            else if (invMassA > 0.0f)
                effectiveMass = 1.0f / invMassA;
            else
                effectiveMass = 1.0f / invMassB;

            float dampingRatio = math.max(0.0f, SimulationSettings.massSpringSpringDampingRatio);
            float criticalDamping = 2.0f * math.sqrt(math.max(1e-6f, spring.stiffness) * math.max(1e-6f, effectiveMass));
            float damping = dampingRatio * criticalDamping;

            float forceScalar = spring.stiffness * displacement - damping * relativeSpeed;
            float maxForce = SimulationSettings.massSpringMaxForce;
            if (maxForce > 0.0f)
                forceScalar = math.clamp(forceScalar, -maxForce, maxForce);
            float3 force = forceScalar * dir;

            if (invMassA > 0.0f)
                nodeForce[a] += force;
            if (invMassB > 0.0f)
                nodeForce[b] -= force;
        }
    }

    void ApplyStrainLimiting()
    {
        float maxStretch = math.max(1.0f, SimulationSettings.massSpringMaxStretch);
        float minStretch = math.clamp(SimulationSettings.massSpringMinStretch, 0.0f, 1.0f);
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

            bool isCompression = correctionMagnitude < 0.0f;
            float compressionRelax = 1.0f;
            if (isCompression)
            {
                compressionRelax = math.clamp(SimulationSettings.massSpringCompressionRelaxation, 0.0f, 1.0f);
                if (compressionRelax <= 0.0f)
                    continue;
                correctionMagnitude *= compressionRelax;
            }

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

            if (isCompression)
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
                IntegratePredictorExplicit(dt);
                break;
            case ClothIntegrationMethod.SemiImplicitEuler:
                IntegratePredictorSemiImplicit(dt);
                break;
            case ClothIntegrationMethod.ImplicitEuler:
                IntegratePredictorImplicit(dt);
                break;
            case ClothIntegrationMethod.Verlet:
                IntegratePredictorVerlet(dt);
                break;
        }
    }

    void IntegratePredictorExplicit(float dt)
    {
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
                continue;
            }

            float3 acceleration = nodeForce[i] * nodeInvMass[i];
            nodePredPos[i] = nodePos[i] + nodeVel[i] * dt;
            nodeVel[i] = nodeVel[i] + acceleration * dt;
        }
    }

    void IntegratePredictorSemiImplicit(float dt)
    {
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
                continue;
            }

            float3 acceleration = nodeForce[i] * nodeInvMass[i];
            nodeVel[i] = nodeVel[i] + acceleration * dt;
            nodePredPos[i] = nodePos[i] + nodeVel[i] * dt;
        }
    }

    void IntegratePredictorImplicit(float dt)
    {
        float implicitDamping = math.max(0.0f, SimulationSettings.massSpringImplicitDamping);
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
                continue;
            }

            float3 acceleration = nodeForce[i] * nodeInvMass[i];
            float3 velocity = nodeVel[i] + acceleration * dt;
            velocity /= (1.0f + implicitDamping * dt);
            nodeVel[i] = velocity;
            nodePredPos[i] = nodePos[i] + velocity * dt;
        }
    }

    void IntegratePredictorVerlet(float dt)
    {
        float damping = math.clamp(SimulationSettings.massSpringVelocityDamping, 0.0f, 1.0f);
        float dtSq = dt * dt;
        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
                nodePrevPos[i] = nodePos[i];
                continue;
            }

            float3 current = nodePos[i];
            float3 previous = nodePrevPos[i];
            float3 acceleration = nodeForce[i] * nodeInvMass[i];
            float3 next = current + (current - previous) * damping + acceleration * dtSq;
            nodePredPos[i] = next;
        }
    }

    void RebuildSelfCollisionConstraints()
    {
        selfCollisionConstraints.Clear();

        float radius = SimulationSettings.selfCollisionRadius;
        float stiffness = SimulationSettings.kSelfCollision;
        if (radius <= 0.0f || stiffness <= 0.0f)
            return;

        int maxPairs = math.max(0, SimulationSettings.maxSelfCollisionPairs);

        if (selfCollisionAccelerator == null || SimulationSettings.selfCollisionPartitionType != currentAcceleratorType)
        {
            currentAcceleratorType = SimulationSettings.selfCollisionPartitionType;
            selfCollisionAccelerator = SelfCollisionAcceleratorFactory.Create(currentAcceleratorType);
        }

        selfCollisionAccelerator.Build(nodePredPos, radius);

        selfCollisionAccelerator.CollectPairs(nodePredPos, radius, nodeInvMass, N, maxPairs, selfCollisionCandidates);

        for (int i = 0; i < selfCollisionCandidates.Count; i++)
        {
            if (maxPairs > 0 && selfCollisionConstraints.Count >= maxPairs)
                break;

            NodePair pair = selfCollisionCandidates[i];
            selfCollisionConstraints.Add(new SelfCollisionConstraint(pair.indexA, pair.indexB, radius, stiffness));
        }
    }

    void ResolveSelfCollisions()
    {
        foreach (SelfCollisionConstraint constraint in selfCollisionConstraints)
        {
            constraint.Solve(ref nodePredPos, nodeInvMass);
        }
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

            float3 gapVec = nodePredPos[i] - colliderPosition;
            float gapLen = math.length(gapVec);
            if (gapLen < radius)
            {
                float3 correction = math.normalize(gapVec) * (radius - gapLen);
                nodePredPos[i] += correction;
            }
        }
    }

    void IntegrateCorrector(float dt)
    {
        float velocityDamping = math.clamp(SimulationSettings.massSpringVelocityDamping, 0.0f, 1.0f);

        if (integrationMethod == ClothIntegrationMethod.Verlet)
        {
            float invTwoDt = 1.0f / math.max(1e-6f, 2.0f * dt);
            for (int i = 0; i < numNode; i++)
            {
                if (nodeInvMass[i] <= 0.0f)
                {
                    nodePrevPos[i] = nodePos[i];
                    nodePos[i] = nodePredPos[i];
                    nodeVel[i] = float3.zero;
                    continue;
                }

                float3 prev = nodePrevPos[i];
                float3 current = nodePos[i];
                float3 next = nodePredPos[i];

                nodePrevPos[i] = current;
                nodePos[i] = next;
                nodeVel[i] = (next - prev) * invTwoDt * velocityDamping;
            }
        }
        else
        {
            for (int i = 0; i < numNode; i++)
            {
                if (nodeInvMass[i] <= 0.0f)
                {
                    nodeVel[i] = float3.zero;
                    nodePos[i] = nodePredPos[i];
                    nodePrevPos[i] = nodePos[i];
                    continue;
                }

                float3 newVelocity = (nodePredPos[i] - nodePos[i]) / dt;
                nodePos[i] = nodePredPos[i];
                nodeVel[i] = newVelocity * velocityDamping;
                nodePrevPos[i] = nodePos[i];
            }
        }
    }

}
