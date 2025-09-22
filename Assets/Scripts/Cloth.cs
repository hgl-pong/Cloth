using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public struct ClothData
{
    public float3 pos;
}

public class Cloth
{
    public float3[] nodePos;
    public float3[] nodeVel;
    public float3[] nodePredPos;
    public float3[] nodeForce;
    public float[] nodeMass;
    public float[] nodeInvMass;
    public List<IConstraint> constraintList;
    List<SelfCollisionConstraint> selfCollisionConstraints;
    ISelfCollisionAccelerator selfCollisionAccelerator;
    List<NodePair> selfCollisionCandidates;
    SelfCollisionPartitionType currentAcceleratorType;
    ClothData[] dataForDraw;

    public int numNode;
    public int N;
    
    public Cloth(int N, float3 startPos, float nodeStep, float density)
    {
        // it shoule note that j + N * i = index
        this.N = N;
        numNode = N * N;

        // now init the nodePos
        nodePos = new float3[numNode];
        for (int index = 0; index < numNode; index++)
        {
            int i = index / N;
            int j = index - N * i;

            float3 tmpPos = new float3(i * nodeStep, 0.0f, j * nodeStep);
            nodePos[index] = tmpPos + startPos;
        }

        // now init the nodePredPos nodeFroce and nodeVel
        nodePredPos = new float3[numNode];
        nodeVel = new float3[numNode];
        nodeForce = new float3[numNode];
        for (int index = 0; index < numNode; index++)
        {
            nodePredPos[index] = new float3(0.0f, 0.0f, 0.0f);
            nodeVel[index] = new float3(0.0f, 0.0f, 0.0f);
            nodeForce[index] = new float3(0.0f, 0.0f, 0.0f);
        }

        //now init the nodeMass, there are (N-1) * (N-1) Trapeziums, and we deal with (N-1) * (N-1) * 2 triangles
        nodeMass = new float[numNode];
        nodeInvMass = new float[numNode];
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                float tMass = nodeStep * nodeStep * 0.5f;
                nodeMass[index0] += tMass * 2.0f / 3.0f;
                nodeMass[index1] += tMass * 1.0f / 3.0f;
                nodeMass[index2] += tMass * 2.0f / 3.0f;
                nodeMass[index3] += tMass * 1.0f / 3.0f;
            }
        }

        for (int index = 0; index < numNode; index++)
        {
            nodeInvMass[index] = 1.0f / nodeMass[index];
        }
        // fix two points, so the mass should be inf 
        nodeInvMass[0] = 0.0f;
        nodeInvMass[N - 1] = 0.0f;

        // now we add distance constraints
        constraintList = new List<IConstraint>();
        selfCollisionConstraints = new List<SelfCollisionConstraint>();
        selfCollisionCandidates = new List<NodePair>();
        currentAcceleratorType = SimulationSettings.selfCollisionPartitionType;
        selfCollisionAccelerator = SelfCollisionAcceleratorFactory.Create(currentAcceleratorType);
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                var tDisConstraint0 = new DistanceConstraint(index0, index1, nodePos[index0], nodePos[index1], SimulationSettings.kStretch);
                var tDisConstraint1 = new DistanceConstraint(index3, index2, nodePos[index3], nodePos[index2], SimulationSettings.kStretch);
                var tDisConstraint2 = new DistanceConstraint(index1, index2, nodePos[index1], nodePos[index2], SimulationSettings.kStretch);
                var tDisConstraint3 = new DistanceConstraint(index0, index3, nodePos[index0], nodePos[index3], SimulationSettings.kStretch);
                var tDisConstraint4 = new DistanceConstraint(index0, index2, nodePos[index0], nodePos[index2], SimulationSettings.kStretch);
                var tDisConstraint5 = new DistanceConstraint(index1, index3, nodePos[index1], nodePos[index3], SimulationSettings.kStretch);

                constraintList.Add(tDisConstraint0);
                constraintList.Add(tDisConstraint1);
                constraintList.Add(tDisConstraint2);
                constraintList.Add(tDisConstraint3);
                constraintList.Add(tDisConstraint4);
                constraintList.Add(tDisConstraint5);
            }
        }

        // now we add spring constraints
        if (SimulationSettings.kSpring > 0.0f)
        {
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    int index = j + i * N;

                    if (i + 2 < N)
                    {
                        int indexDown = j + (i + 2) * N;
                        constraintList.Add(new SpringConstraint(index, indexDown, nodePos[index], nodePos[indexDown], SimulationSettings.kSpring));
                    }

                    if (j + 2 < N)
                    {
                        int indexRight = j + 2 + i * N;
                        constraintList.Add(new SpringConstraint(index, indexRight, nodePos[index], nodePos[indexRight], SimulationSettings.kSpring));
                    }

                    if (i + 2 < N && j + 2 < N)
                    {
                        int indexDiag = j + 2 + (i + 2) * N;
                        constraintList.Add(new SpringConstraint(index, indexDiag, nodePos[index], nodePos[indexDiag], SimulationSettings.kSpring));
                    }

                    if (i + 2 < N && j >= 2)
                    {
                        int indexDiagLeft = j - 2 + (i + 2) * N;
                        constraintList.Add(new SpringConstraint(index, indexDiagLeft, nodePos[index], nodePos[indexDiagLeft], SimulationSettings.kSpring));
                    }
                }
            }
        }

        // now init the size of cloth data
        dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
    }

    public ClothData[] getColthDrawData()
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

    public void updateStep(float3 ballPos)
    {
        calculateGravity();
        for (int i = 0; i < SimulationSettings.numIter; i++)
        {
            rebuildSelfCollisionConstraints();
            updateConstraints();
        }
        collisionDetect(ballPos);
        integrate();
    }

    void calculateGravity()
    {
        for (int i = 0; i < numNode; i++)
        {
            nodeForce[i] = new float3(0.0f, 0.0f, 0.0f);

            if (nodeInvMass[i] > 0)
            {
                nodeForce[i] += SimulationSettings.gravity * nodeMass[i];
            }
        }

        for (int i = 0; i < numNode; i++)
        {
            nodeVel[i] *= SimulationSettings.globalDamping;
            nodeVel[i] = nodeVel[i] + (nodeForce[i] * nodeInvMass[i] * SimulationSettings.timeStep);
        }

        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
            }
            else
            {
                nodePredPos[i] = nodePos[i] + (nodeVel[i] * SimulationSettings.timeStep);
            }
        }
    }

    void updateConstraints()
    {
        foreach (IConstraint constraint in constraintList)
        {
            constraint.Solve(ref nodePredPos, nodeInvMass);
        }

        foreach (SelfCollisionConstraint constraint in selfCollisionConstraints)
        {
            constraint.Solve(ref nodePredPos, nodeInvMass);
        }
    }

    void rebuildSelfCollisionConstraints()
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

        if (selfCollisionCandidates == null)
            selfCollisionCandidates = new List<NodePair>();

        selfCollisionAccelerator.CollectPairs(nodePredPos, radius, nodeInvMass, N, maxPairs, selfCollisionCandidates);

        for (int i = 0; i < selfCollisionCandidates.Count; i++)
        {
            if (maxPairs > 0 && selfCollisionConstraints.Count >= maxPairs)
                break;

            NodePair pair = selfCollisionCandidates[i];
            selfCollisionConstraints.Add(new SelfCollisionConstraint(pair.indexA, pair.indexB, radius, stiffness));
        }
    }

    void collisionDetect(float3 ballPos)
    {
        for (int i = 0; i < numNode; i++)
        {
            float3 gapVec = nodePredPos[i] - ballPos;
            float gapLen = math.length(gapVec);

            if (gapLen < SimulationSettings.ballRadius)
            {
                nodePredPos[i] += math.normalize(gapVec) * (SimulationSettings.ballRadius - gapLen);
                nodePos[i] = nodePredPos[i];
            }
        }
    }
    
    void integrate()
    {
        for (int i = 0; i < numNode; i++)
        {
            nodeVel[i] = (nodePredPos[i] - nodePos[i]) / SimulationSettings.timeStep;
            nodePos[i] = nodePredPos[i];
        }
    }
}


