using System.Collections.Generic;
using Unity.Mathematics;

public enum SelfCollisionPartitionType
{
    BruteForce = 0,
    UniformGrid = 1,
}

public struct NodePair
{
    public int indexA;
    public int indexB;

    public NodePair(int indexA, int indexB)
    {
        this.indexA = indexA;
        this.indexB = indexB;
    }
}

public interface ISelfCollisionAccelerator
{
    void Build(float3[] positions, float radius);
    void CollectPairs(float3[] positions, float radius, float[] nodeInvMass, int clothResolution, int maxPairs, List<NodePair> results);
}

public static class SelfCollisionAcceleratorFactory
{
    public static ISelfCollisionAccelerator Create(SelfCollisionPartitionType type)
    {
        switch (type)
        {
            case SelfCollisionPartitionType.UniformGrid:
                return new UniformGridSelfCollisionAccelerator();
            case SelfCollisionPartitionType.BruteForce:
            default:
                return new BruteForceSelfCollisionAccelerator();
        }
    }
}

class BruteForceSelfCollisionAccelerator : ISelfCollisionAccelerator
{
    public void Build(float3[] positions, float radius)
    {
        // No preprocessing required for brute-force approach.
    }

    public void CollectPairs(float3[] positions, float radius, float[] nodeInvMass, int clothResolution, int maxPairs, List<NodePair> results)
    {
        results.Clear();
        float radiusSq = radius * radius;
        bool limitPairs = maxPairs > 0;

        int numNode = positions.Length;
        for (int i = 0; i < numNode; i++)
        {
            int rowI = i / clothResolution;
            int colI = i - rowI * clothResolution;
            float w1 = nodeInvMass[i];

            for (int j = i + 1; j < numNode; j++)
            {
                if (limitPairs && results.Count >= maxPairs)
                    return;

                float w2 = nodeInvMass[j];
                if (w1 + w2 <= 0.0f)
                    continue;

                int rowJ = j / clothResolution;
                int colJ = j - rowJ * clothResolution;
                if (math.abs(rowI - rowJ) <= 1 && math.abs(colI - colJ) <= 1)
                    continue;

                float3 delta = positions[i] - positions[j];
                float distSq = math.lengthsq(delta);
                if (distSq >= radiusSq || distSq < 1e-12f)
                    continue;

                results.Add(new NodePair(i, j));
            }
        }
    }
}

class UniformGridSelfCollisionAccelerator : ISelfCollisionAccelerator
{
    readonly Dictionary<int3, List<int>> cellMap = new Dictionary<int3, List<int>>();
    readonly List<int3> neighborOffsets;
    float cellSize;
    float3 minBounds;

    public UniformGridSelfCollisionAccelerator()
    {
        neighborOffsets = new List<int3>();
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    if (dx < 0)
                        continue;
                    if (dx == 0 && dy < 0)
                        continue;
                    if (dx == 0 && dy == 0 && dz <= 0)
                        continue;

                    neighborOffsets.Add(new int3(dx, dy, dz));
                }
            }
        }
    }

    public void Build(float3[] positions, float radius)
    {
        cellSize = math.max(radius, 1e-3f);
        cellMap.Clear();
        if (positions.Length == 0)
            return;

        minBounds = positions[0];
        for (int i = 1; i < positions.Length; i++)
        {
            minBounds = math.min(minBounds, positions[i]);
        }

        for (int i = 0; i < positions.Length; i++)
        {
            int3 cell = PositionToCell(positions[i]);
            if (!cellMap.TryGetValue(cell, out var list))
            {
                list = new List<int>();
                cellMap.Add(cell, list);
            }
            list.Add(i);
        }
    }

    public void CollectPairs(float3[] positions, float radius, float[] nodeInvMass, int clothResolution, int maxPairs, List<NodePair> results)
    {
        results.Clear();
        if (positions.Length == 0 || radius <= 0.0f)
            return;

        float radiusSq = radius * radius;
        bool limitPairs = maxPairs > 0;

        foreach (var kvp in cellMap)
        {
            int3 cell = kvp.Key;
            List<int> indices = kvp.Value;
            ProcessCellPairs(indices, indices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, selfPair: true);

            foreach (int3 offset in neighborOffsets)
            {
                int3 neighborCell = cell + offset;
                if (!cellMap.TryGetValue(neighborCell, out var neighborIndices))
                    continue;

                ProcessCellPairs(indices, neighborIndices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, selfPair: false);

                if (limitPairs && results.Count >= maxPairs)
                    return;
            }

            if (limitPairs && results.Count >= maxPairs)
                return;
        }
    }

    void ProcessCellPairs(List<int> cellA, List<int> cellB, float3[] positions, float radiusSq, float[] nodeInvMass, int clothResolution, bool limitPairs, int maxPairs, List<NodePair> results, bool selfPair)
    {
        if (selfPair)
        {
            for (int i = 0; i < cellA.Count; i++)
            {
                int indexI = cellA[i];
                int rowI = indexI / clothResolution;
                int colI = indexI - rowI * clothResolution;
                float w1 = nodeInvMass[indexI];

                for (int j = i + 1; j < cellA.Count; j++)
                {
                    if (limitPairs && results.Count >= maxPairs)
                        return;

                    int indexJ = cellA[j];
                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    int rowJ = indexJ / clothResolution;
                    int colJ = indexJ - rowJ * clothResolution;
                    if (math.abs(rowI - rowJ) <= 1 && math.abs(colI - colJ) <= 1)
                        continue;

                    float3 delta = positions[indexI] - positions[indexJ];
                    float distSq = math.lengthsq(delta);
                    if (distSq >= radiusSq || distSq < 1e-12f)
                        continue;

                    results.Add(new NodePair(indexI, indexJ));
                }
            }
        }
        else
        {
            for (int i = 0; i < cellA.Count; i++)
            {
                int indexI = cellA[i];
                int rowI = indexI / clothResolution;
                int colI = indexI - rowI * clothResolution;
                float w1 = nodeInvMass[indexI];

                for (int j = 0; j < cellB.Count; j++)
                {
                    if (limitPairs && results.Count >= maxPairs)
                        return;

                    int indexJ = cellB[j];
                    if (indexI >= indexJ)
                        continue;

                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    int rowJ = indexJ / clothResolution;
                    int colJ = indexJ - rowJ * clothResolution;
                    if (math.abs(rowI - rowJ) <= 1 && math.abs(colI - colJ) <= 1)
                        continue;

                    float3 delta = positions[indexI] - positions[indexJ];
                    float distSq = math.lengthsq(delta);
                    if (distSq >= radiusSq || distSq < 1e-12f)
                        continue;

                    results.Add(new NodePair(indexI, indexJ));
                }
            }
        }
    }

    int3 PositionToCell(float3 position)
    {
        float3 relative = position - minBounds;
        return new int3(
            (int)math.floor(relative.x / cellSize),
            (int)math.floor(relative.y / cellSize),
            (int)math.floor(relative.z / cellSize));
    }
}
