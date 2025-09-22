using System.Collections.Generic;
using Unity.Mathematics;

public enum SelfCollisionPartitionType
{
    BruteForce = 0,
    UniformGrid = 1,
    SpatialHash = 2,
    MultiLevelSpatialHash = 3,
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
            case SelfCollisionPartitionType.SpatialHash:
                return new SpatialHashSelfCollisionAccelerator();
            case SelfCollisionPartitionType.MultiLevelSpatialHash:
                return new MultiLevelSpatialHashSelfCollisionAccelerator();
            case SelfCollisionPartitionType.BruteForce:
            default:
                return new BruteForceSelfCollisionAccelerator();
        }
    }
}

static class SelfCollisionHelpers
{
    public static readonly List<int3> NeighborOffsets = CreateNeighborOffsets();

    static List<int3> CreateNeighborOffsets()
    {
        var offsets = new List<int3>();
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

                    offsets.Add(new int3(dx, dy, dz));
                }
            }
        }
        return offsets;
    }

    public static bool AreStructuralNeighbours(int indexA, int indexB, int resolution)
    {
        int rowA = indexA / resolution;
        int colA = indexA - rowA * resolution;
        int rowB = indexB / resolution;
        int colB = indexB - rowB * resolution;
        return math.abs(rowA - rowB) <= 1 && math.abs(colA - colB) <= 1;
    }

    public static long EncodePair(int indexA, int indexB)
    {
        if (indexA > indexB)
        {
            int tmp = indexA;
            indexA = indexB;
            indexB = tmp;
        }
        return ((long)indexA << 32) | (uint)indexB;
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
            float w1 = nodeInvMass[i];

            for (int j = i + 1; j < numNode; j++)
            {
                if (limitPairs && results.Count >= maxPairs)
                    return;

                float w2 = nodeInvMass[j];
                if (w1 + w2 <= 0.0f)
                    continue;

                if (SelfCollisionHelpers.AreStructuralNeighbours(i, j, clothResolution))
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
    float cellSize;
    float3 minBounds;

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
        if (cellMap.Count == 0)
            return;

        float radiusSq = radius * radius;
        bool limitPairs = maxPairs > 0;

        foreach (var kvp in cellMap)
        {
            List<int> indices = kvp.Value;
            ProcessCellPairs(indices, indices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, true);

            foreach (int3 offset in SelfCollisionHelpers.NeighborOffsets)
            {
                int3 neighborCell = kvp.Key + offset;
                if (!cellMap.TryGetValue(neighborCell, out var neighborIndices))
                    continue;

                ProcessCellPairs(indices, neighborIndices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, false);

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
                float w1 = nodeInvMass[indexI];

                for (int j = i + 1; j < cellA.Count; j++)
                {
                    if (limitPairs && results.Count >= maxPairs)
                        return;

                    int indexJ = cellA[j];
                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
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

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
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

class SpatialHashSelfCollisionAccelerator : ISelfCollisionAccelerator
{
    readonly Dictionary<int3, List<int>> cellMap = new Dictionary<int3, List<int>>();
    float cellSize;

    public void Build(float3[] positions, float radius)
    {
        float scale = SimulationSettings.selfCollisionHashCellScale;
        if (scale <= 0.0f)
            scale = 1.0f;

        cellSize = math.max(radius * scale, 1e-4f);
        cellMap.Clear();

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
        if (cellMap.Count == 0)
            return;

        float radiusSq = radius * radius;
        bool limitPairs = maxPairs > 0;

        foreach (var kvp in cellMap)
        {
            List<int> indices = kvp.Value;
            ProcessCellPairs(indices, indices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, true);

            foreach (int3 offset in SelfCollisionHelpers.NeighborOffsets)
            {
                int3 neighborCell = kvp.Key + offset;
                if (!cellMap.TryGetValue(neighborCell, out var neighborIndices))
                    continue;

                ProcessCellPairs(indices, neighborIndices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs, results, false);

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
                float w1 = nodeInvMass[indexI];

                for (int j = i + 1; j < cellA.Count; j++)
                {
                    if (limitPairs && results.Count >= maxPairs)
                        return;

                    int indexJ = cellA[j];
                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
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

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
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
        return new int3(
            (int)math.floor(position.x / cellSize),
            (int)math.floor(position.y / cellSize),
            (int)math.floor(position.z / cellSize));
    }
}

class MultiLevelSpatialHashSelfCollisionAccelerator : ISelfCollisionAccelerator
{
    class SpatialHashLevel
    {
        public float CellSize;
        public readonly Dictionary<int3, List<int>> CellMap = new Dictionary<int3, List<int>>();
    }

    readonly List<SpatialHashLevel> levels = new List<SpatialHashLevel>();
    readonly HashSet<long> pairSet = new HashSet<long>();

    public void Build(float3[] positions, float radius)
    {
        levels.Clear();
        pairSet.Clear();

        if (positions.Length == 0 || radius <= 0.0f)
            return;

        int levelCount = math.max(1, SimulationSettings.selfCollisionMultiHashLevels);
        float baseScale = SimulationSettings.selfCollisionHashCellScale;
        if (baseScale <= 0.0f)
            baseScale = 1.0f;

        float levelScale = SimulationSettings.selfCollisionMultiHashScale;
        if (levelScale < 1.0f)
            levelScale = 2.0f;

        for (int levelIndex = 0; levelIndex < levelCount; levelIndex++)
        {
            float multiplier = math.pow(levelScale, levelIndex);
            float cellSize = math.max(radius * baseScale * multiplier, 1e-4f);

            var level = new SpatialHashLevel
            {
                CellSize = cellSize
            };

            for (int i = 0; i < positions.Length; i++)
            {
                int3 cell = PositionToCell(positions[i], cellSize);
                if (!level.CellMap.TryGetValue(cell, out var list))
                {
                    list = new List<int>();
                    level.CellMap.Add(cell, list);
                }
                list.Add(i);
            }

            levels.Add(level);
        }
    }

    public void CollectPairs(float3[] positions, float radius, float[] nodeInvMass, int clothResolution, int maxPairs, List<NodePair> results)
    {
        results.Clear();
        pairSet.Clear();

        if (levels.Count == 0)
            return;

        float radiusSq = radius * radius;
        bool limitPairs = maxPairs > 0;

        foreach (SpatialHashLevel level in levels)
        {
            foreach (var kvp in level.CellMap)
            {
                List<int> indices = kvp.Value;
                ProcessCellPairs(indices, indices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs);

                foreach (int3 offset in SelfCollisionHelpers.NeighborOffsets)
                {
                    int3 neighborCell = kvp.Key + offset;
                    if (!level.CellMap.TryGetValue(neighborCell, out var neighborIndices))
                        continue;

                    ProcessCellPairs(indices, neighborIndices, positions, radiusSq, nodeInvMass, clothResolution, limitPairs, maxPairs);

                    if (limitPairs && pairSet.Count >= maxPairs)
                        goto FinishedCollection;
                }

                if (limitPairs && pairSet.Count >= maxPairs)
                    goto FinishedCollection;
            }

            if (limitPairs && pairSet.Count >= maxPairs)
                break;
        }

    FinishedCollection:
        if (pairSet.Count == 0)
            return;

        foreach (long key in pairSet)
        {
            int indexA = (int)(key >> 32);
            int indexB = (int)(key & 0xffffffff);
            results.Add(new NodePair(indexA, indexB));
            if (limitPairs && results.Count >= maxPairs)
                break;
        }
    }

    void ProcessCellPairs(List<int> cellA, List<int> cellB, float3[] positions, float radiusSq, float[] nodeInvMass, int clothResolution, bool limitPairs, int maxPairs)
    {
        bool selfPair = cellA == cellB;

        if (selfPair)
        {
            for (int i = 0; i < cellA.Count; i++)
            {
                int indexI = cellA[i];
                float w1 = nodeInvMass[indexI];

                for (int j = i + 1; j < cellA.Count; j++)
                {
                    if (limitPairs && pairSet.Count >= maxPairs)
                        return;

                    int indexJ = cellA[j];
                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
                        continue;

                    float3 delta = positions[indexI] - positions[indexJ];
                    float distSq = math.lengthsq(delta);
                    if (distSq >= radiusSq || distSq < 1e-12f)
                        continue;

                    pairSet.Add(SelfCollisionHelpers.EncodePair(indexI, indexJ));
                }
            }
        }
        else
        {
            for (int i = 0; i < cellA.Count; i++)
            {
                int indexI = cellA[i];
                float w1 = nodeInvMass[indexI];

                for (int j = 0; j < cellB.Count; j++)
                {
                    if (limitPairs && pairSet.Count >= maxPairs)
                        return;

                    int indexJ = cellB[j];
                    if (indexI >= indexJ)
                        continue;

                    float w2 = nodeInvMass[indexJ];
                    if (w1 + w2 <= 0.0f)
                        continue;

                    if (SelfCollisionHelpers.AreStructuralNeighbours(indexI, indexJ, clothResolution))
                        continue;

                    float3 delta = positions[indexI] - positions[indexJ];
                    float distSq = math.lengthsq(delta);
                    if (distSq >= radiusSq || distSq < 1e-12f)
                        continue;

                    pairSet.Add(SelfCollisionHelpers.EncodePair(indexI, indexJ));
                }
            }
        }
    }

    static int3 PositionToCell(float3 position, float cellSize)
    {
        return new int3(
            (int)math.floor(position.x / cellSize),
            (int)math.floor(position.y / cellSize),
            (int)math.floor(position.z / cellSize));
    }
}
