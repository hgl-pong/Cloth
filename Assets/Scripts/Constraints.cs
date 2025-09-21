using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// 约束系统 - 包含所有约束类型的定义和处理逻辑
/// </summary>

/// <summary>
/// 距离约束类 - 用于维持两个节点之间的距离
/// </summary>
public class DistanceConstraint
{
    /// <summary>
    /// 构造函数 - 创建距离约束
    /// </summary>
    /// <param name="i1">第一个节点索引</param>
    /// <param name="i2">第二个节点索引</param>
    /// <param name="position1">第一个节点位置</param>
    /// <param name="posiiton2">第二个节点位置</param>
    /// <param name="k_">刚度系数</param>
    public DistanceConstraint(int i1, int i2, float3 position1, float3 posiiton2, float k_)
    {
        index1 = i1;
        index2 = i2;

        pos1 = position1;
        pos2 = posiiton2;

        // 计算静止长度
        restLength = math.length(pos1 - pos2);

        k = k_;
        // 计算修正后的刚度系数，考虑迭代次数
        kPrime = 1.0f - math.pow((1.0f - k), 1.0f / SimulationSettings.numIter);
    }

    /// <summary>
    /// 第一个节点索引
    /// </summary>
    public int index1;
    
    /// <summary>
    /// 第二个节点索引
    /// </summary>
    public int index2;
    
    /// <summary>
    /// 第一个节点初始位置
    /// </summary>
    float3 pos1;
    
    /// <summary>
    /// 第二个节点初始位置
    /// </summary>
    float3 pos2;
    
    /// <summary>
    /// 原始刚度系数
    /// </summary>
    float k;
    
    /// <summary>
    /// 修正后的刚度系数（考虑迭代次数）
    /// </summary>
    public float kPrime;
    
    /// <summary>
    /// 静止长度
    /// </summary>
    public float restLength;
}

/// <summary>
/// 约束求解器 - 处理各种约束的求解逻辑
/// </summary>
public static class ConstraintSolver
{
    /// <summary>
    /// 求解距离约束
    /// </summary>
    /// <param name="constraint">距离约束</param>
    /// <param name="nodePredPos">节点预测位置数组</param>
    /// <param name="nodeInvMass">节点逆质量数组</param>
    public static void SolveDistanceConstraint(DistanceConstraint constraint, ref float3[] nodePredPos, float[] nodeInvMass)
    {
        int index1 = constraint.index1;
        int index2 = constraint.index2;

        float3 dirVec = nodePredPos[index1] - nodePredPos[index2];
        float len = math.length(dirVec);

        // 避免除零错误
        if (len < 1e-6f) return;

        float w1 = nodeInvMass[index1];
        float w2 = nodeInvMass[index2];

        // 计算位置修正量
        float3 dP = (1.0f / (w1 + w2)) * (len - constraint.restLength) * (dirVec / len) * constraint.kPrime;

        // 应用位置修正
        if (w1 > 0.0f)
            nodePredPos[index1] -= dP * w1;
        if (w2 > 0.0f)
            nodePredPos[index2] += dP * w2;
    }
}