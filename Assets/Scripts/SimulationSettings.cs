using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// 全局静态布料模拟设置类
/// 包含所有物理模拟相关的参数
/// </summary>
public static class SimulationSettings
{
    /// <summary>
    /// 迭代次数 - 影响约束求解的精度
    /// </summary>
    public static int numIter = 2;
    
    /// <summary>
    /// 拉伸刚度系数 - 控制布料的拉伸阻力
    /// </summary>
    public static float kStretch = 0.25f;

    /// <summary>
    /// Self-collision stiffness - controls how strongly overlapping nodes are separated
    /// </summary>
    public static float kSelfCollision = 0.5f;

    /// <summary>
    /// Self-collision radius - minimum allowed distance between non-neighbouring nodes
    /// </summary>
    public static float selfCollisionRadius = 0.15f;

    /// <summary>
    /// Maximum self-collision pairs to evaluate per iteration (0 for no limit)
    /// </summary>
    public static int maxSelfCollisionPairs = 4096;

    /// <summary>
    /// Self-collision partition type - selects the acceleration data structure
    /// </summary>
    public static SelfCollisionPartitionType selfCollisionPartitionType = SelfCollisionPartitionType.UniformGrid;

    /// <summary>
    /// Self-collision hash cell scale - multiplier applied to the collision radius when computing hash cell size
    /// </summary>
    public static float selfCollisionHashCellScale = 1.0f;

    /// <summary>
    /// Self-collision multi-hash levels - number of spatial hash levels to evaluate (>= 1)
    /// </summary>
    public static int selfCollisionMultiHashLevels = 3;

    /// <summary>
    /// Self-collision multi-hash scale - growth factor between successive hash levels
    /// </summary>
    public static float selfCollisionMultiHashScale = 2.0f;

    /// <summary>
    /// 弹簧刚度系数 - 控制布料的弯曲弹性
    /// </summary>
    public static float kSpring = 0.1f;

    
    /// <summary>
    /// 重力向量 - 影响布料下垂的方向和强度
    /// </summary>
    public static float3 gravity = new float3(0.0f, -0.98f, 0.0f);
    
    /// <summary>
    /// 时间步长 - 控制模拟的时间精度
    /// </summary>
    public static float timeStep = 1.0f / 60.0f;
    
    /// <summary>
    /// 全局阻尼系数 - 控制布料的能量损失
    /// </summary>
    public static float globalDamping = 0.98f;
    
    /// <summary>
    /// 球体中心位置 - 碰撞检测用
    /// </summary>
    public static float3 ballCenter = new float3(0.0f, -4.0f, 3.0f);
    
    /// <summary>
    /// 球体半径 - 碰撞检测用
    /// </summary>
    public static float ballRadius = 2.0f;
    
    /// <summary>
    /// 球体运动速度 - 控制球体沿x轴的移动速度
    /// </summary>
    public static float sphereSpeed = 6.0f;
    
    /// <summary>
    /// 球体运动范围 - 控制球体在x轴上的移动范围
    /// </summary>
    public static float sphereRange = 5.0f;
}