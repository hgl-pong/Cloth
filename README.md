# Unity PBD 布料模拟项目

这是一个基于Unity引擎实现的Position Based Dynamics (PBD) 布料模拟项目。

## 项目概述

本项目实现了一个实时的布料物理模拟系统，使用Position Based Dynamics算法来模拟布料的物理行为，包括重力、拉伸约束和碰撞检测。

## 主要特性

- **PBD物理模拟**: 使用Position Based Dynamics算法实现稳定的布料模拟
- **实时渲染**: 通过GPU渲染管线实现高效的实时可视化
- **距离约束**: 实现布料节点间的拉伸约束，保持布料结构
- **重力模拟**: 模拟重力对布料的影响
- **球体碰撞**: 支持与球体的碰撞检测和响应
- **自定义着色器**: 使用自定义Shader进行布料渲染

## 技术实现

### 核心组件

1. **dispathcer.cs** - 主要的布料模拟控制器
   - `Cloth` 类：管理布料的节点位置、速度、质量等属性
   - `DistanceConstraint` 类：实现距离约束算法
   - `PARA` 类：存储物理模拟参数

2. **mainShader.shader** - 自定义着色器
   - 使用StructuredBuffer接收布料数据
   - 实现基于顶点的渲染

3. **material.mat** - 材质文件
   - 应用自定义着色器的材质

### 物理参数

- **迭代次数**: 2次约束求解迭代
- **拉伸系数**: 0.25
- **重力**: (0, -0.98, 0)
- **时间步长**: 1/60秒
- **全局阻尼**: 0.98
- **球体碰撞**: 中心位置(0, -4, 3)，半径2.0

## 项目结构

```
Assets/
├── Scenes/
│   ├── SampleScene.unity      # 示例场景
│   └── pbdcloth.unity         # 布料模拟场景
├── dispathcer.cs              # 主要模拟脚本
├── mainShader.shader          # 自定义着色器
└── material.mat               # 材质文件
```

## 使用方法

1. 打开Unity编辑器
2. 加载 `pbdcloth.unity` 场景
3. 运行场景即可看到布料模拟效果
4. 可以在 `PARA` 类中调整物理参数来改变模拟效果

## 算法原理

### Position Based Dynamics (PBD)

PBD是一种基于位置的物理模拟方法，相比传统的基于力的方法具有更好的稳定性：

1. **预测位置**: 根据当前速度和外力预测下一帧位置
2. **约束求解**: 通过迭代方法满足各种约束条件
3. **位置更新**: 根据约束求解结果更新最终位置
4. **速度更新**: 根据位置变化更新速度

### 距离约束

布料的结构通过距离约束来维持，每个约束确保两个相邻节点之间的距离接近其静止长度。

## 系统要求

- Unity 2019.4 或更高版本
- 支持Compute Shader的显卡
- Windows/Mac/Linux

## 开发环境

- Unity 2019.4+
- C# 脚本
- HLSL 着色器语言

## 许可证

本项目仅供学习和研究使用。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 参考资料

- [Position Based Dynamics](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)
- [Unity Compute Shaders](https://docs.unity3d.com/Manual/class-ComputeShader.html)
- [Cloth Simulation](https://graphics.stanford.edu/~mdfisher/cloth.html)