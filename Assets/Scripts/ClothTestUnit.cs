using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
public class ClothTestUnit : MonoBehaviour
{
    // Start is called before the first frame update
    int N;
    bool flag = true;

    ClothData[] dataForDraw; // (N-1) * (N-1) * 12
    ComputeBuffer cBufferDataForDraw;
    Cloth cloth;
    GameObject sphere;

    // Sphere运动相关变量
    private float sphereDirection = 1.0f; // 运动方向
    private Vector3 initialSpherePosition; // 初始位置

    public Material mainMaterial;
    void Start()
    {
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.parent = this.transform;
        
        // 基于ClothTestUnit的transform计算球体位置（使用本地坐标）
        Vector3 sphereLocalPos = new Vector3(SimulationSettings.ballCenter.x, SimulationSettings.ballCenter.y, SimulationSettings.ballCenter.z);
        sphere.transform.localPosition = sphereLocalPos;
        sphere.transform.localScale = new Vector3(SimulationSettings.ballRadius * 2.0f, SimulationSettings.ballRadius * 2.0f, SimulationSettings.ballRadius * 2.0f);

        // 记录初始位置（本地坐标）
        initialSpherePosition = sphere.transform.localPosition;

        // 使用本地坐标系初始化布料（相对于ClothTestUnit的原点）
        float3 clothStartPos = new float3(0.0f, 0.0f, 0.0f);
        cloth = new Cloth(32, clothStartPos, 0.2f, 1.0f);

        N = cloth.N;

        dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
        dataForDraw = cloth.getColthDrawData();

        cBufferDataForDraw = new ComputeBuffer((N - 1) * (N - 1) * 12, 12);
        cBufferDataForDraw.SetData(dataForDraw, 0, 0, (N - 1) * (N - 1) * 12);
    }

    // Update is called once per frame
    void Update()
    {
        // Sphere沿x轴自动运动
        MoveSphere();

        if (Input.GetKeyUp(KeyCode.Space))
        {
            flag = !flag;
        }
        if (flag)
        {
            //flag = false;
            // 直接使用本地坐标传递给物理模拟
            float3 ballPosForPhysics = new float3(sphere.transform.localPosition.x, sphere.transform.localPosition.y, sphere.transform.localPosition.z);
            
            cloth.updateStep(ballPosForPhysics);
            dataForDraw = cloth.getColthDrawData();
            cBufferDataForDraw.SetData(dataForDraw, 0, 0, (N - 1) * (N - 1) * 12);
        }
    }

    void MoveSphere()
    {
        // 计算新的x位置（本地坐标）
        float newX = sphere.transform.localPosition.x + sphereDirection * SimulationSettings.sphereSpeed * Time.deltaTime;
        
        // 检查边界并反转方向
        if (newX > initialSpherePosition.x + SimulationSettings.sphereRange)
        {
            newX = initialSpherePosition.x + SimulationSettings.sphereRange;
            sphereDirection = -1.0f;
        }
        else if (newX < initialSpherePosition.x - SimulationSettings.sphereRange)
        {
            newX = initialSpherePosition.x - SimulationSettings.sphereRange;
            sphereDirection = 1.0f;
        }

        // 更新sphere位置（本地坐标）
        sphere.transform.localPosition = new Vector3(newX, sphere.transform.localPosition.y, sphere.transform.localPosition.z);
        
        // 直接使用本地坐标传递给物理模拟
        float3 ballPosForPhysics = new float3(sphere.transform.localPosition.x, sphere.transform.localPosition.y, sphere.transform.localPosition.z);
        
        // 同时更新SimulationSettings中的ballCenter，保持物理模拟同步
        SimulationSettings.ballCenter = ballPosForPhysics;
    }
    private void OnRenderObject()
    {
        // 设置模型矩阵，将本地坐标转换为世界坐标进行渲染
        Matrix4x4 modelMatrix = transform.localToWorldMatrix;
        mainMaterial.SetMatrix("_ModelMatrix", modelMatrix);
        mainMaterial.SetBuffer("_clothDataBuffer", cBufferDataForDraw);
        mainMaterial.SetPass(0);
        Graphics.DrawProceduralNow(MeshTopology.Lines, (cloth.N - 1) * (cloth.N - 1) * 12);
    }

    private void OnDestroy()
    {
        if (cBufferDataForDraw != null)
            cBufferDataForDraw.Release();
    }
}
