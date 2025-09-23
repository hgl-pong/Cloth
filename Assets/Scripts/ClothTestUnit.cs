using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
public class ClothTestUnit : MonoBehaviour
{
    // Start is called before the first frame update
    int N;
    bool flag = true;

    [Header("Cloth Setup")]
    [SerializeField] ClothSimulationType clothType = ClothSimulationType.PBD;
    [SerializeField, Min(2)] int clothResolution = 32;
    [SerializeField, Min(0.01f)] float clothNodeStep = 0.2f;
    [SerializeField, Min(0.001f)] float clothDensity = 1.0f;
        [SerializeField] ClothIntegrationMethod integrationMethod = SimulationSettings.massSpringIntegrationMethod;

    ClothData[] dataForDraw; // (N-1) * (N-1) * 12
    ComputeBuffer cBufferDataForDraw;
    ICloth cloth;
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
        int resolvedResolution = clothResolution > 1 ? clothResolution : SimulationSettings.clothResolution;
        float resolvedNodeStep = clothNodeStep > 0.0f ? clothNodeStep : SimulationSettings.clothNodeStep;
        float resolvedDensity = clothDensity > 0.0f ? clothDensity : SimulationSettings.clothDensity;
        cloth = ClothFactory.CreateCloth(clothType, resolvedResolution, clothStartPos, resolvedNodeStep, resolvedDensity, integrationMethod);

        N = cloth.Resolution;

        dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
        dataForDraw = cloth.GetClothDrawData();

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
            
            cloth.UpdateStep(ballPosForPhysics);
            dataForDraw = cloth.GetClothDrawData();
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
        Graphics.DrawProceduralNow(MeshTopology.Lines, (cloth.Resolution - 1) * (cloth.Resolution - 1) * 12);
    }

    private void OnDestroy()
    {
        if (cBufferDataForDraw != null)
            cBufferDataForDraw.Release();
    }


    void OnValidate()
    {
        clothResolution = math.max(clothResolution, 2);
        clothNodeStep = math.max(0.001f, clothNodeStep);
        clothDensity = math.max(0.0001f, clothDensity);
    }

}
