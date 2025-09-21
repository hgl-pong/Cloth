using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
public class dispathcer : MonoBehaviour
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
        sphere.transform.position = new Vector3(SimulationSettings.ballCenter.x, SimulationSettings.ballCenter.y, SimulationSettings.ballCenter.z);
        sphere.transform.localScale = new Vector3(SimulationSettings.ballRadius * 2.0f, SimulationSettings.ballRadius * 2.0f, SimulationSettings.ballRadius * 2.0f);

        // 记录初始位置
        initialSpherePosition = sphere.transform.position;

        cloth = new Cloth(32, new float3(0.0f, 0.0f, 0.0f), 0.2f, 1.0f);

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
            cloth.updateStep(sphere.transform.position);
            dataForDraw = cloth.getColthDrawData();
            cBufferDataForDraw.SetData(dataForDraw, 0, 0, (N - 1) * (N - 1) * 12);
        }
    }

    void MoveSphere()
    {
        // 计算新的x位置
        float newX = sphere.transform.position.x + sphereDirection * SimulationSettings.sphereSpeed * Time.deltaTime;
        
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

        // 更新sphere位置
        sphere.transform.position = new Vector3(newX, sphere.transform.position.y, sphere.transform.position.z);
        
        // 同时更新SimulationSettings中的ballCenter，保持物理模拟同步
        SimulationSettings.ballCenter = new float3(sphere.transform.position.x, sphere.transform.position.y, sphere.transform.position.z);
    }
    private void OnRenderObject()
    {
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
