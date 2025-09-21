using UnityEngine;

/// <summary>
/// 自由移动相机控制器
/// 支持鼠标旋转、键盘WASD移动、滚轮缩放等功能
/// </summary>
public class FreeCameraController : MonoBehaviour
{
    [Header("移动设置")]
    [Tooltip("相机移动速度")]
    public float moveSpeed = 5.0f;
    
    [Tooltip("快速移动倍数（按住Shift）")]
    public float fastMoveMultiplier = 2.0f;
    
    [Tooltip("慢速移动倍数（按住Ctrl）")]
    public float slowMoveMultiplier = 0.5f;

    [Header("旋转设置")]
    [Tooltip("鼠标灵敏度")]
    public float mouseSensitivity = 2.0f;
    
    [Tooltip("垂直旋转角度限制")]
    public float verticalRotationLimit = 90.0f;

    [Header("缩放设置")]
    [Tooltip("滚轮缩放速度")]
    public float scrollSpeed = 2.0f;
    
    [Tooltip("最小移动速度")]
    public float minMoveSpeed = 0.1f;
    
    [Tooltip("最大移动速度")]
    public float maxMoveSpeed = 20.0f;

    // 私有变量
    private float rotationX = 0.0f;
    private float rotationY = 0.0f;
    private bool isMouseLookEnabled = false;

    void Start()
    {
        // 获取初始旋转角度
        Vector3 rotation = transform.eulerAngles;
        rotationX = rotation.x;
        rotationY = rotation.y;
    }

    void Update()
    {
        HandleMouseLook();
        HandleMovement();
        HandleScrollZoom();
        HandleMouseToggle();
    }

    /// <summary>
    /// 处理鼠标视角旋转
    /// </summary>
    void HandleMouseLook()
    {
        // 右键按下时启用鼠标视角
        if (Input.GetMouseButtonDown(1))
        {
            isMouseLookEnabled = true;
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
        
        // 右键释放时禁用鼠标视角
        if (Input.GetMouseButtonUp(1))
        {
            isMouseLookEnabled = false;
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        // 鼠标视角控制
        if (isMouseLookEnabled)
        {
            float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
            float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;

            rotationY += mouseX;
            rotationX -= mouseY;

            // 限制垂直旋转角度
            rotationX = Mathf.Clamp(rotationX, -verticalRotationLimit, verticalRotationLimit);

            transform.rotation = Quaternion.Euler(rotationX, rotationY, 0);
        }
    }

    /// <summary>
    /// 处理键盘移动
    /// </summary>
    void HandleMovement()
    {
        float currentMoveSpeed = moveSpeed;

        // 检查修饰键
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            currentMoveSpeed *= fastMoveMultiplier;
        }
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
        {
            currentMoveSpeed *= slowMoveMultiplier;
        }

        // 获取输入
        float horizontal = Input.GetAxis("Horizontal"); // A/D 或 左/右箭头
        float vertical = Input.GetAxis("Vertical");     // W/S 或 上/下箭头
        float upDown = 0f;

        // Q/E 键控制上下移动
        if (Input.GetKey(KeyCode.Q))
            upDown = -1f;
        else if (Input.GetKey(KeyCode.E))
            upDown = 1f;

        // 计算移动向量
        Vector3 moveDirection = new Vector3(horizontal, upDown, vertical);
        
        // 转换到世界坐标系
        Vector3 move = transform.TransformDirection(moveDirection) * currentMoveSpeed * Time.deltaTime;
        
        // 应用移动
        transform.position += move;
    }

    /// <summary>
    /// 处理滚轮缩放（调整移动速度）
    /// </summary>
    void HandleScrollZoom()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0f)
        {
            moveSpeed += scroll * scrollSpeed;
            moveSpeed = Mathf.Clamp(moveSpeed, minMoveSpeed, maxMoveSpeed);
        }
    }

    /// <summary>
    /// 处理鼠标切换
    /// </summary>
    void HandleMouseToggle()
    {
        // ESC键释放鼠标
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            isMouseLookEnabled = false;
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
    }

    /// <summary>
    /// 重置相机到指定位置和旋转
    /// </summary>
    public void ResetCamera(Vector3 position, Vector3 rotation)
    {
        transform.position = position;
        transform.rotation = Quaternion.Euler(rotation);
        rotationX = rotation.x;
        rotationY = rotation.y;
    }

    /// <summary>
    /// 设置相机移动速度
    /// </summary>
    public void SetMoveSpeed(float speed)
    {
        moveSpeed = Mathf.Clamp(speed, minMoveSpeed, maxMoveSpeed);
    }

    void OnGUI()
    {
        // 显示控制提示
        GUI.Label(new Rect(10, 10, 300, 20), "右键拖拽: 旋转视角");
        GUI.Label(new Rect(10, 30, 300, 20), "WASD: 移动相机");
        GUI.Label(new Rect(10, 50, 300, 20), "Q/E: 上下移动");
        GUI.Label(new Rect(10, 70, 300, 20), "Shift: 快速移动");
        GUI.Label(new Rect(10, 90, 300, 20), "Ctrl: 慢速移动");
        GUI.Label(new Rect(10, 110, 300, 20), "滚轮: 调整移动速度");
        GUI.Label(new Rect(10, 130, 300, 20), "ESC: 释放鼠标");
        GUI.Label(new Rect(10, 150, 300, 20), $"当前移动速度: {moveSpeed:F1}");
    }
}