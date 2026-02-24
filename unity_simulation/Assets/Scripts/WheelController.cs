using System;
using UnityEngine;
using ROS2;
using custom_msgs.msg;
using nav_msgs.msg;
using std_msgs.msg;

/// <summary>
/// ROS2 ↔ Unity 차량 물리 제어 스크립트.
///
/// [동작 흐름]
///   1. /cmd_rpm (custom_msgs/WheelRPM) 구독
///      → 좌·우 WheelCollider에 독립 RPM 토크 적용
///   2. WheelCollider RPM 적분 → 차동구동 키네마틱스
///      → nav_msgs/Odometry 를 /odom 으로 발행 (odomPublishHz)
///   3. /estop (std_msgs/Bool) true 수신
///      → 즉각 motorTorque=0, brakeTorque=estopBrakeTorque 적용
///
/// [제어 상태 3단계]
///   ESTOP   : motorTorque=0,  brakeTorque=estopBrakeTorque (강제 잠금)
///   IDLE    : motorTorque=0,  brakeTorque=idleBrakeTorque  (정지 유지)
///   DRIVING : motorTorque=계산값, brakeTorque=0
/// </summary>
public class WheelController : MonoBehaviour
{
    // ─────────────────────────────────────────────
    //  Inspector
    // ─────────────────────────────────────────────

    [Header("Wheel Colliders")]
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;

    [Header("Wheel Visual Meshes (optional)")]
    public Transform wheelFLMesh;
    public Transform wheelFRMesh;
    public Transform wheelRLMesh;
    public Transform wheelRRMesh;

    [Header("Vehicle Parameters")]
    [Tooltip("Wheel radius (m)")]
    public float wheelRadius = 0.1651f;
    [Tooltip("Left-right wheel centre distance (m)")]
    public float trackWidth  = 0.5f;
    [Tooltip("Peak motor torque per wheel (Nm)")]
    public float maxMotorTorque  = 5.0f;
    [Tooltip("Brake torque applied when RPM command is 0 (Nm) – holds vehicle on slope")]
    public float idleBrakeTorque = 300.0f;
    [Tooltip("Brake torque applied on ESTOP (Nm) – instant hard lock")]
    public float estopBrakeTorque = 1000.0f;

    [Header("ROS2 Topics")]
    public string cmdRpmTopic = "/cmd_rpm";
    public string odomTopic   = "/odom";
    public string estopTopic  = "/estop";
    public string odomFrameId = "odom";
    public string baseFrameId = "base_link";

    [Header("Odometry Publish Rate (Hz)")]
    public float odomPublishHz = 50.0f;

    // ─────────────────────────────────────────────
    //  Private state
    // ─────────────────────────────────────────────

    private ROS2UnityComponent       _ros2Unity;
    private ROS2Node                 _ros2Node;
    private ISubscription<WheelRPM>  _cmdRpmSub;
    private ISubscription<Bool>      _estopSub;
    private IPublisher<Odometry>     _odomPub;

    // ROS2 콜백 → FixedUpdate 간 공유 (volatile = 원자적 읽기/쓰기 보장)
    private volatile float _cmdLeftRpm  = 0f;
    private volatile float _cmdRightRpm = 0f;
    private volatile bool  _estop       = false;

    // 오도메트리 누적값
    private double _odomX   = 0.0;
    private double _odomY   = 0.0;
    private double _odomYaw = 0.0;

    private float _odomTimer = 0f;

    // ─────────────────────────────────────────────
    //  Unity lifecycle
    // ─────────────────────────────────────────────

    private void Start()
    {
        _ros2Unity = GetComponent<ROS2UnityComponent>()
                     ?? GetComponentInParent<ROS2UnityComponent>();

        if (_ros2Unity == null)
        {
            Debug.LogError("[WheelController] ROS2UnityComponent not found.");
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        // ROS2 준비 완료 후 노드 한 번만 생성
        if (_ros2Node == null && _ros2Unity.Ok())
        {
            _ros2Node = _ros2Unity.CreateNode("unity_wheel_controller");

            _cmdRpmSub = _ros2Node.CreateSubscription<WheelRPM>(cmdRpmTopic, OnCmdRpm);
            _estopSub  = _ros2Node.CreateSubscription<Bool>(estopTopic, OnEstop);
            _odomPub   = _ros2Node.CreatePublisher<Odometry>(odomTopic);

            Debug.Log("[WheelController] ROS2 node ready.");
        }

        UpdateWheelMesh(wheelFL, wheelFLMesh);
        UpdateWheelMesh(wheelFR, wheelFRMesh);
        UpdateWheelMesh(wheelRL, wheelRLMesh);
        UpdateWheelMesh(wheelRR, wheelRRMesh);
    }

    private void FixedUpdate()
    {
        if (_ros2Node == null) return;

        ApplyWheelControl();
        UpdateOdometry();

        _odomTimer += Time.fixedDeltaTime;
        if (_odomTimer >= 1f / odomPublishHz)
        {
            _odomTimer = 0f;
            PublishOdometry();
        }
    }

    private void OnDestroy() => _ros2Node?.Dispose();

    // ─────────────────────────────────────────────
    //  ROS2 콜백
    // ─────────────────────────────────────────────

    private void OnCmdRpm(WheelRPM msg)
    {
        _cmdLeftRpm  = msg.left_rpm;
        _cmdRightRpm = msg.right_rpm;
    }

    private void OnEstop(Bool msg)
    {
        _estop = msg.data;
        if (_estop)
            Debug.LogWarning("[WheelController] ESTOP! 강제 정지 적용.");
    }

    // ─────────────────────────────────────────────
    //  제어 상태 3단계
    // ─────────────────────────────────────────────

    private void ApplyWheelControl()
    {
        // ── 상태 1: ESTOP ──────────────────────────────
        if (_estop)
        {
            SetAllWheels(motorTorque: 0f, brakeTorque: estopBrakeTorque);
            return;
        }

        bool isIdle = (_cmdLeftRpm == 0f && _cmdRightRpm == 0f);

        // ── 상태 2: IDLE (RPM 명령 = 0) ────────────────
        if (isIdle)
        {
            // motorTorque를 먼저 0으로 해제한 뒤 brakeTorque 적용
            // (둘을 동시에 0이 아닌 값으로 설정하면 Unity 물리가 충돌함)
            SetAllWheels(motorTorque: 0f, brakeTorque: idleBrakeTorque);
            return;
        }

        // ── 상태 3: DRIVING ────────────────────────────
        // brakeTorque를 반드시 0으로 해제한 뒤 motorTorque 적용
        float torqueL = ComputeTorque(wheelFL, wheelRL, RpmToSpeed(_cmdLeftRpm));
        float torqueR = ComputeTorque(wheelFR, wheelRR, RpmToSpeed(_cmdRightRpm));

        SetSideWheels(wheelFL, wheelRL, torqueL, brakeTorque: 0f);
        SetSideWheels(wheelFR, wheelRR, torqueR, brakeTorque: 0f);
    }

    /// <summary>P-제어: 목표 선속도와 현재 선속도 오차 → 토크</summary>
    private float ComputeTorque(WheelCollider front, WheelCollider rear, float targetSpeed)
    {
        float currentSpeed = WheelAvgRpm(front, rear) * (2f * Mathf.PI * wheelRadius / 60f);
        float error  = targetSpeed - currentSpeed;
        return Mathf.Clamp(error * maxMotorTorque, -maxMotorTorque, maxMotorTorque);
    }

    /// <summary>RPM → 접선 선속도 (m/s)</summary>
    private float RpmToSpeed(float rpm) => rpm * 2f * Mathf.PI * wheelRadius / 60f;

    private static float WheelAvgRpm(WheelCollider a, WheelCollider b)
        => (a.rpm + b.rpm) * 0.5f;

    private void SetAllWheels(float motorTorque, float brakeTorque)
    {
        foreach (var wc in new[] { wheelFL, wheelFR, wheelRL, wheelRR })
        {
            wc.motorTorque = motorTorque;
            wc.brakeTorque = brakeTorque;
        }
    }

    private static void SetSideWheels(WheelCollider front, WheelCollider rear,
                                      float motorTorque, float brakeTorque)
    {
        front.motorTorque = motorTorque;
        front.brakeTorque = brakeTorque;
        rear.motorTorque  = motorTorque;
        rear.brakeTorque  = brakeTorque;
    }

    // ─────────────────────────────────────────────
    //  엔코더 기반 Odometry
    // ─────────────────────────────────────────────

    private void UpdateOdometry()
    {
        // WheelCollider.rpm × fixedDeltaTime → 이번 스텝 회전각(rad)
        float dAngleL = WheelAvgRpm(wheelFL, wheelRL) * (2f * Mathf.PI / 60f) * Time.fixedDeltaTime;
        float dAngleR = WheelAvgRpm(wheelFR, wheelRR) * (2f * Mathf.PI / 60f) * Time.fixedDeltaTime;

        // 이동 호 길이
        double dL = wheelRadius * dAngleL;
        double dR = wheelRadius * dAngleR;

        // 차동구동 키네마틱스
        double dCenter = (dL + dR) * 0.5;
        double dTheta  = (dR - dL) / trackWidth;

        _odomYaw += dTheta;
        _odomX   += dCenter * Math.Cos(_odomYaw);
        _odomY   += dCenter * Math.Sin(_odomYaw);
    }

    // ─────────────────────────────────────────────
    //  Odometry 발행
    // ─────────────────────────────────────────────

    private void PublishOdometry()
    {
        if (_odomPub == null) return;

        var msg = new Odometry();

        // Header
        msg.header.frame_id = odomFrameId;
        double now = Time.timeAsDouble;
        msg.header.stamp.sec     = (int)Math.Floor(now);
        msg.header.stamp.nanosec = (uint)((now - Math.Floor(now)) * 1e9);
        msg.child_frame_id = baseFrameId;

        // Pose
        msg.pose.pose.position.x = _odomX;
        msg.pose.pose.position.y = _odomY;
        msg.pose.pose.position.z = 0.0;

        double halfYaw = _odomYaw * 0.5;
        msg.pose.pose.orientation.z = Math.Sin(halfYaw);
        msg.pose.pose.orientation.w = Math.Cos(halfYaw);

        // Twist
        double vL = wheelRadius * WheelAvgRpm(wheelFL, wheelRL) * 2.0 * Math.PI / 60.0;
        double vR = wheelRadius * WheelAvgRpm(wheelFR, wheelRR) * 2.0 * Math.PI / 60.0;
        msg.twist.twist.linear.x  = (vL + vR) * 0.5;
        msg.twist.twist.angular.z = (vR - vL) / trackWidth;

        // Covariance (대각선만 설정)
        msg.pose.covariance[0]   = 0.001;  // x
        msg.pose.covariance[7]   = 0.001;  // y
        msg.pose.covariance[35]  = 0.01;   // yaw
        msg.twist.covariance[0]  = 0.001;  // vx
        msg.twist.covariance[35] = 0.01;   // omega

        _odomPub.Publish(msg);
    }

    // ─────────────────────────────────────────────
    //  헬퍼
    // ─────────────────────────────────────────────

    private static void UpdateWheelMesh(WheelCollider col, Transform mesh)
    {
        if (col == null || mesh == null) return;
        col.GetWorldPose(out Vector3 pos, out Quaternion rot);
        mesh.SetPositionAndRotation(pos, rot);
    }
}
