using System;
using UnityEngine;
using ROS2;
using custom_msgs.msg;
using nav_msgs.msg;
using geometry_msgs.msg;
using std_msgs.msg;

/// <summary>
/// Unity simulation script that:
///   1. Subscribes to /cmd_rpm (custom_msgs/WheelRPM) and drives left/right WheelColliders.
///   2. Accumulates wheel rotation to compute encoder-based Odometry.
///   3. Publishes nav_msgs/Odometry to /odom at a fixed rate.
///   4. Handles ESTOP (subscribes to /estop std_msgs/Bool) for immediate halt.
///
/// Attach to the vehicle root GameObject.
/// Assign the four WheelCollider references and the matching visual transforms in the Inspector.
/// </summary>
public class WheelController : MonoBehaviour
{
    // ─────────────────────────────────────────────
    //  Inspector fields
    // ─────────────────────────────────────────────

    [Header("Wheel Colliders")]
    [Tooltip("Front-left WheelCollider")]
    public WheelCollider wheelFL;
    [Tooltip("Front-right WheelCollider")]
    public WheelCollider wheelFR;
    [Tooltip("Rear-left WheelCollider")]
    public WheelCollider wheelRL;
    [Tooltip("Rear-right WheelCollider")]
    public WheelCollider wheelRR;

    [Header("Wheel Visual Meshes (optional)")]
    public Transform wheelFLMesh;
    public Transform wheelFRMesh;
    public Transform wheelRLMesh;
    public Transform wheelRRMesh;

    [Header("Vehicle Parameters")]
    [Tooltip("Wheel radius in metres")]
    public float wheelRadius = 0.1651f;          // ~6.5 inch wheel
    [Tooltip("Track width (left-right wheel centre distance) in metres")]
    public float trackWidth  = 0.5f;
    [Tooltip("Max motor torque per wheel in Nm")]
    public float maxTorque   = 5.0f;
    [Tooltip("Encoder pulses per wheel revolution")]
    public int encoderPPR    = 1024;

    [Header("ROS2 Topics")]
    public string cmdRpmTopic  = "/cmd_rpm";
    public string odomTopic    = "/odom";
    public string estopTopic   = "/estop";
    public string odomFrameId  = "odom";
    public string baseFrameId  = "base_link";

    [Header("Odometry Publish Rate")]
    [Tooltip("How many times per second /odom is published")]
    public float odomPublishHz = 50.0f;

    // ─────────────────────────────────────────────
    //  Private state
    // ─────────────────────────────────────────────

    // ROS2 node and endpoints
    private ROS2UnityComponent _ros2Unity;
    private ROS2Node           _ros2Node;
    private ISubscription<WheelRPM>  _cmdRpmSub;
    private ISubscription<Bool>      _estopSub;
    private IPublisher<Odometry>     _odomPub;

    // Commanded RPM (thread-safe via volatile)
    private volatile float _cmdLeftRpm  = 0f;
    private volatile float _cmdRightRpm = 0f;
    private volatile bool  _estop       = false;

    // Odometry state
    private double _odomX   = 0.0;
    private double _odomY   = 0.0;
    private double _odomYaw = 0.0;

    // Accumulated wheel angles (degrees) from previous FixedUpdate
    private float _prevAngleL = 0f;
    private float _prevAngleR = 0f;

    // Publish timer
    private float _odomTimer = 0f;

    // ROS2 time helper
    private uint _seq = 0;

    // ─────────────────────────────────────────────
    //  Unity lifecycle
    // ─────────────────────────────────────────────

    private void Start()
    {
        // Grab the ROS2UnityComponent that must exist on the same (or parent) GameObject
        _ros2Unity = GetComponent<ROS2UnityComponent>()
                     ?? GetComponentInParent<ROS2UnityComponent>();

        if (_ros2Unity == null)
        {
            Debug.LogError("[WheelController] ROS2UnityComponent not found. " +
                           "Add it to the same GameObject.");
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        // ROS2 node initialisation is deferred until ROS2UnityComponent is ready
        if (_ros2Node == null && _ros2Unity.Ok())
        {
            _ros2Node = _ros2Unity.CreateNode("unity_wheel_controller");

            // Subscriber: /cmd_rpm
            _cmdRpmSub = _ros2Node.CreateSubscription<WheelRPM>(
                cmdRpmTopic,
                OnCmdRpm);

            // Subscriber: /estop
            _estopSub = _ros2Node.CreateSubscription<Bool>(
                estopTopic,
                OnEstop);

            // Publisher: /odom
            _odomPub = _ros2Node.CreatePublisher<Odometry>(odomTopic);

            // Seed previous wheel angles
            _prevAngleL = GetLeftWheelAngle();
            _prevAngleR = GetRightWheelAngle();

            Debug.Log("[WheelController] ROS2 node ready.");
        }

        // Update visual wheel meshes every frame
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

        // Throttled odometry publish
        _odomTimer += Time.fixedDeltaTime;
        if (_odomTimer >= 1f / odomPublishHz)
        {
            _odomTimer = 0f;
            PublishOdometry();
        }
    }

    private void OnDestroy()
    {
        _ros2Node?.Dispose();
    }

    // ─────────────────────────────────────────────
    //  ROS2 callbacks  (called from ROS2 thread)
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
            Debug.LogWarning("[WheelController] ESTOP received – halting vehicle.");
    }

    // ─────────────────────────────────────────────
    //  Wheel control
    // ─────────────────────────────────────────────

    private void ApplyWheelControl()
    {
        if (_estop)
        {
            // Immediate halt: cut motor torque and apply full brakes
            foreach (var wc in new[] { wheelFL, wheelFR, wheelRL, wheelRR })
            {
                wc.motorTorque  = 0f;
                wc.brakeTorque  = Mathf.Infinity;
            }
            return;
        }

        // Release brakes (set to a small residual so the vehicle doesn't roll
        // uncontrollably on slopes when idle)
        float idleBrake = (_cmdLeftRpm == 0f && _cmdRightRpm == 0f) ? maxTorque : 0f;

        // Convert RPM → angular velocity (rad/s) → tangential speed (m/s)
        float speedL = RpmToWheelSpeed(_cmdLeftRpm);
        float speedR = RpmToWheelSpeed(_cmdRightRpm);

        // Compute torque proportional to the difference between target and
        // current speed.  A simple proportional controller is sufficient for
        // simulation purposes.
        float torqueL = ComputeTorque(wheelFL, wheelRL, speedL);
        float torqueR = ComputeTorque(wheelFR, wheelRR, speedR);

        // Left side
        wheelFL.motorTorque = torqueL;
        wheelRL.motorTorque = torqueL;
        wheelFL.brakeTorque = idleBrake;
        wheelRL.brakeTorque = idleBrake;

        // Right side
        wheelFR.motorTorque = torqueR;
        wheelRR.motorTorque = torqueR;
        wheelFR.brakeTorque = idleBrake;
        wheelRR.brakeTorque = idleBrake;
    }

    /// <summary>Simple P-controller: torque proportional to speed error.</summary>
    private float ComputeTorque(WheelCollider front, WheelCollider rear, float targetSpeed)
    {
        float currentSpeed = (front.rpm + rear.rpm) * 0.5f * (2f * Mathf.PI * wheelRadius / 60f);
        float error = targetSpeed - currentSpeed;
        float torque = Mathf.Clamp(error * maxTorque, -maxTorque, maxTorque);
        return torque;
    }

    private static float RpmToWheelSpeed(float rpm)
        => rpm * 2f * Mathf.PI * 0.1651f / 60f;   // v = ω·r,  ω = rpm·2π/60

    // ─────────────────────────────────────────────
    //  Encoder-based Odometry
    // ─────────────────────────────────────────────

    /// <summary>
    /// Returns the average cumulative rotation angle (degrees) of the left side wheels.
    /// WheelCollider.rpm can be integrated over time instead of using cumulative angle;
    /// here we use the collider's rotation through the physics step.
    /// </summary>
    private float GetLeftWheelAngle()
    {
        // Unity doesn't expose cumulative wheel angle directly.
        // We compute it by integrating RPM × fixedDeltaTime.
        return (wheelFL.rpm + wheelRL.rpm) * 0.5f * 6f * Time.fixedDeltaTime;
        // 6 = 360 degrees / 60 seconds
    }

    private float GetRightWheelAngle()
    {
        return (wheelFR.rpm + wheelRR.rpm) * 0.5f * 6f * Time.fixedDeltaTime;
    }

    private void UpdateOdometry()
    {
        // Angular displacement this step (degrees → radians)
        float dAngleL = GetLeftWheelAngle()  * Mathf.Deg2Rad;
        float dAngleR = GetRightWheelAngle() * Mathf.Deg2Rad;

        // Arc lengths
        double dL = wheelRadius * dAngleL;
        double dR = wheelRadius * dAngleR;

        // Differential-drive kinematics
        double dCenter  = (dL + dR) * 0.5;
        double dTheta   = (dR - dL) / trackWidth;

        _odomYaw += dTheta;
        _odomX   += dCenter * Math.Cos(_odomYaw);
        _odomY   += dCenter * Math.Sin(_odomYaw);
    }

    // ─────────────────────────────────────────────
    //  Odometry publisher
    // ─────────────────────────────────────────────

    private void PublishOdometry()
    {
        if (_odomPub == null) return;

        var msg = new Odometry();

        // Header
        msg.header.frame_id = odomFrameId;
        FillStamp(ref msg.header.stamp);
        msg.child_frame_id = baseFrameId;

        // Pose
        msg.pose.pose.position.x = _odomX;
        msg.pose.pose.position.y = _odomY;
        msg.pose.pose.position.z = 0.0;

        double halfYaw = _odomYaw * 0.5;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = Math.Sin(halfYaw);
        msg.pose.pose.orientation.w = Math.Cos(halfYaw);

        // Twist (current body velocity)
        float avgRpmL = (wheelFL.rpm + wheelRL.rpm) * 0.5f;
        float avgRpmR = (wheelFR.rpm + wheelRR.rpm) * 0.5f;
        double vL = wheelRadius * avgRpmL * 2.0 * Math.PI / 60.0;
        double vR = wheelRadius * avgRpmR * 2.0 * Math.PI / 60.0;

        msg.twist.twist.linear.x  = (vL + vR) * 0.5;
        msg.twist.twist.angular.z = (vR - vL) / trackWidth;

        // Covariance: diagonal, tuned loosely for simulation
        msg.pose.covariance[0]  = 0.001;   // x
        msg.pose.covariance[7]  = 0.001;   // y
        msg.pose.covariance[35] = 0.01;    // yaw
        msg.twist.covariance[0]  = 0.001;  // vx
        msg.twist.covariance[35] = 0.01;   // omega

        _odomPub.Publish(msg);
    }

    // ─────────────────────────────────────────────
    //  Helpers
    // ─────────────────────────────────────────────

    private void FillStamp(ref builtin_interfaces.msg.Time stamp)
    {
        double now = Time.timeAsDouble;
        stamp.sec     = (int)Math.Floor(now);
        stamp.nanosec = (uint)((now - Math.Floor(now)) * 1e9);
    }

    private static void UpdateWheelMesh(WheelCollider col, Transform mesh)
    {
        if (col == null || mesh == null) return;
        col.GetWorldPose(out Vector3 pos, out Quaternion rot);
        mesh.SetPositionAndRotation(pos, rot);
    }
}
