using System.Collections;
using UnityEngine;
using ROS2;
using custom_msgs.msg;

/// <summary>
/// Automated scenario tester that publishes a sequence of /cmd_rpm commands
/// to verify basic vehicle behaviours without requiring an external ROS2 node.
///
/// Attach to the same GameObject as <see cref="WheelController"/> and enable
/// <see cref="runOnStart"/> to execute the test sequence automatically.
///
/// Scenarios exercised:
///   1. Straight forward  (equal positive RPM on both sides)
///   2. Spot turn left    (positive RPM right, negative RPM left)
///   3. Spot turn right   (positive RPM left, negative RPM right)
///   4. Slow creep forward (low equal RPM)
///   5. Full stop
/// </summary>
[RequireComponent(typeof(WheelController))]
public class ScenarioTester : MonoBehaviour
{
    [Header("Scenario Settings")]
    public bool runOnStart = false;

    [Tooltip("Normal drive RPM for test scenarios")]
    public float driveRpm = 60f;
    [Tooltip("Slow creep RPM")]
    public float creepRpm = 15f;
    [Tooltip("Duration of each scenario phase in seconds")]
    public float phaseDuration = 3f;

    private ROS2UnityComponent _ros2Unity;
    private ROS2Node            _testNode;
    private IPublisher<WheelRPM> _cmdPub;

    private void Start()
    {
        if (!runOnStart) return;

        _ros2Unity = GetComponent<ROS2UnityComponent>()
                     ?? GetComponentInParent<ROS2UnityComponent>();

        if (_ros2Unity == null)
        {
            Debug.LogError("[ScenarioTester] ROS2UnityComponent not found.");
            return;
        }

        StartCoroutine(WaitAndRun());
    }

    private IEnumerator WaitAndRun()
    {
        // Wait until ROS2 is ready
        while (!_ros2Unity.Ok()) yield return null;

        _testNode = _ros2Unity.CreateNode("unity_scenario_tester");
        _cmdPub   = _testNode.CreatePublisher<WheelRPM>("/cmd_rpm");

        yield return new WaitForSeconds(1f); // brief settle time

        Debug.Log("[ScenarioTester] === Starting scenario sequence ===");

        yield return RunPhase("Straight forward",   driveRpm,  driveRpm);
        yield return RunPhase("Spot turn left",     -driveRpm, driveRpm);
        yield return RunPhase("Spot turn right",    driveRpm, -driveRpm);
        yield return RunPhase("Slow creep forward", creepRpm,  creepRpm);
        yield return RunPhase("Full stop",          0f,        0f);

        Debug.Log("[ScenarioTester] === Scenario sequence complete ===");

        _testNode.Dispose();
    }

    private IEnumerator RunPhase(string name, float leftRpm, float rightRpm)
    {
        Debug.Log($"[ScenarioTester] Phase: {name}  L={leftRpm} R={rightRpm} RPM");

        float elapsed = 0f;
        while (elapsed < phaseDuration)
        {
            Publish(leftRpm, rightRpm);
            elapsed += Time.deltaTime;
            yield return null;
        }
    }

    private void Publish(float left, float right)
    {
        if (_cmdPub == null) return;
        _cmdPub.Publish(new WheelRPM { left_rpm = left, right_rpm = right });
    }

    private void OnDestroy()
    {
        _testNode?.Dispose();
    }
}
