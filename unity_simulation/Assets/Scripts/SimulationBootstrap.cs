using UnityEngine;
using ROS2;

/// <summary>
/// Must be placed on a single persistent GameObject in the scene.
/// Initialises the ROS2 context before any other script needs it.
///
/// Usage:
///   1. Create an empty GameObject named "ROS2Manager" in the scene.
///   2. Attach both this script and <see cref="ROS2UnityComponent"/> to it.
///   3. Ensure this script executes before WheelController
///      (Edit → Project Settings → Script Execution Order).
/// </summary>
[RequireComponent(typeof(ROS2UnityComponent))]
public class SimulationBootstrap : MonoBehaviour
{
    private void Awake()
    {
        DontDestroyOnLoad(gameObject);
        Debug.Log("[SimulationBootstrap] ROS2 Unity context initialised.");
    }
}
