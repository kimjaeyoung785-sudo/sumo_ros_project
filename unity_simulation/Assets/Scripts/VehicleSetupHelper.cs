using UnityEngine;

/// <summary>
/// Editor helper that auto-configures WheelCollider friction and stiffness values
/// suitable for a small differential-drive robot.
///
/// Attach to the vehicle root and call <see cref="Configure"/> from the Inspector
/// context menu (right-click the component header).
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class VehicleSetupHelper : MonoBehaviour
{
    [Header("Rigidbody")]
    public float vehicleMass       = 15f;   // kg
    public float centerOfMassY     = 0.05f; // metres above wheel axle centre

    [Header("WheelColliders to configure")]
    public WheelCollider[] wheels;

    [Header("Wheel Friction (WheelFrictionCurve)")]
    public float forwardStiffness  = 1.5f;
    public float sidewaysStiffness = 2.0f;

    [ContextMenu("Configure Vehicle")]
    public void Configure()
    {
        var rb = GetComponent<Rigidbody>();
        rb.mass = vehicleMass;
        rb.centerOfMass = new Vector3(0f, centerOfMassY, 0f);

        foreach (var wc in wheels)
        {
            if (wc == null) continue;

            wc.radius            = 0.1651f;
            wc.suspensionDistance = 0.02f;

            var spring = wc.suspensionSpring;
            spring.spring   = 35000f;
            spring.damper   = 4500f;
            spring.targetPosition = 0.5f;
            wc.suspensionSpring = spring;

            wc.forceAppPointDistance = 0f;

            var fFwd = BuildFriction(forwardStiffness);
            var fSide = BuildFriction(sidewaysStiffness);
            wc.forwardFriction  = fFwd;
            wc.sidewaysFriction = fSide;
        }

        Debug.Log("[VehicleSetupHelper] WheelColliders configured.");
    }

    private static WheelFrictionCurve BuildFriction(float stiffness)
    {
        var c = new WheelFrictionCurve
        {
            extremumSlip   = 0.4f,
            extremumValue  = 1.0f,
            asymptoteSlip  = 0.8f,
            asymptoteValue = 0.5f,
            stiffness      = stiffness
        };
        return c;
    }
}
