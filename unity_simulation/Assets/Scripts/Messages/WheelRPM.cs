// Auto-generated ROS2 message binding for custom_msgs/WheelRPM
// Compatible with ros2-for-unity (Robotec.ai)

using ROS2;

namespace custom_msgs.msg
{
    /// <summary>
    /// Custom ROS2 message: custom_msgs/WheelRPM
    /// Carries independent RPM commands for left and right wheels.
    /// </summary>
    public class WheelRPM : Message
    {
        // ROS2 message type string
        public const string k_RosMessageType = "custom_msgs/msg/WheelRPM";

        /// <summary>Left wheel target RPM (positive = forward)</summary>
        public float left_rpm;

        /// <summary>Right wheel target RPM (positive = forward)</summary>
        public float right_rpm;

        public WheelRPM()
        {
            left_rpm = 0.0f;
            right_rpm = 0.0f;
        }

        public override string RosMessageType => k_RosMessageType;
    }
}
