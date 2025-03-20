#include <memory>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class AprilTagLocalization : public rclcpp::Node
{
public:
    AprilTagLocalization()
        : Node("apriltag_localization")
    {
        posePublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("localization_sub", 10);

        auto topic_callback =
            [this](tf2_msgs::msg::TFMessage::UniquePtr msg) -> void
        {
            for (const auto &transform : msg->transforms)
            {
                std::string frameId = transform.child_frame_id;
                geometry_msgs::msg::TransformStamped frameTransform = transform;

                geometry_msgs::msg::Transform absPosition = this->calculateCameraPosition(frameId, frameTransform.transform);

                // Convert quaternion to roll/pitch/yaw
                double roll, pitch, yaw;
                tf2::Quaternion quaternion;
                tf2::fromMsg(absPosition.rotation, quaternion);
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

                // Publish pose
                std_msgs::msg::Float64MultiArray pose;
                pose.data = {absPosition.translation.x, absPosition.translation.y, 180 * (std::fmod((-yaw + 2 * M_PI), (2 * M_PI))) / M_PI};
                this->posePublisher_->publish(pose);
                RCLCPP_INFO(this->get_logger(), "Absolute position: (%f, %f, %f)", pose.data[0], pose.data[1], pose.data[2]);
            }
            RCLCPP_INFO(this->get_logger(), "# of transforms: '%ld'", msg->transforms.size());
        };
        subscription_ =
            this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, topic_callback);
    }

private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;

    geometry_msgs::msg::Transform createTransform(double tX, double tY, double tZ, double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Transform transform;

        transform.translation.x = tX;
        transform.translation.y = tY;
        transform.translation.z = tZ;

        tf2::Quaternion q;

        // Set quaternion from Euler angles (roll, pitch, yaw)
        q.setRPY(roll, pitch, yaw);

        // Convert to geometry_msgs::msg::Quaternion
        transform.rotation.x = q.x();
        transform.rotation.y = q.y();
        transform.rotation.z = q.z();
        transform.rotation.w = q.w();

        return transform;
    }

    geometry_msgs::msg::Pose createPose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Pose pose;

        tf2::Quaternion q;

        // Set quaternion from Euler angles (roll, pitch, yaw)
        q.setRPY(roll, pitch, yaw);

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        // pose.orientation.x = q.x();
        // pose.orientation.y = q.y();
        // pose.orientation.z = q.z();
        // pose.orientation.w = q.w();
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        return pose;
    }

    std::map<std::string, geometry_msgs::msg::Transform> transform_map = {
        {"tag36h11:0", createTransform(0, 0, 0, 0, 0, 0)},
        {"tag36h11:1", createTransform(0, 0, 0, 0, 0, M_PI_2)}};

    geometry_msgs::msg::Transform invertTransform(const geometry_msgs::msg::Transform &transform)
    {
        // Invert the rotation using quaternion
        tf2::Quaternion q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
        tf2::Quaternion q_inverse = q.inverse();

        // Invert the translation
        tf2::Vector3 t(transform.translation.x, transform.translation.y, transform.translation.z);
        tf2::Vector3 t_inverse = -(tf2::Matrix3x3(q_inverse) * t);

        // Populate the inverted transform
        geometry_msgs::msg::Transform inverted_transform;
        inverted_transform.rotation.x = q_inverse.x();
        inverted_transform.rotation.y = q_inverse.y();
        inverted_transform.rotation.z = q_inverse.z();
        inverted_transform.rotation.w = q_inverse.w();

        inverted_transform.translation.x = t_inverse.x();
        inverted_transform.translation.y = t_inverse.y();
        inverted_transform.translation.z = t_inverse.z();

        return inverted_transform;
    }

    // Function to compute the product of two transforms
    geometry_msgs::msg::Transform multiplyTransforms(const geometry_msgs::msg::Transform &t1, const geometry_msgs::msg::Transform &t2)
    {
        // Combine rotations
        tf2::Quaternion q1(t1.rotation.x, t1.rotation.y, t1.rotation.z, t1.rotation.w);
        tf2::Quaternion q2(t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w);
        tf2::Quaternion q_combined = q1 * q2;

        // Combine translations
        tf2::Vector3 v1(t1.translation.x, t1.translation.y, t1.translation.z);
        tf2::Vector3 v2(t2.translation.x, t2.translation.y, t2.translation.z);
        tf2::Matrix3x3 rotation_matrix(q1);
        tf2::Vector3 translated_v2 = rotation_matrix * v2;
        tf2::Vector3 combined_translation = v1 + translated_v2;

        // Populate the combined transform
        geometry_msgs::msg::Transform combined_transform;
        combined_transform.rotation.x = q_combined.x();
        combined_transform.rotation.y = q_combined.y();
        combined_transform.rotation.z = q_combined.z();
        combined_transform.rotation.w = q_combined.w();

        combined_transform.translation.x = combined_translation.x();
        combined_transform.translation.y = combined_translation.y();
        combined_transform.translation.z = combined_translation.z();

        return combined_transform;
    }

    geometry_msgs::msg::Transform alignTagFrameToWorldFrame(const geometry_msgs::msg::Transform &transform)
    {
        tf2::Quaternion q_tag(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
        tf2::Matrix3x3 rotation_matrix(q_tag);

        // Create a 90-degree rotation about the y-axis (to align x -> z)
        tf2::Quaternion rotation_adjustment;
        rotation_adjustment.setRPY(M_PI_2, 0, M_PI_2); // -90 degrees about y-axis
        tf2::Quaternion adjusted_quaternion = q_tag * rotation_adjustment;

        geometry_msgs::msg::Transform adjusted_transform = transform;
        adjusted_transform.rotation.x = adjusted_quaternion.x();
        adjusted_transform.rotation.y = adjusted_quaternion.y();
        adjusted_transform.rotation.z = adjusted_quaternion.z();
        adjusted_transform.rotation.w = adjusted_quaternion.w();

        return adjusted_transform;
    }

    geometry_msgs::msg::Transform calculateCameraPosition(const std::string &tagId, const geometry_msgs::msg::Transform &T_camera_to_tag)
    {
        // Known AprilTag absolute transform (in the world frame)
        geometry_msgs::msg::Transform T_tag_absolute = transform_map[tagId];

        // Compute the inverse of the camera-to-tag transform
        geometry_msgs::msg::Transform T_tag_to_camera = invertTransform(T_camera_to_tag);

        // Compute the absolute camera transform
        geometry_msgs::msg::Transform T_tag_absolute_adjusted = alignTagFrameToWorldFrame(T_tag_absolute);
        geometry_msgs::msg::Transform T_camera_absolute = multiplyTransforms(T_tag_absolute_adjusted, T_tag_to_camera);

        return T_camera_absolute;
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr posePublisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagLocalization>());
    rclcpp::shutdown();
    return 0;
}