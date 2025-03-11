#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using sensor_msgs::msg::Imu;
using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<Imu, Imu> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

class ImuFusionNode : public rclcpp::Node
{
public:
  ImuFusionNode() : Node("imu_fusion")
  {
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Création du publisher pour le message IMU fusionné
    imu_pub_ = this->create_publisher<Imu>("/camera/imu", qos_profile);

    // Initialisation des abonnements via message_filters
    accel_sub_.subscribe(this, "/camera/camera/accel/sample", qos_profile.get_rmw_qos_profile());
    gyro_sub_.subscribe(this, "/camera/camera/gyro/sample", qos_profile.get_rmw_qos_profile());

    // Synchroniser les deux flux
    sync_.reset(new Synchronizer(SyncPolicy(10), accel_sub_, gyro_sub_));
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
    sync_->registerCallback(std::bind(&ImuFusionNode::callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Noeud de fusion IMU initialisé.");
  }

private:
  void callback(const Imu::ConstSharedPtr &accel_msg, const Imu::ConstSharedPtr &gyro_msg)
  {
    Imu fused;

    // Calculer un timestamp moyen
    rclcpp::Time time_accel(accel_msg->header.stamp);
    rclcpp::Time time_gyro(gyro_msg->header.stamp);
    rclcpp::Time fused_time((time_accel.nanoseconds() + time_gyro.nanoseconds()) / 2);
    fused.header.stamp = fused_time;

    // Utiliser le repère de l'accéléromètre comme référence
    fused.header.frame_id = accel_msg->header.frame_id;

    // Fusionner les mesures : accélération de l'accéléromètre et vitesse angulaire du gyroscope
    fused.linear_acceleration = accel_msg->linear_acceleration;
    fused.angular_velocity = gyro_msg->angular_velocity;

    // Fusionner les covariances
    fused.linear_acceleration_covariance = accel_msg->linear_acceleration_covariance;
    fused.angular_velocity_covariance = gyro_msg->angular_velocity_covariance;

    imu_pub_->publish(fused);
    RCLCPP_INFO(this->get_logger(), "Données IMU fusionnées publiées.");
  }

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;

  message_filters::Subscriber<Imu> accel_sub_{this, "/camera/camera/accel/sample"};
  message_filters::Subscriber<Imu> gyro_sub_{this, "/camera/camera/gyro/sample"};

  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
