#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <Eigen/Dense>

using sensor_msgs::msg::Imu;
using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<Imu, Imu> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

class ImuFusionNode : public rclcpp::Node
{
public:
  ImuFusionNode() : Node("imu_fusion")
  {
    this->declare_parameter<bool>("apply_transform", false);
    this->get_parameter("apply_transform", apply_transform_);

    if (apply_transform_) {
      // Définir ici la matrice de transformation statique entre le repère du gyroscope
      // et celui de l'accéléromètre en cas de besoin. Exemple : rotation de 180° autour de l'axe Y.
      Eigen::AngleAxisd rotation(M_PI, Eigen::Vector3d::UnitY());
      gyro_to_accel_ = rotation.toRotationMatrix();
      RCLCPP_INFO(this->get_logger(), "Transformation statique activée du gyroscope vers l'accéléromètre.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Aucune transformation statique appliquée; utilisation du repère de l'accéléromètre.");
    }

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

    // Prendre directement les mesures de l'accéléromètre pour l'accélération linéaire
    fused.linear_acceleration = accel_msg->linear_acceleration;

    // Pour la vitesse angulaire, appliquer la transformation si activée
    geometry_msgs::msg::Vector3 gyro_transformed = gyro_msg->angular_velocity;
    if (apply_transform_) {
      Eigen::Vector3d gyro_vec(gyro_msg->angular_velocity.x,
                               gyro_msg->angular_velocity.y,
                               gyro_msg->angular_velocity.z);
      Eigen::Vector3d transformed_vec = gyro_to_accel_ * gyro_vec;
      gyro_transformed.x = transformed_vec.x();
      gyro_transformed.y = transformed_vec.y();
      gyro_transformed.z = transformed_vec.z();
    }
    fused.angular_velocity = gyro_transformed;

    // Fusionner les covariances
    fused.linear_acceleration_covariance = accel_msg->linear_acceleration_covariance;
    fused.angular_velocity_covariance = gyro_msg->angular_velocity_covariance;

    imu_pub_->publish(fused);
    RCLCPP_INFO(this->get_logger(), "Données IMU fusionnées publiées.");
  }

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;

  // Abonnements via message_filters
  message_filters::Subscriber<Imu> accel_sub_{this, "/camera/camera/accel/sample"};
  message_filters::Subscriber<Imu> gyro_sub_{this, "/camera/camera/gyro/sample"};

  // Synchronizer pour fusionner les deux flux de messages
  std::shared_ptr<Synchronizer> sync_;

  // Paramètre pour l'application de transformation
  bool apply_transform_{false};
  // Matrice de transformation statique du repère du gyroscope vers celui de l'accéléromètre
  Eigen::Matrix3d gyro_to_accel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
