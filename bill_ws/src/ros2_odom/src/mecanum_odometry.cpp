#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>
#include <cmath>

class MecanumOdometry : public rclcpp::Node {
public:
    MecanumOdometry() : Node("mecanum_odometry"), x_(0.0), y_(0.0), th_(0.0) {

        // --- PARÂMETROS ---
        this->declare_parameter<double>("ticks_per_revolution", 663.0);
        this->declare_parameter<double>("wheel_radius", 0.075);
        this->declare_parameter<double>("robot_length", 0.255);
        this->declare_parameter<double>("robot_width", 0.22);
        
        // --- ALTERAÇÃO: Assinaturas atualizadas para os novos tópicos unificados ---
        // enc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "encoder_counts", 10, std::bind(&MecanumOdometry::encoder_callback, this, std::placeholders::_1));
        
        // Assinatura para as velocidades dos motores (RPM)
        vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "motor_velocities_rpm", 10, std::bind(&MecanumOdometry::velocity_callback, this, std::placeholders::_1));

        // Publisher de odometria
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/unfiltered", 50);
        
        motor_velocities_rpm_.resize(4, 0.0f);
        last_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Nó de odometria Mecanum iniciado.");
    }

private:
    // Posição e orientação atuais do robô
    double x_, y_, th_;

    // Assinaturas e publicadores
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr enc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // --- ALTERAÇÃO: Vetores para armazenar dados de todos os 4 encoders/motores ---
    std::vector<float> encoder_counts_;
    std::vector<float> motor_velocities_rpm_;

    rclcpp::Time last_time_;

    // --- ALTERAÇÃO: Callback único para os dados dos encoders ---
    void encoder_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            encoder_counts_ = msg->data;
        }
    }

    // --- ALTERAÇÃO: Callback único para os dados de velocidade (RPM) ---
    // A lógica principal de cálculo da odometria foi movida para cá.
    void velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Recebendo dados de velocidade com tamanho diferente de 4. Esperando 4 valores.");
            return;
        }
        motor_velocities_rpm_ = msg->data;

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        // Obtém os parâmetros para os cálculos
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double robot_length = this->get_parameter("robot_length").as_double();
        double robot_width = this->get_parameter("robot_width").as_double();

        // Converte RPM para radianos por segundo (Ordem: FL, FR, RL, RR)
        double w1 = (motor_velocities_rpm_[0] * 2.0 * M_PI) / 60.0;
        double w2 = (motor_velocities_rpm_[1] * 2.0 * M_PI) / 60.0;
        double w3 = (motor_velocities_rpm_[2] * 2.0 * M_PI) / 60.0;
        double w4 = (motor_velocities_rpm_[3] * 2.0 * M_PI) / 60.0;

        // Cinemática direta para calcular a velocidade do robô (vx, vy, vth)
        double vx = (w1 + w2 + w3 + w4) * (wheel_radius / 4.0);
        double vy = (-w1 + w2 - w3 + w4) * (wheel_radius / 4.0);
        double vth = (-w1 + w2 + w3 - w4) * (wheel_radius / (4.0 * (robot_length + robot_width)));

        // Calcula a mudança na posição no frame do robô
        double delta_x_robot = vx * dt;
        double delta_y_robot = vy * dt;
        double delta_th = vth * dt;
        
        // Projeta a mudança para o frame de odometria (odom)
        double delta_x = delta_x_robot * cos(th_) - delta_y_robot * sin(th_);
        double delta_y = delta_x_robot * sin(th_) + delta_y_robot * cos(th_);

        // Atualiza a pose
        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        // Cria e publica a mensagem de odometria
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";       // O frame pai
        odom_msg.child_frame_id = "base_link";   // O frame filho

        // Posição
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Orientação
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // --- AJUSTE NA COVARIÂNCIA ---
        // Adiciona uma covariância básica para indicar incerteza.
        // Valores maiores significam mais incerteza. Ajuste conforme necessário.
        odom_msg.pose.covariance[0] = 0.1;   // x
        odom_msg.pose.covariance[7] = 0.1;   // y
        odom_msg.pose.covariance[35] = 0.2;  // yaw

        // Velocidade
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = vth;
        odom_msg.twist.covariance = odom_msg.pose.covariance; // Pode usar a mesma ou definir uma diferente

        odom_pub_->publish(odom_msg);

        last_time_ = current_time;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumOdometry>());
    rclcpp::shutdown();
    return 0;
}