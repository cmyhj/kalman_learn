#include "rclcpp/rclcpp.hpp"
#include "kalman/kalman.hpp"

#include "atomic"

#include "geometry_msgs/msg/vector3.hpp"
#include <random>


//生成高斯噪音
double generateGaussianNoise(double mean, double std_dev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> distr(mean, std_dev);
    return distr(gen);
}


class Simulator : public rclcpp::Node {

public:
    Simulator(std::string name) : Node(name) {
        Kalman_fillter_CA = new Kalman(0.01);
        Kalman_fillter_CA->Q_set(30);
        Kalman_fillter_CA->R_set(0.05);



        //同一个组别，类型设置为Reentrant
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        //用于发送原始数据
        raw_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw", 1);
        //用于发送kalman滤波后的数据
        fillted_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("fillted", 1);

        //用于kalman运行的定时器，和云台输出频率一致
        kalman_timer_ = create_wall_timer(std::chrono::microseconds(1000),
                                          [this] { timer_cb(); }, callback_group_);
        //用于生成原始数据的定时器
        simulator_timer_ = create_wall_timer(std::chrono::microseconds(10000),
                                             [this] { simulator_cb(); }, callback_group_);

        RCLCPP_INFO(this->get_logger(), "节点%s已启动", name.c_str());
    }

private:

    Kalman *Kalman_fillter_CA;

    rclcpp::CallbackGroup::SharedPtr callback_group_;


    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr raw_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fillted_publisher_;


    rclcpp::TimerBase::SharedPtr kalman_timer_;
    rclcpp::TimerBase::SharedPtr simulator_timer_;

    double simulate_pos_x;


    std::atomic<bool> isunsuccessful = true;

    void timer_cb() {

        Eigen::Matrix<double, 1, 1> meassure;
        Eigen::Vector2d CA_fillter_output;
        meassure << simulate_pos_x;

        if (!isunsuccessful) {
            CA_fillter_output = Kalman_fillter_CA->predict();
            RCLCPP_INFO(this->get_logger(), "kalman->predict()");
        }
        CA_fillter_output = Kalman_fillter_CA->update(meassure);
        RCLCPP_INFO(this->get_logger(), "kalman->update(meassure)");

        geometry_msgs::msg::Vector3 fillted;
        fillted.x = CA_fillter_output(0);
        fillted.y = CA_fillter_output(1);

        fillted_publisher_->publish(fillted);
        isunsuccessful = true;


    }


    //测试匀速直线运动
    void simulator_cb() {
        static double pos_x = 0;
        static double vec_x = 1;
        static double acc_x = 0;
        static double T = 0.01;

        static uint16_t error_inject_cnt;
        static uint16_t vec_change_cnt;

        RCLCPP_INFO(this->get_logger(), "simulator");
        /*if(vec_x > 2)
        {
            acc_x = -0.5;
        }
        else if(vec_x < -2)
        {
            acc_x = 0.5;
        }*/

        vec_x += acc_x * T;
        pos_x = pos_x + vec_x * T + 0.5 * acc_x * T * T;


        simulate_pos_x = pos_x + generateGaussianNoise(0, 0.05);

        if (error_inject_cnt > 200)//故障注入
        {
            simulate_pos_x += 2;//注入1m错误
            error_inject_cnt = 0;
        }
        if (vec_change_cnt > 400)//随机改变速度
        {
            std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(-2, 2);
            vec_x = distribution(generator);
            vec_change_cnt = 0;
        }

        error_inject_cnt++;
        vec_change_cnt++;

        isunsuccessful = false;

        geometry_msgs::msg::Vector3 raw;
        raw.x = simulate_pos_x;
        raw.y = vec_x;
        raw.z = acc_x;
        raw_publisher_->publish(raw);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulator>("test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}