#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

ros::Publisher point_count_pub;

// Moving average filter class
class MovingAverageFilter
{
public:
    MovingAverageFilter(int window_size) : window_size_(window_size), sum_(0), index_(0)
    {
        data_.resize(window_size_);
        data_.assign(window_size_, 0);
    }

    void addData(int value)
    {
        sum_ -= data_[index_];
        sum_ += value;
        data_[index_] = value;
        index_ = (index_ + 1) % window_size_;
    }

    float getAverage()
    {
        return static_cast<float>(sum_) / window_size_;
    }

private:
    int window_size_;
    std::vector<int> data_;
    int index_;
    int sum_;
};

// Point cloud callback function with moving average filter
void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    static MovingAverageFilter filter(5);

    int num_points = msg->width * msg->height;

    filter.addData(num_points);

    std_msgs::Float64 point_count_msg;
    point_count_msg.data = filter.getAverage();
    point_count_pub.publish(point_count_msg);

    // Print filtered result
    // ROS_INFO("Received point cloud with %d points, average point count: %.3f", num_points, filter.getAverage());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_points_counter");
    ros::NodeHandle nh;

    std::string topic_name = "/extract_cylinder_indices/output";
    if (argc > 1)
    {
        topic_name = argv[1];
    }
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic_name, 1, pointcloud_callback);
    point_count_pub = nh.advertise<std_msgs::Float64>("/handle_pc_count", 1);

    ros::spin();

    return 0;
}
