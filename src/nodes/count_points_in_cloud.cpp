#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Moving average filter class
class MovingAverageFilter
{
public:
    MovingAverageFilter(int window_size) : window_size_(window_size), sum_(0), index_(0)
    {
        data_.resize(window_size_);
        data_.assign(window_size_, 0);
    }

    void addData(float value)
    {
        sum_ -= data_[index_];
        sum_ += value;
        data_[index_] = value;
        index_ = (index_ + 1) % window_size_;
    }

    float getAverage()
    {
        return sum_ / window_size_;
    }

private:
    int window_size_;
    std::vector<float> data_;
    int index_;
    float sum_;
};

// Point cloud callback function with moving average filter
void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    static MovingAverageFilter filter(5);

    int num_points = msg->width * msg->height;

    // Calculate average point distance and add to filter
    float avg_dist = 0;
    for (int i = 0; i < msg->data.size(); i += 3)
    {
        float x = *((float*)(&msg->data[i]));
        float y = *((float*)(&msg->data[i+4]));
        float z = *((float*)(&msg->data[i+8]));
        avg_dist += sqrt(x*x + y*y + z*z);
    }
    avg_dist /= num_points;
    filter.addData(avg_dist);

    // Print filtered result
    ROS_INFO("Received point cloud with %d points, average distance: %.3f", num_points, filter.getAverage());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/extract_cylinder_indices/output", 1, pointcloud_callback);

    ros::spin();

    return 0;
}
