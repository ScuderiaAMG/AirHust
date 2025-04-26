//��һ���汾��kimi 1.5
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <slam_gmapping/slam_gmapping.h> // ����ʹ��gmapping����SLAM

// ȫ�ֱ���
ros::Publisher vel_pub;
geometry_msgs::Twist vel_cmd;

// SLAM��ر���
bool is_slam_initialized = false;
nav_msgs::Odometry odom_data;

// �����״�ص�����
void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static int nCount = 0;

    // ��ȡ�����״��м��ľ���
    int nMid = msg->ranges.size() / 2;
    float fMidDist = msg->ranges[nMid];
    ROS_INFO("ǰ����� ranges[%d] = %f ��", nMid, fMidDist);

    // �����߼�
    if (fMidDist < 1.5f) { // ���ǰ������С��1.5�ף�ִ�б���
        vel_cmd.angular.z = 0.3; // ��ת����
        nCount = 50; // ���ü���������ʱ����
    }
    else { // ��������ǰ��
        vel_cmd.linear.x = 0.05; // ǰ���ٶ�
    }

    // �����ٶ�ָ��
    if (nCount > 0) {
        nCount--;
    }
    else {
        vel_pub.publish(vel_cmd);
    }
}

// SLAM��ʼ������
void initializeSLAM() {
    ros::NodeHandle nh;
    ros::Subscriber slam_sub = nh.subscribe("/scan", 10, LidarCallback);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

    // ����SLAM�ڵ�
    slam_gmapping::SlamGmapping slam;
    if (!slam.initialize()) {
        ROS_ERROR("SLAM initialization failed!");
        return;
    }

    is_slam_initialized = true;
    ROS_INFO("SLAM initialized successfully!");
}

// ������
int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "drone_navigation");

    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ��ʼ��SLAM
    initializeSLAM();

    // ���ļ����״�����
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, LidarCallback);

    // ��ѭ��
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // �����ٶ�ָ��
        vel_pub.publish(vel_cmd);

        // ����ROS��Ϣ
        ros::spinOnce();

        // ����ѭ��Ƶ��
        loop_rate.sleep();
    }

    return 0;
}

/************************************************************************************************************************************************************************/
//�ڶ����汾��deepseek R1

//��Ҫʹ�ã�<depend>gmapping</depend>
//<depend>tf2_geometry_msgs< / depend>
//<depend>sensor_msgs< / depend>��ros��

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <memory>

class SlamProcessor {
public:
    explicit SlamProcessor(ros::NodeHandle& nh)
        : tf_listener_(tf_buffer_)
    {
        // ������ʼ��
        nh.param("laser_topic", laser_topic_, std::string("/scan"));
        nh.param("map_frame", map_frame_, std::string("map"));
        nh.param("odom_frame", odom_frame_, std::string("odom"));

        // �����״����ݶ���
        laser_sub_ = nh.subscribe(laser_topic_, 1, &SlamProcessor::laserCallback, this);

        // ��ͼ���ݷ���
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/slam_map", 1);

        // SLAM��ʼ���߼�
        initSlamSystem();
    }

    // ��װSLAM������ѭ��
    void update() {
        if (!laser_data_ || !tfAvailable()) return;

        // ִ��SLAM����
        runSlamUpdate();

        // �������µ�ͼ
        publishMap();
    }

    // �ӿڣ���ȡ��ǰ��ͼ
    nav_msgs::OccupancyGrid getMap() const {
        return current_map_;
    }

    // �ӿڣ���ȡ��ȷ��λ��̬
    bool getRobotPose(geometry_msgs::PoseStamped& pose) {
        return getFramePose(map_frame_, pose);
    }

private:
    void initSlamSystem() {
        // ��ʼ��SLAM�㷨����
        // �˴��ɼ���Gmapping��Hector��SLAMʵ��
        ROS_INFO("Initializing SLAM system...");
        current_map_.header.frame_id = map_frame_;
        // �����ʼ������...
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        laser_data_ = msg;
    }

    bool tfAvailable() {
        return tf_buffer_.canTransform(map_frame_, odom_frame_, ros::Time(0));
    }

    bool getFramePose(const std::string& target_frame, geometry_msgs::PoseStamped& pose) {
        try {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(target_frame, "base_link", ros::Time(0));
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = target_frame;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup failure: %s", ex.what());
            return false;
        }
    }

    void runSlamUpdate() {
        // �˴�ʵ�־����SLAM�㷨�߼�
        // ʾ��α���룺
        // 1. ��ȡ��������
        // 2. ��ȡ��̼�����
        // 3. ִ��ɨ��ƥ��
        // 4. ���µ�ͼ����

        std::lock_guard<std::mutex> lock(data_mutex_);
        // ʵ��SLAM�������...

        // ʾ�����򵥵�ͼ���£�ʵ��Ӧ�滻ΪSLAM�㷨��
        static int map_update_count = 0;
        current_map_.header.stamp = ros::Time::now();
        current_map_.info.resolution = 0.05;
        current_map_.info.width = 100;
        current_map_.info.height = 100;
        current_map_.data.resize(100 * 100, -1);
        map_update_count++;
    }

    void publishMap() {
        if (+()) return;
        map_pub_.publish(current_map_);
    }

    // SLAM�������
    ros::Subscriber laser_sub_;
    ros::Publisher map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // �̰߳�ȫ���ݷ���
    std::mutex data_mutex_;
    sensor_msgs::LaserScan::ConstPtr laser_data_;
    nav_msgs::OccupancyGrid current_map_;

    // ���ò���
    std::string laser_topic_;
    std::string map_frame_;
    std::string odom_frame_;
};

// ���п�������
class FlightController {
public:
    FlightController(ros::NodeHandle& nh)
        : slam_processor_(nh)
    {
        // ��ʼ���������п������...
    }

    void mainLoop() {
        ros::Rate rate(10);
        while (ros::ok()) {
            // SLAM����
            slam_processor_.update();

            // ��ȡ��λ��Ϣ
            geometry_msgs::PoseStamped current_pose;
            if (slam_processor_.getRobotPose(current_pose)) {
                // ·���滮������߼�
                executeNavigation(current_pose);
            }

            rate.sleep();
        }
    }

private:
    void executeNavigation(const geometry_msgs::PoseStamped& pose) {
        // ʵ�ֵ��������߼�
        // 1. ��ȡ��ǰ��ͼ
        // 2. ·���滮
        // 3. �˶�����

        nav_msgs::OccupancyGrid current_map = slam_processor_.getMap();
        // ·���滮����...
    }

    SlamProcessor slam_processor_;
    // �����������...
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_slam_controller");
    ros::NodeHandle nh("~");

    FlightController controller(nh);
    controller.mainLoop();

    return 0;
}



/***********************************************************************************************************************/
//�������汾��deepseek-zju

//��װ��Ҫ����sudo apt - get install ros - noetic - slam - gmapping

/*
* ���׵�.launch
<launch>
    <!-- ����SLAM�ڵ� -->
    <include file="$(find gmapping)/launch/slam_gmapping.launch"/>
    
    <!-- ���������ڵ� -->
    <node pkg="your_package" type="navigation_node" name="drone_navigation" output="screen">
        <param name="safe_distance" value="1.2"/>
        <param name="target_threshold" value="0.25"/>
    </node>
</launch> 
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>

class DroneNavigation {
public:
    DroneNavigation() : nh_("~") {
        // ������ʼ��
        nh_.param("safe_distance", safe_distance_, 1.5f);
        nh_.param("target_threshold", target_threshold_, 0.3f);

        // �����߳�ʼ��
        laser_sub_ = nh_.subscribe("/scan", 1, &DroneNavigation::laserCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &DroneNavigation::odomCallback, this);

        // �����߳�ʼ��
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // SLAM�����ʼ��
        slam_client_ = nh_.serviceClient<nav_msgs::GetMap>("/dynamic_map");
    }

    void mainLoop() {
        ros::Rate rate(10);
        while (ros::ok()) {
            this->slamMapping();
            this->navigateThroughDoor();
            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    // SLAM��ͼ����
    void slamMapping() {
        nav_msgs::GetMap srv;
        if (slam_client_.call(srv)) {
            current_map_ = srv.response.map;
            // �˴���ӵ�ͼ�����߼�
        }
    }

    // �����״�ص�
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        latest_scan_ = *scan;
        processObstacleData(scan);
    }

    // ��̼ƻص�
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        current_pose_ = odom->pose.pose;
    }

    // �ϰ��ﴦ������߼�
    void processObstacleData(const sensor_msgs::LaserScan::ConstPtr& scan) {
        float min_distance = *std::min_element(scan->ranges.begin(), scan->ranges.end());

        if (min_distance < safe_distance_) {
            executeAvoidanceManeuver();
        }
        else {
            maintainCourse();
        }
    }

    // ���ŵ����߼�
    void navigateThroughDoor() {
        static bool door_detected = false;

        // �ſ��⣨ʾ���߼���
        if (detectDoorFrame(latest_scan_)) {
            door_detected = true;
            adjustApproachAngle();
        }

        if (door_detected && checkClearance()) {
            executeDoorTraversal();
        }
    }

    // ���ϻ���
    void executeAvoidanceManeuver() {
        geometry_msgs::Twist cmd;
        cmd.angular.z = 0.5;  // ˳ʱ����ת
        cmd_pub_.publish(cmd);
    }

    // ���ֺ���
    void maintainCourse() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.5;   // ǰ���ٶ�
        cmd_pub_.publish(cmd);
    }

    // �ſ��⺯��
    bool detectDoorFrame(const sensor_msgs::LaserScan& scan) {
        // �򻯵��ſ����߼�
        const int mid_index = scan.ranges.size() / 2;
        return (scan.ranges[mid_index - 10] < 1.0 &&
            scan.ranges[mid_index + 10] < 1.0 &&
            scan.ranges[mid_index] > 2.0);
    }

    // ������Ա����...

    // ��Ա����
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_, odom_sub_;
    ros::Publisher cmd_pub_;
    ros::ServiceClient slam_client_;

    sensor_msgs::LaserScan latest_scan_;
    geometry_msgs::Pose current_pose_;
    nav_msgs::OccupancyGrid current_map_;

    float safe_distance_;
    float target_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_navigation_node");
    DroneNavigation drone;
    drone.mainLoop();
    return 0;
}

//