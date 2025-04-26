//���ϲ��õ���Բ׶�����뱣�ַɻ����򣬿��Բο�collision_avoidance_gama.cpp��ı����㷨��ע����ı���Command_now.yaw_sp
//�Ӿ�ģ��ȱʧ



#include <ros/ros.h>

//topic ͷ�ļ�
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ȫ �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};
//--------------------------------------------����--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //�����״��������
geometry_msgs::PoseStamped pos_drone;                                  //���˻���ǰλ��
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float sleep_time;
float target_x;                                                 //����λ��_x
float target_y;                                                 //����λ��_y
float x_1;
float y_1;
float x_2;
float y_2;
float x_3;
float y_3;
float x_4;
float y_4;
int range_min;                                                //�����״�̽�ⷶΧ ��С�Ƕ�
int range_max;                                                //�����״�̽�ⷶΧ ���Ƕ�
float last_time = 0;
float fly_height;
//--------------------------------------------�㷨���--------------------------------------------------
float abs_distance;
float track_angle;
float avoid_angle;
float yaw_angle;                                                 //ƫ����
float R_outside, R_inside;                                       //��ȫ�뾶 [�����㷨��ز���]
float distance_c, angle_c;                                       //����ϰ������ �Ƕ�
float top_angle;                                                 //Բ׶���ǵ�һ��
float p_xy;                                                     //׷�ٲ���λ�û�P
float vel_track;                                             //׷�ٲ����ٶ�
int flag_land;                                                  //�����־λ
//--------------------------------------------���--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //�Ƿ�������ģʽ��־λ
float vel_sp_ENU[2];                                            //ENU�µ����ٶ�
float vel_sp_max;                                               //���ٶ��޷�
px4_command::command Command_now;                               //���͸�position_control.cpp������
int hold_flag;                                                  //�Ƿ������ͣ
int point_1_flag;                                               //�Ƿ񵽴�Ŀ����־
int turn_flag;
int point_2_flag;
int point_3_flag;
int point_4_flag;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void turn();
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //��ӡ����
void printf_param();                                                                 //��ӡ��������Թ����
void collision_avoidance(float target_x, float target_y);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//�����״�����ݣ�������Ӧ����,Ȼ�����ǰ������������С����
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;



    int count;    //count = 359
    count = Laser.ranges.size();

    //�޳�inf�����
    for (int i = 0; i < count; i++)
    {
        //�ж��Ƿ�Ϊinf
        int a = isinf(Laser_tmp.ranges[i]);
        //���Ϊinf����ֵ��һ�Ƕȵ�ֵ
        if (a == 1)
        {
            if (i == 0)
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[count - 1];
            }
            else
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[i - 1];
            }
        }
    }
    for (int i = 0; i < count; i++)
    {
        if (i + 180 > 359) Laser.ranges[i] = Laser_tmp.ranges[i - 180];
        else Laser.ranges[i] = Laser_tmp.ranges[i + 180];
        //cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
    }
    //cout<<"//////////////"<<endl;
    //����ǰ������������С����
    cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);

    yaw_angle = Euler_fcu[2] / M_PI * 180.0;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    // Ƶ�� [20Hz]
    ros::Rate rate(20.0);
    //�����ġ�Lidar����
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);
    //ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
    //�����ġ����˻���ǰλ�� ����ϵ NEDϵ
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // �����������͸�position_control.cpp������
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //��ȡ�������еĲ���
    nh.param<float>("sleep_time", sleep_time, 100.0);

    nh.param<float>("x_1", x_1, 6.0);
    nh.param<float>("y_1", y_1, 0.0);

    nh.param<float>("x_2", x_2, 6.0);
    nh.param<float>("y_2", y_2, -2.0);

    nh.param<float>("x_3", x_3, 0.0);
    nh.param<float>("y_3", y_3, -5.0);

    nh.param<float>("x_4", x_4, 6.0);
    nh.param<float>("y_4", y_4, -6.0);

    nh.param<float>("R_outside", R_outside, 1.4);
    nh.param<float>("R_inside", R_inside, 0.42);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);
    nh.getParam("fly_height", fly_height);


    //��ӡ��ʵ������
    printf_param();

    //check paramater
    int check_flag;
    //����1,�������������˳�����
    cout << "Please check the parameter and setting��1 for go on�� else for quit: " << endl;
    cin >> check_flag;
    if (check_flag != 1) return -1;

    //check arm
    int Arm_flag;
    cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
    cin >> Arm_flag;
    if (Arm_flag == 1)
    {
        Command_now.command = Arm;
        command_pub.publish(Command_now);
    }
    else return -1;

    //check takeoff
    int Take_off_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit" << endl;
    cin >> Take_off_flag;
    if (Take_off_flag == 1)
    {
        Command_now.command = Takeoff;
        command_pub.publish(Command_now);
    }
    else return -1;

    //takeoff
    int i = 0;
    int comid = 1;
    hold_flag = 0;   
    while (i < sleep_time)
    {
        ros::spinOnce();

        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        printf();
        if (sqrt((pos_drone.pose.position.x) * (pos_drone.pose.position.x) + (pos_drone.pose.position.y) * (pos_drone.pose.position.y) + (pos_drone.pose.position.z - fly_height) * (pos_drone.pose.position.z - fly_height)) < 0.3)
            i++;
        cout << "i:" << i << endl;
    }
    hold_flag = 1;

    //��ֵ
    track_angle = 0;

    vel_track = 0;

    vel_sp_ENU[0] = 0;
    vel_sp_ENU[1] = 0;

    flag_land = 0;

        
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    //����һ����
    target_x = x_1;
    target_y = y_1;
    point_1_flag = 0;
    abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
    while (ros::ok() && abs_distance > 0.1)
    {
        //�ص�һ�� ���´�����״̬
        ros::spinOnce();

        Command_now.command = Move_ENU;     //����ϵ���ƶ�

        Command_now.sub_mode = 0;//��һ�β��þ������
        Command_now.pos_sp[0] = target_x;
        Command_now.pos_sp[1] = target_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = track_angle;
        Command_now.comid = comid;
        comid++;


        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));

        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //��ӡ
        printf();
        rate.sleep();
    }
    point_1_flag = 1;
    printf();


    //��ת
    turn_flag = 0;
    while (ros::ok() && abs(yaw_angle + 90) > 3)
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        //Ϊ��ȷ����ת�����в�ƫ��Ŀ��㣬�����������
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = target_x;
        Command_now.pos_sp[1] = target_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = track_angle;
        Command_now.comid = comid;
        comid++;

        if (track_angle > -90)
        {
            track_angle = track_angle - 1;
            cout << "***turning***" << endl;
        }

        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //��ӡ
        printf();
        rate.sleep();
    }
    turn_flag = 1;
    printf();
    
    

    //����ȥ�ڶ�����
    target_x = x_2;
    target_y = y_2;
    point_2_flag = 0;
    abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
    while (ros::ok() && abs_distance > 0.1)
    {
        //�ص�һ�� ���´�����״̬
        ros::spinOnce();

        Command_now.command = Move_ENU;     //����ϵ���ƶ�

        collision_avoidance(target_x, target_y);

        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = target_x;
        Command_now.pos_sp[1] = target_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = track_angle;
        Command_now.comid = comid;
        comid++;

        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));

        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //��ӡ
        printf();
        rate.sleep();
    }
    point_2_flag = 1;
    printf();

    //����ȥ��������
    target_x = x_3;
    target_y = y_3;
    point_3_flag = 0;
    abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
    while (ros::ok() && abs_distance > 0.1)
    {
        //�ص�һ�� ���´�����״̬
        //�����״�������ݣ��洢��Laser��,������������С����
        ros::spinOnce();
      
        //if (abs_distance > 0.3)//Ϊ�˽���ɻ���Ŀ����ΧתȦ
        
        collision_avoidance(target_x, target_y);

        Command_now.command = Move_ENU;     //����ϵ���ƶ�
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy �ٶȿ���ģʽ z λ�ÿ���ģʽ
        Command_now.vel_sp[0] = vel_sp_ENU[0];
        Command_now.vel_sp[1] = vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = track_angle;
        
        
        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));

        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //��ӡ
        printf();
        rate.sleep();
    }
    point_3_flag = 1;
    printf();

    //����ȥ���ĸ���
    target_x = x_4;
    target_y = y_4;
    point_4_flag = 0;
    abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
    while (ros::ok() && abs_distance > 0.1)
    {
        //�ص�һ�� ���´�����״̬
        //�����״�������ݣ��洢��Laser��,������������С����
        ros::spinOnce();

        //if (abs_distance > 0.3)
        collision_avoidance(target_x, target_y);

        Command_now.command = Move_ENU;     //����ϵ���ƶ�
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy �ٶȿ���ģʽ z λ�ÿ���ģʽ
        Command_now.vel_sp[0] = vel_sp_ENU[0];
        Command_now.vel_sp[1] = vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = track_angle;

        
        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
       
        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //��ӡ
        printf();
        rate.sleep();
    }
    point_4_flag = 1;
    printf();


    //����
    flag_land = 1;
    Command_now.command = Land;
    while (ros::ok())
    {
        command_pub.publish(Command_now);
        rate.sleep();
        printf();
    }
    rate.sleep();
    cout << "Mission complete, exiting...." << endl;


    return 0;
}


//����ǰ������������С����
void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    for (int i = range_min; i <= range_max; i++)
    {
        if (Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }
}

//���ͺ���
float satfunc(float data, float Max)
{
    if (abs(data) > Max) return (data > 0) ? Max : -Max;
    else return data;
}

void collision_avoidance(float target_x, float target_y)
{
    //����׷�ٽǶ�
    track_angle = atan2((target_y - pos_drone.pose.position.y), (target_x - pos_drone.pose.position.x));
    if (track_angle < 0)
        track_angle = track_angle + 2 * M_PI;
    track_angle = track_angle / M_PI * 180;//����ת�Ƕ�0-359.99

    //avoid_angle��ʼ��
    avoid_angle = track_angle;


    if (fabs(R_inside) <= fabs(distance_c))//����ϰ���δ����R_inside
    {
        //����Բ׶���ǵ�һ��
        top_angle = asin(R_inside / distance_c) / M_PI * 180;

        //�ж��ϰ����Ƿ񵱵�
        bool Is_in = false;
        if (fabs(angle_c - 360) < top_angle || angle_c < top_angle)
            Is_in = true;


        //������С������Ƿ񵱵��жϣ��Ƿ����ñ��ϲ���
        if (distance_c >= R_outside || !Is_in)
        {
            flag_collision_avoidance.data = false;
        }
        else
        {
            flag_collision_avoidance.data = true;
        }



        // ���ϲ���
        if (flag_collision_avoidance.data == true)
        {
            if (angle_c < 180)
            {
                avoid_angle = (int)(angle_c - top_angle + 360) % 360;
                avoid_angle = (int)(yaw_angle + avoid_angle) % 360;
            }
            else
            {
                avoid_angle = (int)(avoid_angle + (int)(angle_c + top_angle - 360) % 360) % 360;
                avoid_angle = (int)(yaw_angle + avoid_angle) % 360;
            }
        }

    }
    else//����ɻ��Ѿ�����R_inside���跨�˳�R_inside�����ø�һ�������ٶȴﵽɲ����Ч��������ɲ��Ч���ܶ෽����Ӱ�죺��Χ�ϰ�����ܼ��̶ȡ��ɻ����ٶȡ��ɻ������ܡ�����
    {
        avoid_angle = ((int)angle_c + 180) % 360;
        avoid_angle = (int)(yaw_angle + avoid_angle) % 360;
    }

    //��������������ʽ���ȼ����˻���ϵ�ı��Ͻǣ�Ȼ��ת�����磬���ȡ��Ŀ����ȷ���Ƕ���0-359.99



    //����׷���ٶ�
    vel_track = p_xy * sqrt((target_x - pos_drone.pose.position.x) * (target_x - pos_drone.pose.position.x) + (target_y - pos_drone.pose.position.y) * (target_y - pos_drone.pose.position.y));

    //�ٶ��޷�
    vel_track = satfunc(vel_track, vel_sp_max);



    //����׷���ٶȵı��ϽǼ�������ϵ�µ��ٶ�
    vel_sp_ENU[0] = vel_track * cos(avoid_angle / 180 * M_PI);
    vel_sp_ENU[1] = vel_track * sin(avoid_angle / 180 * M_PI);

}

void printf()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>integrated work<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    if (hold_flag) cout << "hold finished" << endl;
    else cout << "hold" << endl;

    if (point_1_flag) cout << "point 1 finished" << endl;
    else cout << "point 1" << endl;

    if (turn_flag) cout << "turn finished" << endl;
    else cout << "turn" << endl;

    if (point_2_flag) cout << "point 2 finished" << endl;
    else cout << "point 2" << endl;

    if (point_3_flag) cout << "point 3 finished" << endl;
    else cout << "point 3" << endl;

    if (point_4_flag) cout << "point 4 finished" << endl;
    else cout << "point 4" << endl;

    if (flag_land) cout << "landing" << endl;
    else cout << "land" << endl;

    if (point_2_flag)
    {
        cout << "Minimun_distance : " << endl;
        cout << "Distance : " << distance_c << " [m] " << endl;
        cout << "Angle :    " << angle_c << " [du] " << endl;
        if (flag_collision_avoidance.data == true)
        {
            cout << "Collision avoidance Enabled " << endl;
        }
        else
        {
            cout << "Collision avoidance Disabled " << endl;
        }
        cout << "vel_track : " << vel_track << " [m/s] " << endl;        
        cout << "top_angle :" << top_angle << "[deg]" << endl;
        cout << "track_angle :" << track_angle << "[deg]" << endl;
        cout << "avoid_angle :" << avoid_angle << "[deg]" << endl;
    }
    
    cout << "X:" << pos_drone.pose.position.x << endl;
    cout << "Y:" << pos_drone.pose.position.y << endl;
    cout << "Euler_fcu[2]:" << Euler_fcu[2] << "[rad]" << endl;
    cout << "yaw_angle:" << yaw_angle << "[deg]" << endl;
    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] " << endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] " << endl;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "sleep_time" << sleep_time << endl;
    cout << "x_1 : " << x_1 << endl;
    cout << "y_1 : " << y_1 << endl;

    cout << "x_2 : " << x_2 << endl;
    cout << "y_2 : " << y_2 << endl;

    cout << "x_3 : " << x_3 << endl;
    cout << "y_3 : " << y_3 << endl;

    cout << "x_4 : " << x_4 << endl;
    cout << "y_4 : " << y_4 << endl;

    cout << "R_outside : " << R_outside << endl;
    cout << "R_inside : " << R_inside << endl;

    cout << "p_xy : " << p_xy << endl;

    cout << "vel_sp_max : " << vel_sp_max << endl;
    cout << "range_min : " << range_min << endl;
    cout << "range_max : " << range_max << endl;
    cout << "fly height: " << fly_height << endl;
}