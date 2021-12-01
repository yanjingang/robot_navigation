/**
   @file pid.ino
   @author yanjingang@mail.com
   @brief  监听ros的/cmd_vel底盘请求，使用模拟信号控制电机
           https://www.cnblogs.com/flyingjun/p/8954043.html
   @version 0.1
   @date 2021-11-30
   @copyright Copyright (c) 2021

*/
#include <PID_v1.h>
//#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//#define Pin_left 2   //外部中断0,左轮
//#define Pin_right 3  //外部中断1,右轮

#define max_linear 20     //最大线速度cm/秒
#define max_turn_line 18  //最大转弯线速度
//#define max_angular 1.45
#define max_linear_pwd 255

#define hole_number 2    //码盘孔数
#define diameter 18.5  //轮cm直径
#define diamete_ratio 1.0  //左轮相对于右轮轮径比系数,往左偏,调小,往右偏调大
#define center_speed 220  //小车电机的PWM功率初始值
#define gear_ratio 75     //转速比
#define car_width 15      //小车宽度
#define car_length 12     //小车长度

// 电机的
#define PIN_left_m 9  // L298P直流电机驱动板的左轮电机转向端口连接到数字接口
#define PIN_left_e 10  // L298P直流电机驱动板的左轮电机使能端口连接到数字接口
#define PIN_right_m 5  //连接小车右轮电机的转向端口到数字接口
#define PIN_right_e 6  //连接小车右轮电机的使能端口到数字接口

int val_right_count_target =0;  //小车右轮码盘每秒计数PID调节目标值,根据这个值PID val_rigth;
int val_right = 0;  //小车右轮电机的PWM功率值
int val_left_count_target =0;  //小车左轮码盘每秒计数PID调节目标值,根据这个值PID val_left;
int val_left = 0;  //左轮电机PWM功率值。以左轮为基速度,PID调节右轮的速度
int count_left = 0;   //左轮编码器码盘脉冲计数值;用于PID调整
int count_right = 0;  //右轮编码器码盘脉冲计数值;用于PID调整
//
char run_direction = 'f';  // f:前进;b:后退;s:stop
int linear = 0;            // 15; //cm/second线速度
int angular = 0;           // 1; //角速度,ros的angular.z
//转弯半径一定要大于小车宽度的一半,也就是linear /
//angular一定是大于13.5,也就是最小转弯半径是13.5
//
unsigned long left_old_time = 0, right_old_time = 0;  // 时间标记
unsigned long time1 = 0, time2 = 0;                   // 时间标记

//ros
ros::NodeHandle nh;
// sub cmd_vel
void subCmd(const geometry_msgs::Twist& vel) {
    linear = vel.linear.x * 100;  // ROS中的单位是m/s;这里换算成cm的单位
    angular = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", subCmd);
// pub debug info
std_msgs::String str_msg;
String debug = "";
ros::Publisher pub("/debug",&str_msg);
/*// pub odom
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char frame_base_link[] = "/base_footprint";
char frame_odom[] = "/odom";
nav_msgs::Odometry odom;
ros::Publisher pub("/odom", &odom);
*/

//PID
double left_Setpoint, left_Input, left_Output, left_setpoint;
double left_kp = 1, left_ki = 0.005, left_kd = 0.0001;  // kp = 0.040,ki = 0.0005,kd =0.0011;
PID left_PID(&left_Input, &left_Output, &left_Setpoint, left_kp, left_ki, left_kd, DIRECT);

double right_Setpoint, right_Input, right_Output, right_setpoint;
double right_kp = 0.8, right_ki = 0.005, right_kd = 0.0021;  // kp = 0.040,ki = 0.0005,kd =0.0011;
PID right_PID(&right_Input, &right_Output, &right_Setpoint, right_kp, right_ki, right_kd, DIRECT);


void setup() {
    // put your setup code here, to run once:
    Serial.begin(57600);  // 启动串口通信，波特率为9600b/s
    // reserve 200 bytes for the inputString

    pinMode(PIN_left_m, OUTPUT);  // L298P直流电机驱动板的控制端口设置为输出模式
    pinMode(PIN_left_e, OUTPUT);
    pinMode(PIN_right_m, OUTPUT);
    pinMode(PIN_right_e, OUTPUT);

    //定义外部中断0和1的中断子程序Code(),中断触发为下跳沿触发
    //当编码器码盘的OUT脉冲信号发生下跳沿中断时，
    //将自动调用执行中断子程序Code()。
    left_old_time = millis();
    right_old_time = millis();
    //attachInterrupt(0, Code1, FALLING);  //小车左车轮电机的编码器脉冲中断函数
    //attachInterrupt(1, Code2, FALLING);  //小车右车轮电机的编码器脉冲中断函数

    // ROS节点初始化
    nh.initNode();
    nh.advertise(pub);  // publish
    nh.subscribe(sub);  // subscribe
    
    // broadcaster.init(nh);
    left_PID.SetOutputLimits(-254, 254);
    left_PID.SetSampleTime(500);
    left_PID.SetMode(AUTOMATIC);
    left_PID.SetTunings(left_kp, left_ki, left_kd);

    right_PID.SetOutputLimits(-254, 254);
    right_PID.SetSampleTime(500);
    right_PID.SetMode(AUTOMATIC);
    right_PID.SetTunings(right_kp, right_ki, right_kd);
}


void loop() {
    // 根据ros cmd_vel指令移动
    cmd_vel_move();

    // pub deubg信息
    str_msg.data = debug.c_str();
    pub.publish(&str_msg);
    

    /*// pub odom (uno板子内存2k跑不起来，放到pi端pub)
    odom.header.stamp = nh.now(); //1638365722.576050519
    odom.header.frame_id = frame_odom;
    odom.child_frame_id = frame_base_link;
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 0;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    pub.publish(&odom);
    //broadcaster.sendTransform(odom);*/
    

    nh.spinOnce();
}


//移动方法
void advance()  //前进（这个前进方向和激光雷达的位置一定对应好，别搞反了）
{
    digitalWrite(PIN_left_m, HIGH);
    analogWrite(PIN_left_e, val_left);
    digitalWrite(PIN_right_m, HIGH);
    analogWrite(PIN_right_e, val_right);
}
void back()  //后退
{
    digitalWrite(PIN_left_m, LOW);
    analogWrite(PIN_left_e, val_left);
    digitalWrite(PIN_right_m, LOW);
    analogWrite(PIN_right_e, val_right);
}
void Stop()  //停止
{
    digitalWrite(PIN_left_m, LOW);
    digitalWrite(PIN_right_e, LOW);
    digitalWrite(PIN_right_m, LOW);
    digitalWrite(PIN_left_e, LOW);
}

// 根据ros cmd_vel指令移动
void cmd_vel_move(){
  if (angular == 0) {    //直行
        if (linear > 0) {  //前进
            /*
            if (linear > max_linear)
                linear = max_linear;
            
            float linear_left = linear;   //左内圈线速度
            float linear_right = linear;  //右外圈线速度

            val_right_count_target = linear_right * gear_ratio / (diameter / hole_number);  //左内圈线速度对应的孔数
            val_left_count_target = linear_left * gear_ratio / (diameter * diamete_ratio / hole_number);  //右外圈线速度对应的孔数

            val_right = linear_right * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,左轮
            val_left = linear_left * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,右

            left_Setpoint = val_left_count_target;
            right_Setpoint = val_right_count_target;
            */
            // 直行时，值越大速度越慢，基本超过200就不走了
            val_right = 0;
            val_left = 0;

            advance();
            run_direction = 'f';
            
            debug = "forward: " + String(val_right) + "," + String(val_left) + "  linear:" + String(linear) + " angular:" + String(angular);
            Serial.println(debug + "\n");
        } else if (linear < 0) {  //后退
            Serial.println("Go Backward!\n");
            linear = abs(linear);
            if (linear > max_linear)
                linear = max_linear;
            float linear_left = linear;   //左内圈线速度
            float linear_right = linear;  //右外圈线速度

            val_right_count_target = linear_right * gear_ratio / (diameter * diamete_ratio / hole_number);  //左内圈线速度对应的孔数
            val_left_count_target = linear_left * gear_ratio / (diameter / hole_number);  //右外圈线速度对应的孔数

            val_right = linear_right * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,左轮
            val_left = linear_left * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,右轮

            left_Setpoint = val_left_count_target;
            right_Setpoint = val_right_count_target;

            back();
            run_direction = 'b';
            
            debug = "back: " + String(val_right) + "," + String(val_left) + "  linear:" + String(linear) + " angular:" + String(angular);
        }

    } else if (angular > 0) {  //左转
        Serial.println("Turn Left!\n");
        if (linear > max_turn_line){  //限制最大转弯线速度
            angular = angular * max_turn_line / linear;
            linear = max_turn_line;
        } else if (linear == 0) {
            linear = max_turn_line;
        }

        float radius = linear / angular;  //计算半径
        if (radius < car_width / 2)  //如果计算的转弯半径小于最小半径,则设置为最小转弯半径
            radius = car_width / 2;

        float radius_left = radius - car_width / 2;   //左内圈半径
        float radius_right = radius + car_width / 2;  //右外圈半径

        float linear_left = radius_left * angular;    //左内圈线速度
        float linear_right = radius_right * angular;  //右外圈线速度

        if (linear == max_turn_line) {
            linear_left = 255 * (linear_left / linear_right);
            linear_right = 255;
        }

        val_right_count_target = linear_right * gear_ratio / (diameter / hole_number);  //左内圈线速度对应的孔数
        val_left_count_target = linear_left * gear_ratio / (diameter * diamete_ratio / hole_number);  //右外圈线速度对应的孔数

        val_right = linear_right * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,左轮
        val_left = linear_left * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,右轮

        left_Setpoint = val_left_count_target;
        right_Setpoint = val_right_count_target;

        run_direction = 'f';
        advance();
            
        debug = "turn left: " + String(val_right) + "," + String(val_left) + "  linear:" + String(linear) + " angular:" + String(angular);

    } else if (angular < 0) {  //右转
        Serial.println("Turn Right!");
        if (linear > max_turn_line){  //限制最大转弯线速度
            angular = angular * max_turn_line / linear;
            linear = max_turn_line;

        } else if (linear == 0) {
            linear = max_turn_line;
        }

        float radius = linear / angular;
        if (radius < car_width / 2)  //如果计算的转弯半径小于最小半径,则设置为最小转弯半径
            radius = car_width / 2;

        float radius_left = radius + car_width / 2;
        float radius_right = radius - car_width / 2;

        float linear_left = radius_left * angular;
        float linear_right = radius_right * angular;

        if (linear == max_turn_line) {
            linear_right = 255 * (linear_right / linear_left);
            linear_left = 255;
        }

        val_right_count_target = linear_right * gear_ratio / (diameter / hole_number);  //左内圈线速度对应的孔数
        val_left_count_target = linear_left * gear_ratio / (diameter * diamete_ratio / hole_number);  //右外圈线速度对应的孔数

        val_right = linear_right * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,左轮
        val_left = linear_left * (max_linear_pwd / max_linear);  //根据轮径参数计算出来的线速度对应的PWD值,右轮

        left_Setpoint = val_left_count_target;
        right_Setpoint = val_right_count_target;

        advance();
        run_direction = 'f';
            
        debug = "turn right: " + String(val_right) + "," + String(val_left) + "  linear:" + String(linear) + " angular:" + String(angular);
    }
    delay(1000);
    val_left_count_target = 0;
    left_Setpoint = 0;
    val_right_count_target = 0;
    right_Setpoint = 0;
    linear = 0;
    angular = 0;
    Stop();
    run_direction = 's'; 
}

/*
void PID_left() {
    Serial.println("********************************begin PID left");

    left_Input = count_left * 10;
    left_PID.Compute();

    val_left = val_left + left_Output;
    if (val_left > 255)
        val_left = 255;
    if (val_left < 0)
        val_left = 0;
    if (run_direction ==
        'f')  //根据刚刚调节后的小车电机PWM功率值，及时修正小车前进或者后退状态
        advance();
    if (run_direction == 'b')
        back();
    Serial.println("********************************end PID Left");
}
void PID_right() {
    Serial.println("********************************begin PID Right");

    right_Input = count_right * 10;
    right_PID.Compute();
    val_right = val_right + right_Output;
    if (val_right > 255)
        val_right = 255;
    if (val_right < 0)
        val_right = 0;
    if (run_direction == 'f')  //根据刚刚调节后的小车电机PWM功率值，及时修正小车前进或者后退状态
        advance();
    if (run_direction == 'b')
        back();
    Serial.println("********************************end PID Right");
}*/