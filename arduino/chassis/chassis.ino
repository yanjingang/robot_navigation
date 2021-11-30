/**
   @file chassis.ino
   @author yanjingang@mail.com
   @brief  arduino底盘
   @version 0.1
   @date 2021-11-30

   @copyright Copyright (c) 2021

*/
#include <IRremote.h>  //包含红外库
#include <ros.h>  //包含ros库
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>  //包含ros msg类型库
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

int infraredPin = 3;                //红外线接收器端口
long INFRARED_PLAY = 0x00FFA857;    //红外遥控器上的PLAY键指令
long INFRARED_LEFT = 0x00FFE01F;    //红外遥控器上的左键指令
long INFRARED_RIGHT = 0x00FF906F;   //红外遥控器上的右键指令
long INFRARED_UP = 0x00FF02FD;      //红外遥控器上的上键指令
long INFRARED_DOWN = 0x00FF9867;    //红外遥控器上的下键指令
long INFRARED_C = 0x00FFB04F;       //红外遥控器上的C指令
long INFRARED_TEST = 0x00FF22DD;    //红外遥控器上的TEST键指令
long INFRARED_RETURN = 0x00FFC23D;  //红外遥控器上的RETURN键指令
int ledLightPin = 4;                // led常亮端口
int ledPin = 13;                    // led控制提醒端口
int rightMotor = 5;                 // 定义uno的pin 5 向 rightMotor 输出
int rightMotor_ = 6;                // 定义uno的pin 6 向 rightMotor_ 输出
int leftMotor = 9;                  // 定义uno的pin 9 向 leftMotor 输出
int leftMotor_ = 10;                // 定义uno的pin 10 向 leftMotor_ 输出

IRrecv irrecv(infraredPin);  //初始化红外接收器对象
decode_results results;      //定义results变量为红外结果存放位置

ros::NodeHandle nh;  // ros节点句柄

/* pub 底盘里程计数据 */
std_msgs::String str_msg;
char hello[] = "I'm tank!";
ros::Publisher pub("/tank/data", &str_msg);

/* sub 导航的/cmd_vel指令并执行 */
#define max_linear 20      //当前电机最大线速度cm/秒
#define max_turn_line 18   //最大转弯线速度
#define car_width 27       //小车宽度
#define car_length 27      //小车长度
char run_direction = 'f';  //方向：f:前进;b:后退;s:stop
int linear = 0;            // 15; //线速度：cm/second
int angular = 0;           // 1; //角速度：ros的angular.z
void control(const geometry_msgs::Twist& cmd) {
    angular = cmd.angular.z;  //角速度：=0直行；>0左转；<0右转
    linear = cmd.linear.x * 100;  //线速度：ROS中的单位是m/s，这里换算成cm单位；>0前进，<0后退

    // execute(long(cmd.data));  //执行指令
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &control);

/* 指令执行 */
bool cmd_keep = true;  //是否保持末次指令一直执行
void execute(long cmd) {
    if (cmd == INFRARED_TEST) {  //遥控器按下了TEST键，不保持末次指令
        cmd_keep = false;
    } else if (cmd == INFRARED_RETURN) {  //遥控器按下了返回键，保持末次指令一直执行
        cmd_keep = true;
    } else if (cmd == INFRARED_PLAY || cmd == INFRARED_UP) {  //遥控器按下了PLAY/UP键，前进
        digitalWrite(ledPin, HIGH);
        // forward 向前转
        digitalWrite(rightMotor, HIGH);  //给高电平
        digitalWrite(rightMotor_, LOW);  //给低电平
        digitalWrite(leftMotor, HIGH);   //给高电平
        digitalWrite(leftMotor_, LOW);   //给低电平
    } else if (cmd == INFRARED_DOWN) {   //遥控器按下了－键，后退
        digitalWrite(ledPin, HIGH);
        // back 向后转
        digitalWrite(rightMotor, LOW);
        digitalWrite(rightMotor_, HIGH);
        digitalWrite(leftMotor, LOW);
        digitalWrite(leftMotor_, HIGH);
    } else if (cmd == INFRARED_LEFT) {  //遥控器按下了左键，左转
        digitalWrite(ledPin, HIGH);
        // left 左转
        digitalWrite(leftMotor, LOW);
        digitalWrite(leftMotor_, LOW);
        digitalWrite(rightMotor, HIGH);
        digitalWrite(rightMotor_, LOW);
    } else if (cmd == INFRARED_RIGHT) {  //遥控器按下了右键，右转
        digitalWrite(ledPin, HIGH);
        // right 右转
        digitalWrite(rightMotor, LOW);
        digitalWrite(rightMotor_, LOW);
        digitalWrite(leftMotor, HIGH);
        digitalWrite(leftMotor_, LOW);
    } else if (cmd == INFRARED_C ||
               cmd_keep == false) {  //遥控器按下了C键或非持续类指令，停止
        digitalWrite(ledPin, LOW);
        // stop 停止
        digitalWrite(rightMotor, LOW);
        digitalWrite(rightMotor_, LOW);
        digitalWrite(leftMotor, LOW);
        digitalWrite(leftMotor_, LOW);
    }
}

/* 执行ros指令 */
void execute_ros() {
    cmd_keep = false;      //
    if (angular == 0) {    //直行
        if (linear > 0) {  //前进
            Serial.println("Go Forward!\n");
            if (linear > max_linear)
                linear = max_linear;

            execute(INFRARED_UP);
            run_direction = 'f';
        } else if (linear < 0) {  //后退
            Serial.println("Go Backward!\n");
            linear = abs(linear);
            if (linear > max_linear)
                linear = max_linear;

            execute(INFRARED_DOWN);
            run_direction = 'b';
        }
    } else if (angular > 0) {  //左转
        Serial.println("Turn Left!\n");
        if (linear > max_turn_line) {  //限制最大转弯线速度
            angular = angular * max_turn_line / linear;
            linear = max_turn_line;
        } else if (linear == 0) {
            linear = max_turn_line;
        }
        float radius = linear / angular;  //计算半径
        if (radius <
            car_width /
                2)  //如果计算的转弯半径小于最小半径,则设置为最小转弯半径
            radius = car_width / 2;

        execute(INFRARED_LEFT);
        run_direction = 'f';
    } else if (angular < 0) {  //右转
        Serial.println("Turn Right!");
        if (linear > max_turn_line) {  //限制最大转弯线速度
            angular = angular * max_turn_line / linear;
            linear = max_turn_line;
        } else if (linear == 0) {
            linear = max_turn_line;
        }
        float radius = linear / angular;
        if (radius <
            car_width /
                2)  //如果计算的转弯半径小于最小半径,则设置为最小转弯半径
            radius = car_width / 2;
        float radius_left = radius + car_width / 2;
        float radius_right = radius - car_width / 2;
        float linear_left = radius_left * angular;
        float linear_right = radius_right * angular;
        if (linear == max_turn_line) {
            linear_right = 255 * (linear_right / linear_left);
            linear_left = 255;
        }

        execute(INFRARED_RIGHT);
        run_direction = 'f';
    }
    // delay(1000);
    // linear = 0;
    // angular = 0;
    // execute(INFRARED_C);
    // run_direction = 's';
}

/* 履带坦克 */
void setup() {
    //红外接收器端口初始化
    pinMode(infraredPin, INPUT);
    //初始化电机IO,模式为OUTPUT 输出模式
    pinMode(rightMotor, OUTPUT);
    pinMode(rightMotor_, OUTPUT);
    pinMode(leftMotor, OUTPUT);
    pinMode(leftMotor_, OUTPUT);

    // led端口初始化
    pinMode(ledPin, OUTPUT);
    pinMode(ledLightPin, OUTPUT);
    digitalWrite(ledLightPin, HIGH);
    //波特率57600
    // Serial.begin(57600);
    //启动红外解码
    irrecv.enableIRIn();

    // ROS节点初始化
    nh.initNode();
    nh.advertise(pub);  // publish
    nh.subscribe(sub);  // subscribe
}

void loop() {
    //检测红外信号
    if (irrecv.decode(
            &results)) {  //是否接收到解码数据,把接收到的数据存储在变量results中
        // Serial.println(results.value,
        // HEX);//接收到的数据以16进制的方式在串口输出 执行指令
        execute(results.value);

        //继续等待接收下一组信号
        irrecv.resume();

        // led熄灭
        delay(200);
        digitalWrite(ledPin, LOW);
    } else {
        //执行来自ros的cmd_vel指令
        execute_ros();
    }

    /*
      //pub ros消息
      str_msg.data = hello;
      pub.publish(&str_msg);//publish a message
      delay(10);
    */
    nh.spinOnce();
}
