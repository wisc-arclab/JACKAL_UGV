#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define MAX_SPEED 0.5
#define MAX_TURN 1.0

int getch() {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering      
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::Publisher twist_pub = nh.advertise<geometry_msgs/Twist>("/cmd_vel", 1);

    geometry_msgs::Twist twist;
    int c;

    printf("Control your robot!\n");
    printf("---------------------------\n");
    printf("Moving around:\n");
    printf("   u    i    o\n");
    printf("   j    k    l\n");
    printf("   m    ,    .\n");
    printf("q/z : increase/decrease max speeds by 10%%\n");
    printf("w/x : increase/decrease only linear speed by 10%%\n");
    printf("e/c : increase/decrease only angular speed by 10%%\n");
    printf("anything else : stop\n");

    while (ros::ok()) {
        c = getch();   // call your non-blocking input function

        switch(c) {
            case 'w':
                twist.linear.x = MAX_SPEED;
                twist.angular.z = 0;
                break;
            case 's':
                twist.linear.x = -MAX_SPEED;
                twist.angular.z = 0;
                break;
            case 'a':
                twist.angular.z = MAX_TURN;
                twist.linear.x = 0;
                break;
            case 'd':
                twist.angular.z = -MAX_TURN;
                twist.linear.x = 0;
                break;
            default:
                twist.linear.x = 0;
                twist.angular.z = 0;
        }

        twist_pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}
