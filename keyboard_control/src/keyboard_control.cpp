#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define MAX_SPEED 0.5
#define MAX_TURN 1.0

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // Save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Apply new settings

    int c = getchar(); // Read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore old settings
    return c;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist twist;
    int c;

    printf("Control your robot!\n");
    printf("Use 'w' to move forward, 's' to move backward, 'a' to turn left, and 'd' to turn right.\n");
    printf("Press 'q' to quit.\n");

    while (ros::ok()) {
        c = getch();   // call non-blocking input function

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
            case 'q':
                return 0;
            default:
                twist.linear.x = 0;
                twist.angular.z = 0;
        }

        twist_pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}
