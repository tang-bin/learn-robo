#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int w = img.width;
    int h = img.height;
    int i;
    int j;

    int move = 0;
    int wStart = w;
    int wEnd = 0;

    for (i = 0; i < h; i++)
    {
        int preIdx = i * w * 3;
        for (j = 0; j < w; j++)
        {
            int idx = preIdx + j * 3;
            if (img.data[idx] == white_pixel && img.data[idx + 1] == white_pixel && img.data[idx + 2] == white_pixel)
            {
                if(move == 0){
                    move = 1;
                    wStart = j;
                };
            } else if(move == 1){
                wEnd = j;
                break;
            }
        }
        if(move == 1) break;
    }

    if (move == 1)
    {
        int moveJ = (wStart + wEnd) / 2;
        if (wStart < 1 || wEnd > w - 1 || (wStart < 100 && wEnd > w - 100))
            drive_robot(0, 0); // too close, stop!
        else if (moveJ < w / 3)
            drive_robot(0.3, 0.3);
        else if (moveJ > 2 * w / 3)
            drive_robot(0.3, -0.3);
        else
            drive_robot(0.3, 0);
    }
    else
        drive_robot(0, 0);

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char **argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    printf("init");

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}