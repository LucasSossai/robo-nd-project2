#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the comman_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //  Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    // Print if error
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This function check if the pixel is white
bool IsWhitePixel(int r, int g , int b)
{
    int whitePixel = 255;
    if (r == whitePixel && g == whitePixel && b == whitePixel)
    {
        return true;
    }
    return false;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    //determine in which region the white pixels are and call the drive_robot function
    int leftFrontier = img.width / 3;
    int rightFrontier = img.width * 2 / 3;
    int whitePixelCount = 0;
    int xAxisCount = 0;

    for (int i = 0; i + 2 < img.data.size(); i += 3)
    {
        int redValue = img.data[i];
        int greenValue = img.data[i + 1];
        int blueValue = img.data[i + 2];
        int whitePixel = 255;

        if (IsWhitePixel(redValue, greenValue, blueValue))
        {
            int xPos = (i % (img.width * 3)) / 3;
            xAxisCount += xPos;
            whitePixelCount += 1;
        }
    }
    // If no white pixels, stop the robot
    if (whitePixelCount == 0)
    {
        drive_robot(0.0, 0.0);
    }
    else // See in which frontier the mean of the white pixels are and drives the robot
    {
        int meanXPos = xAxisCount / whitePixelCount;
        if (meanXPos > rightFrontier) 
        {
            drive_robot(0.5, -0.5);
        }
        else if (meanXPos < leftFrontier)
        {
            drive_robot(0.5, 0.5);
        }
        else
        {
            drive_robot(0.5, 0.0);
        }
    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}