#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <numeric>

using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot to the target.");
    
    // Request service with velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the DriveToTarget service and pass the requested velocities
    if (!client.call(srv)) {
	    ROS_ERROR("Failed to call service DriveToTarget.");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    // sensor_msgs/Image Message
    // http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
    //
    // Compact Message Definition
    // std_msgs/Header header
    // uint32 height
    // uint32 width
    // string encoding
    // uint8 is_bigendian
    // uint32 step
    // uint8[] data


    // Initial variables
    // Initial white pixel counter vector (in step direction)
    // Base on the range of left, forward, right (unit: pixel)
    // Find the start, end index of each range in the vector
    int height = img.height;
    int step = img.step;

/*
    int vec_cout[step] = {};
    
    unsigned long cout_left    = 0;
    unsigned long cout_forward = 0;
    unsigned long cout_right   = 0;
    
    unsigned int left_range  = (unsigned int)((float)step / 3.0 + 1);
    unsigned int right_range = (unsigned int)((float)step / 3.0 + 1);
    
    unsigned int idx_left_start    = 0;
    unsigned int idx_left_end      = left_range;
    unsigned int idx_forward_start = left_range + 1;
    unsigned int idx_forward_end   = step - right_range - 1;
    unsigned int idx_right_start   = step - right_range;
    unsigned int idx_right_end     = step - 1;
*/
    
    float x = 0.0;
    float z = 0.0;
  
    float offset_accumulated = 0;
    int count_total = 0;

/*
    // Loop through each pixel
    //   Outer loop height from top to botton
    //       Inner loop step from left to right
    //           Identify pixel is a bright one or not
    //           Increase counter in the white pixel counter vector
*/
  
    // We will loop through all the pixel from left to right then top to bottom.
    // Outer loop will be height direction, inner loop will be step direction.
    // Assuming the ball is symmetry, we can calculate the offset from the center 
    // of picture to the center of the ball by following approach.
    // 1. White pixel detected, calculate the offset from the center of the picture
    // 2. Accumulate those offset and count how many pixel detected.
    // 3. Loop through all the pixel then divide accumulated offset by total counts
    // 4. You get an approximate offset from the center of the picture
  
    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < step; j++) {
            if (img.data[i * step + j] == white_pixel) {
                // vec_cout[j]++;
                offset_accumulated += j - step / 2.0;
                count_total++;
            }
        }
    }

/*
    // Sum up the counter in different range
    unsigned long init = 0;
    cout_left    = accumulate(vec_cout + idx_left_start,    vec_cout + idx_left_end,    init);
    cout_forward = accumulate(vec_cout + idx_forward_start, vec_cout + idx_forward_end, init);
    cout_right   = accumulate(vec_cout + idx_right_start,   vec_cout + idx_right_end,   init);
*/

/*
    // Sum up the white pixel counter vector
    int sum = 0;
    sum = accumulate(vec_cout, vec_cout + step, 0);

    // Find the index of first maximum value in the vector
    unsigned int idx_first_max = distance(vec_cout, max_element(vec_cout, vec_cout + step));
    // Find the index of last maximum value in the vector
    unsigned int idx_last_max = step + distance(max_element(vec_cout, vec_cout + step), vec_cout);
    // Calculate the center index with first and last maximum value in the vector
    // This will be the estimated center of the white ball
    unsigned int idx_center_ball = (idx_first_max + idx_last_max) / 2.0 + 1;
*/

    // Base on the result, determine the action
    // If there is no pixel, then tell the car to stop
    if (count_total == 0) {
        x = 0.0;
        z = 0.0;
    }
    else {
        x = 0.1;
        // z = 0.5 * (step / 2.0 - idx_center_ball);

        // Calculate the average offset (from -step/2.0 to +step/2.0)
        // Normalize the average offset (from -1.0 to 1.0)
        // Multiply with magic number -4.0 to turn
        z = -4.0 * offset_accumulated / count_total / (step /2.0);
    }
    
    // Send request to service
    drive_robot(x, z);
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
