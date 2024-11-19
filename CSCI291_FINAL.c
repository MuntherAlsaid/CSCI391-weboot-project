#include <webots/robot.h> // Include Webots robot control library
#include <webots/motor.h> // Include Webots motor control library
#include <webots/light_sensor.h> // Include Webots light sensor library
#include <webots/distance_sensor.h> // Include Webots distance sensor library
#include <webots/gps.h> // Include Webots GPS sensor library
#include <stdio.h> // Standard I/O library for printing to the console
#include <math.h> // Math library for operations like fabs()

#define TIME_STEP 64 // Simulation step time in milliseconds
#define MAX_SPEED 6.28 // Maximum motor speed

int main(int argc, char **argv) {
  wb_robot_init();
  bool hasMoved = false; // Ftrack if the robot has moved
  bool stopped = false; // determine when the robot should stop after returning to the start
  bool stopped_light = false; // Flag to determine when the robot reaches the brightest point

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Set motors to velocity control mode (infinite position)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); // Start with zero velocity
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize proximity sensors
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name); // Get proximity sensor device
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  // Initialize light sensor
  WbDeviceTag light_sensor = wb_robot_get_device("ls0"); // Assuming only one light sensor
  wb_light_sensor_enable(light_sensor, TIME_STEP); // Enable light sensor

  // Initialize GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP); // Enable GPS sensor

  // Wait for sensors to initialize
  for (int i = 0; i < 10; i++) {
    wb_robot_step(TIME_STEP);
  }

  double left_speed = MAX_SPEED; // Default left motor speed
  double right_speed = MAX_SPEED; // Default right motor speed

  // Initialize variables
  double Bright_max = 0.0; // Maximum brightness observed
  double bright_cords_x = 0.0; // X-coordinate of max brightness
  double bright_cords_y = 0.0; // Y-coordinate of max brightness

  // Get and print initial GPS coordinates
  const double *initial_coords = wb_gps_get_values(gps);
  double initial_x = initial_coords[0];
  double initial_y = initial_coords[1];
  printf("initial coordinates (%f, %f) \n", initial_x, initial_y);

  // Main loop: Navigate maze and detect maximum brightness
  while (wb_robot_step(TIME_STEP) != -1 && stopped == false) {
    const double *current_coords = wb_gps_get_values(gps); // Get current GPS coordinates
    double current_x = current_coords[0];
    double current_y = current_coords[1];
    printf("current coordinates: (%f, %f)\n", current_x, current_y);

    // Detect obstacles using proximity sensors
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
    bool left_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;

    
    if (front_wall) { // Front wall detected
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED; //Turn
    } else {
      if (left_wall) { // Left wall detected
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED; //Move forward
      } else { //No wall detected
        left_speed = MAX_SPEED / 8; //turn to the left
        right_speed = MAX_SPEED;
      }

      if (left_corner) { // Corner on the left detected
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 8; // Slight turn to the right
      }
    }

    // Check and update maximum brightness
    double Bright_current = wb_light_sensor_get_value(light_sensor); // Get current brightness
    printf("Current Brightness: %f at coordinates (%f, %f)\n", Bright_current, current_x, current_y);
    if (Bright_current > Bright_max) { // Update max brightness if current is greater
      Bright_max = Bright_current;
      bright_cords_x = current_x;
      bright_cords_y = current_y;
    }

    // Print max brightness and its coordinates
    printf("Max Brightness: %f at coordinates (%f, %f) \n", Bright_max, bright_cords_x, bright_cords_y);

    // Check if the robot has moved from the initial position
    if (!hasMoved && (fabs(current_x - initial_x) > 0.1 || fabs(current_y - initial_y) > 0.1)) {
      hasMoved = true; // Set flag once robot moves
    }

    // Check if the robot has returned to the initial position
    if (hasMoved && fabs(current_x - initial_x) < 0.1 && fabs(current_y - initial_y) < 0.1) {
      printf("Returned to initial coordinates. Stopping.\n");
      left_speed = 0; // Stop motors
      right_speed = 0;
      stopped = true; // End the first phase
    }

    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed / 2); // Move at half speed
    wb_motor_set_velocity(right_motor, right_speed / 2);
  }

  // Second phase: Move to coordinates of highest brightness
  while (wb_robot_step(TIME_STEP) != -1 && stopped_light == false) {
    const double *current_coords = wb_gps_get_values(gps); // Get current GPS coordinates
    double current_x = current_coords[0];
    double current_y = current_coords[1];

    // Check if the robot has reached the brightest point
    if (fabs(current_x - bright_cords_x) < 0.1 && fabs(current_y - bright_cords_y) < 0.1) {
      printf("Reached the coordinates of highest brightness: (%f, %f)\n", bright_cords_x, bright_cords_y);
      left_speed = 0; // Stop motors
      right_speed = 0;
      stopped_light = true; // End the second phase
    }

    // Navigation logic (same as before)
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
    bool left_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;

    if (front_wall) {
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED; // Turn
    } else {
      if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED; // Move forward
      } else {
        left_speed = MAX_SPEED / 8;
        right_speed = MAX_SPEED;
      }

      if (left_corner) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 8;
      }
    }

    // Set motor speeds for reduced speed navigation
    wb_motor_set_velocity(left_motor, left_speed / 8);
    wb_motor_set_velocity(right_motor, right_speed / 8);
  }

  // Ensure motors are stopped before exiting
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  wb_robot_cleanup();
  return 0;
}
