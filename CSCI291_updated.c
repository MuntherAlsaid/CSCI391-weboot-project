#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
  wb_robot_init();
  bool hasMoved = false;
  bool stopped = false;

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  WbDeviceTag light_sensor = wb_robot_get_device("ls0");
  wb_light_sensor_enable(light_sensor, TIME_STEP);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  for (int i = 0; i < 10; i++) {
    wb_robot_step(TIME_STEP);
  }

  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;


  double Bright_max = 0.0;
  double bright_cords_x = 0.0;
  double bright_cords_y = 0.0;


  const double *initial_coords = wb_gps_get_values(gps);
  double initial_x = initial_coords[0];
  double initial_y = initial_coords[1];
  printf("initial coordinates (%f, %f) \n", initial_x, initial_y);

  while (wb_robot_step(TIME_STEP) != -1 && stopped == false) {
    const double *current_coords = wb_gps_get_values(gps);
    double current_x = current_coords[0];
    double current_y = current_coords[1];
    printf("current coordinates: (%f, %f)\n", current_x, current_y);

    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
    bool left_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;

    if (front_wall) {
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else {
      if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        left_speed = MAX_SPEED / 8;
        right_speed = MAX_SPEED;
      }

      if (left_corner) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 8;
      }
    }

    double Bright_current = wb_light_sensor_get_value(light_sensor);
    printf("Current Brightness: %f at coordinates (%f, %f)\n", Bright_current, current_x, current_y);

    if (Bright_current > Bright_max) {
      Bright_max = Bright_current;
      bright_cords_x = current_x;
      bright_cords_y = current_y;
    }

    printf("Max Brightness: %f at coordinates (%f, %f) \n", Bright_max, bright_cords_x, bright_cords_y );

    if (!hasMoved && (fabs(current_x - initial_x) > 0.1 || fabs(current_y - initial_y) > 0.1)) {
      hasMoved = true;
    }
    if (hasMoved && fabs(current_x - initial_x) < 0.1 && fabs(current_y - initial_y) < 0.1) {
      printf("Returned to initial coordinates. Stopping.\n");
      left_speed = 0;
      right_speed = 0;
      stopped = true;
    }

    wb_motor_set_velocity(left_motor, left_speed / 2);
    wb_motor_set_velocity(right_motor, right_speed / 2);
  }

  wb_robot_cleanup();
  return 0;
}
