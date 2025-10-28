  #include <webots/robot.h>
  #include <webots/motor.h>
  #include <webots/display.h>
  #include <webots/gps.h>
  #include <webots/lidar.h>
  #include <webots/inertial_unit.h>
  #include <stdio.h>
  #include <math.h>
  #include <stdlib.h>
  #include <time.h>
  
  #define TIME_STEP 64
  #define MAP_SIZE_METERS 2.0
  #define GRID_WIDTH 200
  #define GRID_HEIGHT 200
  #define RESOLUTION (MAP_SIZE_METERS / GRID_WIDTH)
  #define ORIGIN_X (GRID_WIDTH / 2)
  #define ORIGIN_Y (GRID_HEIGHT / 2)
  #define HIT_INCREMENT 5
  #define OCCUPANCY_THRESHOLD 20
  #define OBSTACLE_DISTANCE 0.15
  #define STUCK_THRESHOLD 100.0
  #define ESCAPE_DURATION 25
  #define RANDOM_TURN_CHANCE 0.02
  #define RANDOM_TURN_DURATION 20
  #define RANDOM_TURN_STRENGTH 0.5
  
  int confidence_grid[GRID_WIDTH][GRID_HEIGHT] = {0};
  int display_grid[GRID_WIDTH][GRID_HEIGHT] = {0};
  int escape_counter = 0;
  int random_turn_counter = 0;
  double random_turn_direction = 1.0;
  
  FILE *map_file;
  FILE *path_file;
  
  void world_to_grid(double world_x, double world_z, int *grid_x, int *grid_y) {
    *grid_x = (int)(world_x / RESOLUTION) + ORIGIN_X;
    *grid_y = ORIGIN_Y - (int)(world_z / RESOLUTION);
    if (*grid_x < 0) *grid_x = 0;
    if (*grid_x >= GRID_WIDTH) *grid_x = GRID_WIDTH - 1;
    if (*grid_y < 0) *grid_y = 0;
    if (*grid_y >= GRID_HEIGHT) *grid_y = GRID_HEIGHT - 1;
  }
  
  int main() {
    wb_robot_init();
    srand(time(NULL));
  
    // Archivos CSV
    map_file = fopen("obstacles.csv", "w");
    path_file = fopen("trajectory.csv", "w");
    fprintf(map_file, "x_world,z_world\n");
    fprintf(path_file, "x_world,z_world\n");
  
    // Motores
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
  
    // Velocidad máxima real del robot
    double MAX_SPEED = wb_motor_get_max_velocity(left_motor);
  
    // Sensores
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  
    WbDeviceTag imu = wb_robot_get_device("imu");
    wb_inertial_unit_enable(imu, TIME_STEP);
  
    WbDeviceTag lidar = wb_robot_get_device("lidar");
    wb_lidar_enable(lidar, TIME_STEP);
    wb_lidar_enable_point_cloud(lidar);
  
    int lidar_res = wb_lidar_get_horizontal_resolution(lidar);
    double lidar_fov = wb_lidar_get_fov(lidar);
    double lidar_max = wb_lidar_get_max_range(lidar);
  
    // Display para mapa de ocupación
    WbDeviceTag display = wb_robot_get_device("display");
    wb_display_set_color(display, 0x000000);
    wb_display_fill_rectangle(display, 0, 0, GRID_WIDTH, GRID_HEIGHT);
  
    // Bucle principal
    while (wb_robot_step(TIME_STEP) != -1) {
      // === SENSORES ===
      const double *gps_values = wb_gps_get_values(gps);
      double robot_x = gps_values[0];
      double robot_z = gps_values[2];
  
      const double *imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
      double robot_theta = imu_values[2];
  
      const float *ranges = wb_lidar_get_range_image(lidar);
  
      // Guardar posición del robot
      fprintf(path_file, "%f,%f\n", robot_x, robot_z);
  
      // === MAPEO DE OBSTÁCULOS ===
      for (int i = 0; i < lidar_res; i++) {
        double r = ranges[i];
        if (isinf(r) || r >= lidar_max) continue;
        if (r < wb_lidar_get_min_range(lidar)) continue;
  
        double angle_lidar = -lidar_fov / 2.0 + (double)i / lidar_res * lidar_fov;
        double absolute_angle = angle_lidar - robot_theta - M_PI / 2.0;
        double point_x = robot_x + r * cos(absolute_angle);
        double point_z = robot_z + r * sin(absolute_angle);
  
        int gx, gy;
        world_to_grid(point_x, point_z, &gx, &gy);
  
        if (confidence_grid[gx][gy] < 100)
          confidence_grid[gx][gy] += HIT_INCREMENT;
  
        if (confidence_grid[gx][gy] > OCCUPANCY_THRESHOLD && display_grid[gx][gy] == 0) {
          wb_display_set_color(display, 0xFFFFFF);
          wb_display_draw_pixel(display, gx, gy);
          display_grid[gx][gy] = 1;
          fprintf(map_file, "%f,%f\n", point_x, point_z);
        }
      }
  
      // Dibujar posición actual del robot
      int grid_x, grid_y;
      world_to_grid(robot_x, robot_z, &grid_x, &grid_y);
      wb_display_set_color(display, 0xFF0000);
      wb_display_fill_rectangle(display, grid_x - 1, grid_y - 1, 3, 3);
  
      // === CONTROL DE MOVIMIENTO ===
      double left_speed = MAX_SPEED;
      double right_speed = MAX_SPEED;
  
      if (escape_counter > 0) {
        // Modo escape (retroceder y girar)
        left_speed = -MAX_SPEED;
        right_speed = MAX_SPEED;
        escape_counter--;
      } else if (random_turn_counter > 0) {
        // Giro aleatorio ocasional
        left_speed = MAX_SPEED * (1.0 - RANDOM_TURN_STRENGTH * random_turn_direction);
        right_speed = MAX_SPEED * (1.0 + RANDOM_TURN_STRENGTH * random_turn_direction);
        random_turn_counter--;
      } else {
        // Evasión de obstáculos
        double left_influence = 0.0, right_influence = 0.0;
        int mid_point = lidar_res / 2;
  
        for (int i = 0; i < lidar_res; ++i) {
          if (ranges[i] < OBSTACLE_DISTANCE) {
            double proximity = 1.0 - (ranges[i] / OBSTACLE_DISTANCE);
            if (i < mid_point) right_influence += proximity;
            else left_influence += proximity;
          }
        }
  
        // Si está atrapado, entrar en modo escape
        if (left_influence > STUCK_THRESHOLD && right_influence > STUCK_THRESHOLD) {
          escape_counter = ESCAPE_DURATION;
        } else {
          // Control diferencial
          double turn_influence = right_influence - left_influence;
          double base_speed = MAX_SPEED * 0.75;
          left_speed = base_speed + turn_influence * (MAX_SPEED / 20.0);
          right_speed = base_speed - turn_influence * (MAX_SPEED / 20.0);
  
          // Giro aleatorio ocasional
          if ((double)rand() / RAND_MAX < RANDOM_TURN_CHANCE) {
            random_turn_counter = RANDOM_TURN_DURATION;
            random_turn_direction = (rand() % 2 == 0) ? 1.0 : -1.0;
          }
        }
      }
  
      // === LIMITAR VELOCIDADES (evita los warnings) ===
      if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
      if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
      if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
      if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;
  
      // Aplicar velocidades finales
      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);
    }
  
    fclose(map_file);
    fclose(path_file);
    wb_robot_cleanup();
    return 0;
  }
  