/*
 * area_search_manager.c
 */

#include "area_search_manager.h"

#include <math.h>

#define PI 3.14159265

static double area_width_m;
static double area_length_m;
static int area_num_passes;
static int area_stops_per_pass;
static pose2d_t area_start_pose;

static int num_destinations_reached;

void area_search_manager_generate_area(double width_m, double length_m, int num_passes, int stops_per_pass, pose2d_t start_pose) {
	// Set trajectory values
	area_width_m = width_m;
	area_length_m = length_m;
	area_num_passes = num_passes;
	area_stops_per_pass = stops_per_pass;
	area_start_pose = start_pose;
	num_destinations_reached = 1;
}

pose2d_t area_search_manager_retrieve_next_destination(bool* line_complete) {
	// Calculate coordinates to go to in robot's original frame
	int pass_number = num_destinations_reached / (area_stops_per_pass + 2);
	double y_orig = pass_number * area_width_m / (area_num_passes - 1);

	double stop_number = num_destinations_reached % (area_stops_per_pass + 2);
	*line_complete = stop_number == 0; // Set line complete to true if next stop is first of the next pass
	if (pass_number % 2 == 1) {
		stop_number = (area_stops_per_pass + 1) - stop_number;
	}
	double x_orig = stop_number * area_length_m / (area_stops_per_pass + 1);

	// Convert coordinates to world frame
	double x_world = x_orig * cos(area_start_pose.theta) - y_orig * sin(area_start_pose.theta) + area_start_pose.x;
	double y_world = x_orig * sin(area_start_pose.theta) + y_orig * cos(area_start_pose.theta) + area_start_pose.y;

	// Theta will be either be the original robot pose or pi radians from it
	double theta_world = area_start_pose.theta;
	if (pass_number % 2 == 1) {
		theta_world += theta_world > 0 ? -PI : PI;
	}

	pose2d_t next_pose = {x_world, y_world, theta_world};
	return next_pose;
}

bool area_search_manager_is_complete() {
	// 2 intermediate stops per pass * number of passes
	return num_destinations_reached >= (area_stops_per_pass + 2) * area_num_passes;
}
