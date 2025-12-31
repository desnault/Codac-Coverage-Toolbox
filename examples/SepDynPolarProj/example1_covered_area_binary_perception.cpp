/*
 * @file example1_covered_area_binary_perception.cpp
 * @brief Example of computing the covered area using SepDynPolarProj with a binary perception sensor.
 *
 * This example shows how to:
 * 1. Define a 3D trajectory for the vehicle (x, y, heading).
 * 2. Convert the trajectory into a TubeVector for interval-based projection.
 * 3. Define a binary detection area using a range and field-of-view (FOV) in the robot frame.
 * 4. Use SepDynPolarProj to project the sensor coverage along the trajectory.
 * 5. Apply a paving algorithm (SIVIA) to display the covered area.
 * 6. Display the vehicle trajectory, tube, sensor coverage, and vehicle positions at selected times.
 *
 * Note on robot frame and FOV (delta angle):
 * - The robot frame is defined with x pointing forward, y to the left, and heading as the rotation around z-axis.
 * - delta interval defines the sensor's angular field-of-view in the robot frame, relative to the current heading.
 * - 0 corresponds to the forward direction, positive angles rotate counterclockwise.
 * - SepDynPolarProj internally handles angle wrap-around using modulo2pi to ensure the FOV remains in [-pi, pi].
 *
 * This example demonstrates binary coverage (inside/outside) for a directional sensor.
 */

#include "codac.h"
#include "vibes.h"

#include "SepDynPolarProj.h"

#include <vector>
#include <iostream>

using namespace codac;



/*==================================================================================
 * TRAJECTORY SETUP
 *==================================================================================*/

// Initial and final time of the trajectory
const double t0 = 0.55;
const double tf = 12;

// Time step between trajectory points
const double dt = 0.01;

// Time domain for the trajectory
const Interval tdomain(t0, tf);

// Temporal definition of the trajectory using TFunction
// Format: (x(t); y(t); heading(t))
TFunction path("( 5*cos(0.5*t)*sin(t) ; 5*cos(0.5*t)*cos(t) ; atan2(-2.5*sin(0.5*t)*cos(t) - 5*cos(0.5*t)*sin(t) , -2.5*sin(0.5*t)*sin(t) + 5*cos(0.5*t)*cos(t)))");

// Generate TrajectoryVector from the path
TrajectoryVector generate_trajectory()
{
    TrajectoryVector traj(tdomain, path, dt);
    traj[2] = traj[2].make_continuous(); // Ensure heading continuity
    return traj;
}

// Convert TrajectoryVector to TubeVector for contraction
TubeVector generate_tube(TrajectoryVector &traj)
{
    TubeVector tube(traj, dt);
    return tube;
}



/*==================================================================================
 * DETECTION PARAMETERS
 *==================================================================================*/

// Binary sensor detection range [r_min, r_max]
Interval detection_range(1, 2);

// Sensor field-of-view in the robot frame (delta angle)
// Interval centered at 0, inflated by 45 degrees (pi/4)
Interval fov = Interval(0).inflate(M_PI/4.);



/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

// Time interval for projecting the coverage (whole trajectory)
Interval t_proj(t0, tf);

// Temporal resolution of projection
double eps_proj = 0.01;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of the paving (smaller = more refined discretization)
double paving_resolution = 0.2;

// Area to be paved (2D bounding box)
IntervalVector paving_area = {{-7, 7}, {-8, 8}};



int main()
{
    // Generate trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Start VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Set axis limits and draw initial paving area
    fig.axis_limits(-9, 9, -9, 9);
    fig.draw_box(paving_area, "black");

    // Create the SepDynPolarProj separator/projector
    // This combines CtcDynPolar separator with SepProj projection along the trajectory
    SepDynPolarProj my_sep(my_tube, detection_range, fov, t_proj, eps_proj, true);

    // Apply SIVIA paving algorithm
    // Classifies areas as detected (inside coverage) or not detected (outside coverage)
    SIVIA(paving_area, my_sep, paving_resolution);

    // Draw trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display vehicle and sensor coverage at selected times
    std::vector<double> detection_time = {1.3, 6.5, 11.5};
    for (int i = 0; i < detection_time.size(); i++)
    {
        // Get robot position and heading at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Draw the directional detection area (pie) in the robot frame
        fig.draw_pie(px, py, detection_range, heading + fov, "black");

        // Draw vehicle at this time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.6);
    }

    // Show figure without displaying robot at the end
    fig.show(0);

    // End drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}