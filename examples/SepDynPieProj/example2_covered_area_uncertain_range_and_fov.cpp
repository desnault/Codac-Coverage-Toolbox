/*
 * @file example2_covered_area_uncertain_range_and_fov.cpp
 * @brief Example of computing the covered area using SepDynPieProj with uncertain sensor range and FOV.
 *
 * This example shows how to:
 * 1. Define a 3D trajectory for the vehicle (x, y, heading).
 * 2. Convert the trajectory into a TubeVector for interval-based projection.
 * 3. Define a detection area with a nominal range/FOV and an uncertain border (three-valued coverage: inside/uncertain/outside).
 * 4. Use SepDynPieProj to project the sensor coverage along the trajectory.
 * 5. Apply a paving algorithm (SIVIA) to display the covered area.
 * 6. Display the vehicle trajectory, tube, sensor coverage, and vehicle positions at selected times.
 *
 * Note on robot frame and FOV (delta angle):
 * - The robot frame is defined with x pointing forward, y to the left, and heading as the rotation around z-axis.
 * - The FOV (delta) interval defines the sensor's angular coverage relative to the vehicle's current heading.
 * - The uncertain FOV and range define the border of the coverage with possible uncertainty.
 * - SepDynPieProj internally handles angle wrap-around using modulo2pi to ensure FOV remains in [-π, π].
 *
 * This example demonstrates three-valued coverage (inside/uncertain/outside) for a directional sensor.
 */

#include "codac.h"
#include "vibes.h"

#include "SepDynPieProj.h"

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

// Nominal detection range
double detection_range = 1.0;

// Uncertain border of detection range
double detection_border = 0.5;

// Nominal sensor FOV (delta)
Interval fov = Interval(0).inflate(M_PI/4.);

// Uncertain FOV border
Interval uncertain_fov = Interval(0).inflate(1.2*M_PI/4.);



/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

// Projection time interval (whole trajectory)
Interval t_proj(t0, tf);

// Temporal resolution of projection
double eps_proj = 0.01;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of paving (smaller = finer discretization)
double paving_resolution = 0.2;

// Area to be paved (2D bounding box)
IntervalVector paving_area = {{-7, 7}, {-8, 8}};



int main()
{
    // Generate trajectory and TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Start VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Set axis limits and draw initial paving area
    fig.axis_limits(-9, 9, -9, 9);
    fig.draw_box(paving_area, "black");

    // Create the SepDynPieProj separator/projector with nominal and uncertain borders
    // This produces three-valued coverage: inside/uncertain/outside
    SepDynPieProj my_sep(my_tube, detection_range, detection_border, fov, uncertain_fov, t_proj, eps_proj, true);

    // Apply SIVIA paving algorithm
    // Classifies areas as detected (inside), uncertain, or not detected (outside)
    SIVIA(paving_area, my_sep, paving_resolution);

    // Draw vehicle trajectory and TubeVector
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display vehicle and sensor coverage at selected times
    std::vector<double> detection_time = {1.3, 6.5, 11.5};
    for (double t : detection_time)
    {
        // Get vehicle position and heading at time t
        double px = my_traj[0](t);
        double py = my_traj[1](t);
        double heading = my_traj[2](t);

        // Draw nominal detection area (inside coverage)
        fig.draw_pie(px, py, Interval(0, detection_range), heading + fov, "black");

        // Draw uncertain border (uncertain coverage)
        fig.draw_pie(px, py, Interval(0, detection_range + detection_border), heading + uncertain_fov, "black");

        // Draw vehicle at this time
        fig.draw_vehicle(t, &my_traj, 0.6);
    }

    // Show figure without displaying robot at the end
    fig.show(0);

    // End drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}