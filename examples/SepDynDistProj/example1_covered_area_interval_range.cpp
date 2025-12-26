/*
 * @file example1_covered_area_interval_range.cpp
 * @brief Example of computing the covered area using SepDynDistProj with a binary perception sensor.
 *
 * This example shows how to:
 * 1. Define a 3D trajectory for the robot.
 * 2. Convert the trajectory into a tube (TubeVector) for interval-based projection.
 * 3. Define a binary detection range (interval) for the ranging sensor.
 * 4. Use SepDynDistProj to project the sensor coverage along the trajectory.
 * 5. Apply a paving algorithm (SIVIA) to display the covered area.
 * 6. Display the robot trajectory, tube, sensor coverage, and vehicle positions at selected times.
 *
 * This is a beginner-friendly example demonstrating how to compute and visualize the covered area
 * for a sensor whose coverage is defined by an interval (binary coverage).
*/

#include "codac.h"
#include "vibes.h"

#include "SepDynDistProj.h"

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
    traj[2] = traj[2].make_continuous(); // Make heading continuous to handle wrap-around at 0/2pi
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

// Binary sensor detection range (interval [r_min, r_max])
Interval detection_range(1, 2);



/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

// Time period over which to project the sensor coverage
// Here we consider the whole trajectory
Interval t_proj(t0, tf);

// Temporal resolution of the projection (smaller = finer resolution)
double eps_proj = 0.01;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of the paving (smaller = more refined discretization)
double paving_resolution = 0.1;

// 2D area to be paved (x, y)
IntervalVector paving_area = {{-7, 7}, {-8, 8}};



int main()
{
    // Generate the robot trajectory and the corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Set the map limits
    fig.axis_limits(-9,9,-9,9);

    // Draw the initial paving area (black box)
    fig.draw_box(paving_area, "black");

    // Create the separator / projector
    // SepDynDistProj combines SepDynDist (sensor coverage along trajectory) with SepProj
    // (projection along a temporal interval). It allows us to compute the covered area.
    SepDynDistProj my_sep(my_tube, detection_range, t_proj, eps_proj, true);

    // Paving algorithm
    // The paving algorithm classifies the area as:
    // - Detected (inside the coverage)
    // - Not detected (outside the coverage)
    // The resulting map shows the covered area by the sensor along the trajectory.
    SIVIA(paving_area, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the robot and sensor coverage at selected times
    std::vector<double> detection_time = {1.3, 6.5, 11.5};
    for (int i=0; i<detection_time.size(); i++)
    {
        // Display the detection annulus at the considered time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);

        // Draw the detection range (upper and lower bounds)
        fig.draw_circle(px, py, detection_range.ub(), "black");
        fig.draw_circle(px, py, detection_range.lb(), "black");

        // Draw the robot vehicle at this time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.8);
    }

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}