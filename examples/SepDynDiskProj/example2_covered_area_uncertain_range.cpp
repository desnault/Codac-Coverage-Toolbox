/*
 * @file example2_covered_area_uncertain_range.cpp
 * @brief Example of computing the covered area using SepDynDiskProj with a three-valued (uncertain border) sensor.
 *
 * This example shows how to:
 * 1. Define a 3D trajectory for the robot.
 * 2. Convert the trajectory into a tube (TubeVector) for interval-based projection.
 * 3. Define a sensor coverage as a disk with an uncertain border (three-valued coverage).
 *    - Inside: always detected
 *    - Uncertain: possibly detected (border)
 *    - Outside: not detected
 * 4. Use SepDynDiskProj to project the sensor coverage along the trajectory.
 * 5. Apply a paving algorithm (SIVIA) to visualize the covered area.
 * 6. Display the robot trajectory, tube, sensor coverage, and vehicle positions at selected times.
 *
 * This example demonstrates how to model and visualize uncertainty in the sensor coverage.
 */

#include "codac.h"
#include "vibes.h"

#include "SepDynDiskProj.h"

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

// Sensor detection disk radius (always detected area)
double detection_range = 1.0;

// Width of the uncertain border (possibly detected area)
double detection_border = 0.5;



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
double paving_resolution = 0.2;

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

    // Create the SepDynDiskProj separator/projector
    // SepDynDiskProj allows three-valued coverage: inside / uncertain / outside
    SepDynDiskProj my_sep(my_tube, detection_range, detection_border, t_proj, eps_proj, true);

    // Paving algorithm
    // The SIVIA algorithm classifies the area as:
    // - Detected (inside the disk)
    // - Possibly detected (uncertain border)
    // - Not detected (outside)
    // The resulting map shows the coverage with uncertainty along the trajectory.
    SIVIA(paving_area, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the robot and sensor coverage at selected times
    std::vector<double> detection_time = {1.3, 6.5, 11.5};
    for (int i=0; i<detection_time.size(); i++)
    {
        // Coordinates of the robot at the current time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);

        // Draw the detected area (inner disk)
        fig.draw_circle(px, py, detection_range, "black");

        // Draw the uncertain border (outer disk)
        fig.draw_circle(px, py, detection_range + detection_border, "black");

        // Draw the robot vehicle at this time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.8);
    }

    // Show figure without displaying the robot at the end
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}