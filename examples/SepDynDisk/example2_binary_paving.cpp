/********************************************************************************************
 * @file    example2_binary_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage paving using the SepDynDisk separator.
 *
 * @details
 * This example illustrates the use of the SepDynDisk class in a binary perception
 * context (covered / not covered) combined with a paving (SIVIA) algorithm.
 *
 * Scenario:
 * A robot equipped with a disk-shaped range sensor follows a known 2D trajectory.
 * We want to compute which parts of a given area (paving_area) are covered or not
 * covered at a specific detection time.
 *
 * Goal:
 * Demonstrate how to:
 *   - Instantiate a SepDynDisk separator with a disk range (binary logic),
 *   - Use the separator in a SIVIA paving algorithm to classify the area,
 *   - Visualize the robot trajectory, detection range, and paving area.
 *
 * Key points for beginners:
 * - SepDynDisk is a local separator acting on (x, y, t).
 * - The detection time is modeled as a degenerated interval [t, t].
 * - SIVIA splits the area into inside (covered) and outside (not covered) boxes.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynDisk.h"

#include <vector>
#include <iostream>

using namespace codac;



/*==================================================================================
 * TRAJECTORY SETUP
 *==================================================================================*/

// Initial and final times of the trajectory
const double t0 = 0.55;
const double tf = 12;

// Time step between trajectory points
const double dt = 0.01;

// Time domain of the trajectory
const Interval tdomain(t0, tf);

// Temporal definition of the trajectory (x(t), y(t), heading(t))
TFunction path("( 5*cos(0.5*t)*sin(t) ; 5*cos(0.5*t)*cos(t) ; atan2(-2.5*sin(0.5*t)*cos(t) - 5*cos(0.5*t)*sin(t) , -2.5*sin(0.5*t)*sin(t) + 5*cos(0.5*t)*cos(t)))");

// Generate TrajectoryVector from TFunction
TrajectoryVector generate_trajectory()
{
    TrajectoryVector traj(tdomain, path, dt);
    traj[2] = traj[2].make_continuous(); // Ensure heading is continuous
    return traj;
}

// Convert TrajectoryVector to TubeVector
TubeVector generate_tube(TrajectoryVector &traj)
{
    TubeVector tube(traj, dt);
    return tube;
}



/*==================================================================================
 * DETECTION PARAMETERS
 *==================================================================================*/

// Time at which detection occurs
double detection_time = 6.4;

// Disk-shaped sensor range (binary perception)
double detection_range = 1.5;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Paving resolution (size of smallest boxes for SIVIA)
double paving_resolution = 0.1;

// Area to be classified (paved)
IntervalVector paving_area = {{-4, 3}, {-7.5, -2}};



int main()
{
    // Generate the robot trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Start VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Instantiate the SepDynDisk separator (binary perception)
    SepDynDisk my_sep(my_tube, detection_range, true);

    // Create 3D IntervalVector for SIVIA ([x], [y], [t])
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time); // degenerated interval for detection time

    // Apply SIVIA to classify the paving area
    // - inside boxes = covered by sensor
    // - outside boxes = not covered by sensor
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display detection disk at detection time
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    fig.draw_circle(px, py, detection_range, "black");

    // Display robot position at detection time
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display the original paving area in black
    fig.draw_box(paving_area, "black");

    // Set figure limits for visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without displaying robot at the end
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}