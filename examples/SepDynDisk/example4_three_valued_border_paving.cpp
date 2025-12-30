/********************************************************************************************
 * @file    example4_three_valued_border_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage paving using SepDynDisk with uncertain border.
 *
 * @details
 * This example illustrates the use of the SepDynDisk class in a three-valued perception
 * context (covered / uncertain / not covered) using a border around the detection range.
 *
 * Scenario:
 * A robot equipped with a disk-shaped range sensor follows a known 2D trajectory.
 * The paving algorithm (SIVIA) classifies a 2D area as inside, outside, or uncertain
 * with respect to the detection at a specific time.
 *
 * Goal:
 * Demonstrate how to:
 *   - Instantiate a SepDynDisk separator with a detection range and uncertain border,
 *   - Apply the separator with a SIVIA paving algorithm,
 *   - Visualize inside (red), outside (blue), and uncertain border (yellow/green) regions.
 *
 * Key points for beginners:
 * - SepDynDisk is a local separator acting on (x, y, t).
 * - Detection time is modeled as a degenerated interval [t, t].
 * - SIVIA recursively subdivides the paving area to classify each sub-box.
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

// Detection time
double detection_time = 6.4;

// Detection range
double detection_range = 1.5;

// Detection uncertain border
double detection_border = 0.5;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Paving resolution (minimum box size for SIVIA)
double paving_resolution = 0.1;

// Paving area (2D area to classify)
IntervalVector paving_area = {{-4,3}, {-7.5,-2}};



int main()
{
    // Generate robot trajectory and TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Start VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Create SepDynDisk separator for three-valued perception
    SepDynDisk my_sep(my_tube, detection_range, detection_border, true);

    // Create an IntervalVector that contains the 2D paving area + detection time
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time);

    // Apply SIVIA paving algorithm
    // Classify the paving area as inside, outside, or uncertain at the detection time
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display detection range and uncertain border
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    fig.draw_circle(px, py, detection_range, "black");              // certain detection
    fig.draw_circle(px, py, detection_range + detection_border, "black"); // uncertain border

    // Display robot position at detection time
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display initial paving area
    fig.draw_box(paving_area, "black");

    // Set figure axis limits
    fig.axis_limits(-7,6,-8.5,5);

    // Show figure without displaying robot at the end
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}