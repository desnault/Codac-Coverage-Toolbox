/********************************************************************************************
 * @file    example2_binary_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage paving using the SepDynPolar separator.
 *
 * @details
 * This example illustrates the use of the SepDynPolar class in a binary perception
 * context (covered / not covered) using a SIVIA-based paving algorithm.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * For a given detection time, the detection area is defined by a distance interval
 * and a field of view (FOV) interval.
 *
 * An uncertain 2D area (paving_area) is subdivided using a paving algorithm (SIVIA),
 * which classifies each sub-box as:
 *   - certainly detected (inside, red)
 *   - certainly not detected (outside, blue)
 *   - undetermined (yellow)
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynPolar separator with distance and FOV intervals,
 *   - apply the separator with a SIVIA paving algorithm,
 *   - visualize the detected area over a paving region.
 *
 * Key points for beginners:
 * - SepDynPolar is a local separator acting on (x, y, t) for oriented range sensors.
 * - The detection time is modeled as a degenerated interval [t, t].
 * - The SIVIA algorithm recursively subdivides the area until resolution is reached.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynPolar.h"

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

// Time domain of the trajectory
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

// Detection time (exact instant of detection)
double detection_time = 6.2;

// Detection range (interval)
Interval detection_range(0.5, 1.5);

// Detection field of view (interval)
Interval detection_fov(-M_PI/4., M_PI/4.);



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of the paving algorithm (size of smallest box)
double paving_resolution = 0.1;

// Area to be paved (2D space)
IntervalVector paving_area = {{-4, 3}, {-7.5, -2}};



int main()
{
    // Generate the robot trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Create the SepDynPolar separator for the given trajectory, detection range, and FOV
    SepDynPolar my_sep(my_tube, detection_range, detection_fov, true);

    // Create an IntervalVector representing the 3D area to be paved ([x, y, t])
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time); // Degenerated interval for detection time

    // Apply the SIVIA paving algorithm
    // Subdivide the area and classify each sub-box as inside, outside, or undetermined
    SIVIA(X0, my_sep, paving_resolution);

    // Display the robot trajectory and corresponding tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the detection area at the detection time
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    double heading = my_traj[2](detection_time);
    fig.draw_pie(px, py, detection_range, heading+detection_fov, "black");

    // Display the robot position at detection time
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display the initial paving area
    fig.draw_box(paving_area, "black");

    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show the figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}