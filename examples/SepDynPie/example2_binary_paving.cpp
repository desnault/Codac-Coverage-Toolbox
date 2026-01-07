/********************************************************************************************
 * @file    example2_binary_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage paving using the SepDynPie separator.
 *
 * @details
 * This example illustrates how to compute the spatial area covered by a robot
 * equipped with an oriented range sensor at a given detection time.
 *
 * Contrary to example 1 (binary contraction on a single box), this example uses
 * a paving algorithm (SIVIA) to classify an entire 2D area into:
 *   - certainly detected regions (inside set),
 *   - certainly not detected regions (outside set),
 *   - uncertain regions (boundary between inside and outside).
 *
 * The detection model is a "pie":
 *   - distance ∈ [0, r_max]
 *   - angle ∈ FOV (defined in the robot frame)
 *
 * The detection time is modeled as a degenerated interval [t, t].
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynPie separator,
 *   - embed it in a SIVIA paving algorithm,
 *   - compute and visualize the covered area at a given time.
 *
 * Key points for beginners:
 * - SepDynPie acts on a 3D space (x, y, t).
 * - SIVIA explores a 2D spatial area while time is fixed.
 * - The FOV is expressed in the robot frame and moves with the robot heading.
 * - The paving resolution controls the spatial accuracy of the result.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators, SIVIA)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynPie.h"

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

// Detection time (single instant, modeled later as a degenerated interval)
double detection_time = 6.2;

// Maximum detection range
double detection_range = 2.0;

// Detection field of view (FOV)
//
// IMPORTANT:
// - This interval is expressed in the robot frame.
// - 0 rad corresponds to the robot heading direction.
// - The effective angular sector is:
//     heading + detection_fov
Interval detection_fov(-M_PI / 4., M_PI / 4.);



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Spatial resolution of the paving (smaller = finer result, higher computation cost)
double paving_resolution = 0.1;

// 2D area to classify (x, y)
IntervalVector paving_area = {{-4,3}, {-7.5,-2}};



int main()
{
    // Generate the robot trajectory and associated tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Create the SepDynPie separator
    // This separator models a detection pie:
    //   - distance ∈ [0, detection_range]
    //   - angle ∈ detection_fov (robot frame)
    SepDynPie my_sep(my_tube, detection_range, detection_fov, true);

    // Initial box for SIVIA: [x, y, t]
    // - x, y : paving area
    // - t    : degenerated detection time
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time);

    // Run the paving algorithm
    // SIVIA recursively bisects X0 and uses the separator to classify
    // each box as inside, outside, or undetermined.
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the detection pie at the considered time
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    double heading = my_traj[2](detection_time);
    fig.draw_pie(px, py, Interval(0,detection_range), heading+detection_fov, "black");

    // Display robot pose
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display paving area boundary
    fig.draw_box(paving_area, "black");

    // Set figure limits for clarity
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show result (without final robot pose)
    fig.show(0);
    vibes::endDrawing();

	return EXIT_SUCCESS;
}