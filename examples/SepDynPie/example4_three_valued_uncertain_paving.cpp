/********************************************************************************************
 * @file    example4_three_valued_uncertain_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage paving using the SepDynPie separator.
 *
 * @details
 * This example illustrates how to compute a global coverage map using the
 * SepDynPie separator in a three-valued perception context, using a paving
 * algorithm (SIVIA).
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * At a given detection time, the robot performs a detection with:
 *   - a nominal detection model (certain detection),
 *   - an enlarged uncertain detection model (possible detection).
 *
 * The goal is to classify a spatial area into:
 *   - certainly detected zones,
 *   - certainly not detected zones,
 *   - uncertain zones.
 *
 * This is achieved by applying the SepDynPie separator within a SIVIA paving
 * algorithm, which recursively bisects space until a desired spatial resolution
 * is reached.
 *
 * Goal:
 * Demonstrate how to:
 *   - use SepDynPie with three-valued logic in a paving context,
 *   - compute a guaranteed coverage map under sensor uncertainty,
 *   - visualize certain and uncertain detected areas.
 *
 * Key points for beginners:
 * - SepDynPie is a local separator acting on (x, y, t).
 * - SIVIA handles the global space exploration and classification.
 * - The detection time is modeled as a degenerated interval [t, t].
 * - Uncertainty is encoded via enlarged distance and angular domains.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators)
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

// Detection time (single detection, exact instant)
double detection_time = 6.4;

// Nominal detection range
double detection_range = 1.5;

// Uncertainty margin on the detection range
double detection_border = 0.5;

// Nominal detection FOV (robot frame)
//
// Interval(0).inflate(a) produces [-a, a]
Interval fov = Interval(0).inflate(M_PI / 4.);

// Uncertain detection FOV (robot frame)
Interval uncertain_fov = Interval(0).inflate(1.2 * M_PI / 4.);



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Desired paving spatial resolution
double paving_resolution = 0.1;

// Area to classify ([x], [y])
IntervalVector paving_area = {{-4, 3}, {-7.5, -2}};



int main()
{
    // Generate the robot trajectory and corresponding tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Create the SepDynPie separator with three-valued perception
    // This separator encodes:
    //   - a certain detection pie,
    //   - an uncertain detection pie,
    // allowing classification into inside / outside / unknown.
    SepDynPie my_sep(my_tube, detection_range, detection_border, fov, uncertain_fov, true);

    // Initial box for the paving algorithm: [x, y, t]
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time);

    // Run SIVIA
    // The paving algorithm recursively bisects X0 and uses the separator
    // to classify each box as:
    //   - inside  (certainly detected),
    //   - outside (certainly not detected),
    //   - unknown (uncertain detection).
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display detection areas at detection time
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    double heading = my_traj[2](detection_time);

    // Nominal detection area (certain detection)
    fig.draw_pie(px, py, Interval(0, detection_range), heading+fov, "black");

    // Uncertain detection area (possible detection)
    fig.draw_pie(px, py, Interval(0, detection_range+detection_border), heading+uncertain_fov, "black");

    // Display robot pose and paving area
    fig.draw_vehicle(detection_time, &my_traj, 0.5);
    fig.draw_box(paving_area, "black");

    // Set visualization limits
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show result without final robot pose
    fig.show(0);
    vibes::endDrawing();

	return EXIT_SUCCESS;
}