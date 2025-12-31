/********************************************************************************************
 * @file    example4_three_valued_uncertain_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage paving using the SepDynPolar separator.
 *
 * @details
 * This example illustrates the use of the SepDynPolar class in a three-valued
 * perception context (certainly detected / uncertain / certainly not detected)
 * using uncertain detection ranges and FOVs in combination with a paving algorithm.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * A detection occurs at a specific time, associated with:
 *   - a nominal detection range,
 *   - an uncertain detection range (thicker border),
 *   - a nominal FOV, and
 *   - an uncertain FOV.
 *
 * The paving algorithm (SIVIA) is used to classify a given area into:
 *   - certainly detected (inside contractor),
 *   - certainly not detected (outside contractor),
 *   - uncertain (neither inside nor outside).
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate SepDynPolar with both nominal and uncertain range/FOV intervals,
 *   - perform paving (SIVIA) over a 2D area at a given detection time,
 *   - visualize the robot trajectory, detection area, and classified paving area.
 *
 * Key points for beginners:
 * - SepDynPolar supports three-valued logic via uncertain intervals.
 * - The detection time is modeled as a degenerated interval [t, t].
 * - Visualization shows nominal detection (green), uncertain border (yellow), and robot trajectory.
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

// Detection time (single detection instant)
double detection_time = 6.4;

// Nominal detection range
Interval detection_range(1,2);

// Uncertain detection border (thicker range)
Interval detection_border(0.5,2.5);

// Nominal FOV
Interval fov = Interval(0).inflate(M_PI/4.);

// Uncertain FOV
Interval uncertain_fov = Interval(0).inflate(1.2*M_PI/4.);



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of the paving algorithm (SIVIA)
double paving_resolution = 0.1;

// Area to pave (2D)
IntervalVector paving_area = {{-4,3}, {-7.5,-2}};


int main()
{
    // Generate the trajectory and the corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Create a SepDynPolar separator with nominal and uncertain range/FOV
    SepDynPolar my_sep(my_tube, detection_range, detection_border, fov, uncertain_fov, true);

    // Create a 3D IntervalVector [x, y, t] for the paving algorithm
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time); // Degenerated interval for detection time

    // Apply SIVIA paving algorithm
    // Classifies paving area as: inside (detected), outside (not detected), uncertain
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and TubeVector
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display nominal and uncertain detection areas
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    double heading = my_traj[2](detection_time);

    fig.draw_pie(px, py, detection_range, heading+fov, "black");          // nominal detection
    fig.draw_pie(px, py, detection_border, heading+uncertain_fov, "black"); // uncertain border

    // Display robot at detection time
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display initial paving area
    fig.draw_box(paving_area, "black");

    // Set figure axis limits
    fig.axis_limits(-7,6,-8.5,5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}