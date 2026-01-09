/********************************************************************************************
 * @file    example2_binary_paving.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage perception using SepDynDist and a paving algorithm.
 *
 * @details
 * This example illustrates how to use the SepDynDist separator in a binary perception
 * context (covered / not covered) together with a paving algorithm.
 *
 * Scenario:
 * A robot equipped with a range sensor follows a known 2D trajectory. At a given
 * detection time, the robot measures a distance to a static object, modeled as
 * an interval. The exact object position is unknown.
 *
 * Instead of contracting a single prior box, a 2D search area is explored using
 * a paving algorithm. Each spatial region is classified as:
 *   - inside  (certainly detected),
 *   - outside (certainly not detected),
 *   - undetermined.
 *
 * The detection time is assumed to be perfectly known and is modeled as a
 * degenerated interval.
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynDist separator with a distance interval (binary logic),
 *   - embed the separator into a paving algorithm,
 *   - visualize the perceived area for a fixed detection time.
 *
 * Key points for beginners:
 * - SepDynDist acts on 3D boxes ([x], [y], [t]).
 * - The paving is performed in space, while time is fixed.
 * - Paving algorithm repeatedly applies the separator to classify the space.
 * - This approach is useful to visualize sensor perception regions.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, separators, paving algorithms)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynDist.h"

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

// Detection time (exact instant, modeled later as a degenerated interval)
double detection_time = 6.4;

// Detection range (distance interval)
Interval detection_range(1, 2);



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Spatial resolution of the paving algorithm
double paving_resolution = 0.1;

// 2D area to be classified by paving algorithm ([x], [y])
IntervalVector paving_area = {{-4, 3}, {-7.5, -2}};



int main()
{
    // Generate the robot trajectory and the corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Create the separator for the given trajectory and detection range
    // Binary logic: inside = detected, outside = not detected
    SepDynDist my_sep(my_tube, detection_range, true);

    // Build the initial 3D box [x, y, t] for paving
    // The time dimension is fixed to the detection time
    IntervalVector X0(3);
    X0[0] = paving_area[0];
    X0[1] = paving_area[1];
    X0[2] = Interval(detection_time); 

    // Run the paving algorithm
    // Each spatial box is classified using the SepDynDist separator
    SIVIA(X0, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the detection annulus at the considered time
    double px = my_traj[0](detection_time);
    double py = my_traj[1](detection_time);
    fig.draw_circle(px, py, detection_range.ub(), "black");
    fig.draw_circle(px, py, detection_range.lb(), "black");

    // Display robot pose at detection time
    fig.draw_vehicle(detection_time, &my_traj, 0.5);

    // Display the paving area
    fig.draw_box(paving_area, "black");

    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

    return EXIT_SUCCESS;
}