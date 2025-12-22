/********************************************************************************************
 * @file    example2_uncertain_time.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * @brief   Demonstrates the use of the CtcDynDist contractor with an uncertain detection
 *          time interval to refine the estimated position of a 2D object along a robot trajectory.
 *
 * @details
 * This example illustrates how to use the CtcDynDist contractor from the 
 * codac-coverage-toolbox when the detection time is **uncertain**, i.e., represented
 * by an interval [t] with non-zero width.
 *
 * Scenario:
 * - A robot moves along a known trajectory in 2D space.
 * - A single object is detected at an uncertain time interval [t].
 * - The detection has a measured distance interval [d] (sensor uncertainty).
 * - The object has a prior estimated box representing its uncertain position.
 *
 * Goal:
 * Apply CtcDynDist to propagate the Euclidean distance constraint and contract
 * the object’s box both **spatially** (x, y) and **temporally** (t).
 *
 * Key points for beginners:
 * - The detection time is an interval with width (non-degenerate).
 * - Temporal contraction occurs because the time is uncertain.
 * - The robot trajectory is assumed perfectly known.
 *
 * Dependencies:
 * - CODAC (for contractors and tubes)
 * - IBEX (interval computations)
 * - Eigen3 (linear algebra)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "CtcDynDist.h"

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

// Uncertain detection time of the object represented as an interval [t]
Interval detection_time(5.9, 6.7);

// Measured distance interval for the detection [d]
Interval detection_range(0, 1);

// Prior box representing uncertain object position ([x], [y])
IntervalVector box_to_contract({{-5, 5}, {-6.5, -2}});



int main()
{
    // Generate the robot trajectory and the corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over sampled times for visualization purposes only
    // These times are used to display the detection range and robot positions.
    // They do NOT affect the contraction, which uses the full detection_time interval.
    std::vector<double> display_times = {5.9, 6.1, 6.3, 6.5, 6.7};
    for (int i = 0; i < display_times.size(); i++)
    {
        double px = my_traj[0](display_times[i]);
        double py = my_traj[1](display_times[i]);

        // Draw detection range circles (upper bound and lower bound)
        fig.draw_circle(px, py, detection_range.ub(), "black[gray]"); // max distance
        fig.draw_circle(px, py, detection_range.lb(), "black[white]"); // min distance

        // Draw robot position at this sampled time
        fig.draw_vehicle(display_times[i], &my_traj, 0.5);
    }

    // Display the initial prior box
    fig.draw_box(box_to_contract, "black");

    // Create the contractor for the given trajectory and detection range
    CtcDynDist my_ctc(my_tube, detection_range, true);

    // Copy the box to preserve original for display
    IntervalVector contracted_box = box_to_contract;

    // Apply contraction using the uncertain detection time interval
    // The contractor will refine the box taking into account spatial and temporal constraints.
    my_ctc.contract(contracted_box, detection_time);

    // Draw the contracted box if it is not empty
    if (!contracted_box.is_empty())
    {
        fig.draw_box(contracted_box, "red");
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

    return EXIT_SUCCESS;
}