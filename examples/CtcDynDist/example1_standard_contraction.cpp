/********************************************************************************************
 * @file    example1_standard_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * @brief   Demonstrates the standard use of the CtcDynDist contractor to refine the 
 *          estimated positions of multiple 2D objects detected along a known robot trajectory.
 *
 * @details
 * This example illustrates how to use the CtcDynDist contractor from the 
 * codac-coverage-toolbox in its **standard configuration**, i.e., with a degenerate 
 * (thin) detection time interval [t].
 *
 * Scenario:
 * - A robot moves along a known trajectory in 2D space.
 * - Three objects (obj1, obj2, obj3) are detected at times t1, t2, t3.
 * - Each detection has a measured distance interval [d] to account for sensor uncertainty.
 * - Each object has a prior estimated position represented by a 2D interval vector (box).
 *
 * Goal:
 * Apply CtcDynDist to propagate the Euclidean distance constraint between the robot 
 * and each object and **contract the boxes** to improve the position estimation.
 *
 * Key points for beginners:
 * - Detection time intervals are **degenerate** (thin) in this example.
 * - The trajectory of the robot is assumed perfectly known.
 * - Boxes are contracted individually for each detection.
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

// Times at which objects are detected
std::vector<double> detection_time = {2.5, 6.5, 11.5};

// Measured distances (intervals) for each detection
std::vector<Interval> detection_range = {{0,1}, {2,3}, {1,1.5}};

// Prior boxes representing uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{1,2}, {-0.8,0.2}}, {{0,1}, {-6,-5}}, {{-6,-1}, {2.5,4}}};



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

    // For each detection time, 
    for (int i=0; i<detection_time.size(); i++)
    {
        // Convert detection time to a degenerate interval [t] for CtcDynDist
        Interval t_interval(detection_time[i]);

        // Copy the prior box to preserve original for display
        IntervalVector contracted_box = boxes_to_contract[i];

        // Create a contractor for this detection
        CtcDynDist my_ctc(my_tube, detection_range[i], true);

        // Apply contraction: reduces box size based on Euclidean distance constraint
        my_ctc.contract(contracted_box, t_interval);

        // Visualization:
        fig.draw_circle(my_traj[0](detection_time[i]),
                        my_traj[1](detection_time[i]),
                        detection_range[i].ub(), "black[gray]"); // max distance
        fig.draw_circle(my_traj[0](detection_time[i]),
                        my_traj[1](detection_time[i]),
                        detection_range[i].lb(), "black[white]"); // min distance

        fig.draw_box(boxes_to_contract[i], "black"); // original box
        fig.draw_box(contracted_box, "red");         // contracted box
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5); // robot at detection time
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}
