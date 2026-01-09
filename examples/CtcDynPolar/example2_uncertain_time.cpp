/********************************************************************************************
 * @file    example2_uncertain_time.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * @brief   Demonstrates the use of the CtcDynPolar contractor with uncertain detection time.
 *
 * @details
 * This example illustrates how to use the CtcDynPolar contractor from the 
 * codac-coverage-toolbox when the detection time interval [t] is **not degenerate**, i.e.,
 * when the exact detection time is uncertain.
 *
 * Scenario:
 * - A robot moves along a known trajectory in 2D space, with a heading angle.
 * - A single detection occurs over an interval of time, modeling uncertain measurement timing.
 * - The detection is associated with:
 *   - a range interval [range], modeling sensor distance uncertainty
 *   - an angle interval [delta], modeling the Field of View (FOV) of the oriented sensor
 * - A prior box represents the uncertain object position.
 *
 * Goal:
 * Apply CtcDynPolar to propagate the polar constraint between the robot and the object 
 * and **contract the box** to refine the position estimation considering both distance, 
 * orientation, and detection-time uncertainty.
 *
 * Key points for beginners:
 * - The detection time interval is **non-degenerate** in this example.
 * - The robot trajectory is assumed perfectly known.
 * - The box is contracted once for the entire time interval.
 * - Visualization includes pie sectors to represent FOV and the contracted box.
 *
 * Dependencies:
 * - CODAC (for contractors and tubes)
 * - IBEX (interval computations)
 * - Eigen3 (linear algebra)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "CtcDynPolar.h"

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

// Times at which the detection may occur (uncertain detection time)
std::vector<double> detection_time = {5.9, 6.1, 6.3, 6.5, 6.7};

// Detection range interval
Interval detection_range(1, 2);

// Detection angle interval (Field of View relative to robot heading)
Interval detection_angle(M_PI/4., 3*M_PI/4.);

// Prior box representing uncertain object position ([x], [y])
IntervalVector box_to_contract({{-5, 5}, {-8, -4}});



int main()
{
    // Generate the robot trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over each detection time for visualization
    for (int i=0; i<detection_time.size(); i++)
    {
        // Extract robot position and heading at the detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double theta = my_traj[2](detection_time[i]);
        
        // Display the Field of View as a pie sector at each considered time
        fig.draw_pie(px, py, detection_range, theta+detection_angle, "black[gray]");

        // Display robot position at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Display the initial state of the prior box
    fig.draw_box(box_to_contract, "black");

    // Create the contractor for the trajectory, detection range, and angle
    CtcDynPolar my_ctc(my_tube, detection_range, detection_angle, true);

    // Copy the prior box to preserve its initial state for display
    IntervalVector contracted_box = box_to_contract;

    // Create a single interval covering all detection times (uncertain detection time)
    Interval detection_time_interval(detection_time[0], detection_time.back());

    // Apply contraction: reduces box size based on polar constraint over the time interval
    my_ctc.contract(contracted_box, detection_time_interval);

    // Display the contracted box if it is not empty
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