/********************************************************************************************
 * @file    example1_binary_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage contraction using the SepDynPolar separator.
 *
 * @details
 * This example illustrates the use of the SepDynPolar class in a binary perception
 * context (covered / not covered) using a degenerated detection time.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * Several static objects are detected at known instants along the trajectory.
 * For each detection, both the measured distance and field of view (FOV) are uncertain
 * and modeled as intervals.
 *
 * For each object, an a priori uncertain position is represented by a 2D box.
 * Using the SepDynPolar separator, we separate the parts of the box that are:
 *   - certainly consistent with the detection (inside set),
 *   - certainly inconsistent with the detection (outside set).
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynPolar separator with distance and angular intervals,
 *   - apply the separator on a 3D box ([x], [y], [t]),
 *   - visualize the inside and outside contracted sets.
 *
 * Key points for beginners:
 * - SepDynPolar is a local separator acting on (x, y, t) for oriented range sensors.
 * - The detection time is modeled as a degenerated interval [t, t].
 * - The separator splits the search space into "inside" (red) and "outside" (blue).
 * - No paving algorithm is used here, only direct contraction.
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

// Times at which objects are detected (exact instants, modeled later as degenerated intervals)
std::vector<double> detection_time = {2.2, 6.2, 11.5};

// Measured distances (intervals) for each detection
std::vector<Interval> detection_range = {{0.5,1.}, {1.,2.}, {0.5,1.5}};

// Field of view (FOV) intervals for each detection
std::vector<Interval> detection_fov = {{-M_PI/4,M_PI/4}, {M_PI/4,3*M_PI/4}, {-M_PI/2,M_PI/2}};

// Prior boxes representing uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{0.2,1.2}, {-1.6,-1.2}}, {{0,1}, {-6.5,-5.5}}, {{-6.5,-1}, {1.5,5}}};



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

    // Loop over all detection times
    for (int i=0; i<detection_time.size(); i++)
    {
        // Get robot position and heading at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Display the detection area (pie) at the considered time
        fig.draw_pie(px, py, detection_range[i], heading+detection_fov[i],"black[gray]");

        // Display the initial state of the box
        fig.draw_box(boxes_to_contract[i],"black");

        // Create the SepDynPolar separator for the given trajectory, distance interval, and FOV interval
        SepDynPolar my_sep(my_tube, detection_range[i], detection_fov[i], true);

        // Detection time as a degenerated interval
        Interval detection_time_interval(detection_time[i]);

        // Create IntervalVector for inside contraction ([x, y, t])
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        // Create IntervalVector for outside contraction ([x, y, t])
        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;

        // Apply the separator
        // Inside box = points consistent with detection (red)
        // Outside box = points inconsistent with detection (blue)
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display the contracted boxes
        fig.draw_box(contracted_inside_box.subvector(0,1),"red");
        fig.draw_box(contracted_outside_box.subvector(0,1),"blue");

        // Display robot pose at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7,6,-8.5,5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}