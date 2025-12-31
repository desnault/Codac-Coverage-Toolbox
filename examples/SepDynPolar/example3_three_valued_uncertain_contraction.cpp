/********************************************************************************************
 * @file    example3_three_valued_uncertain_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage contraction using the SepDynPolar separator.
 *
 * @details
 * This example illustrates the use of the SepDynPolar class in a three-valued
 * perception context (certainly detected / uncertain / certainly not detected)
 * using uncertain detection ranges and field-of-view (FOV) intervals.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * Several detections occur at specific times, each associated with:
 *   - a nominal detection range,
 *   - an uncertain detection range (thicker border),
 *   - a nominal FOV, and
 *   - an uncertain FOV.
 *
 * An a priori uncertain object position is represented by a 2D box. Using
 * SepDynPolar, the box is split into:
 *   - certainly inside the detection area (inside contractor),
 *   - certainly outside (outside contractor),
 *   - uncertain region (neither inside nor outside).
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate SepDynPolar with both nominal and uncertain range/FOV intervals,
 *   - apply the separator on a 3D box ([x, y, t]),
 *   - visualize the inside, outside, and uncertain regions.
 *
 * Key points for beginners:
 * - SepDynPolar supports both binary and three-valued logic via uncertain intervals.
 * - The detection time is modeled as a degenerated interval [t, t].
 * - Inside = red, outside = blue, uncertain = default (not drawn).
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

// Detection times
std::vector<double> detection_time = {2.3, 6.5, 11.5};

// Nominal detection ranges (intervals)
std::vector<Interval> detection_range = {{0.5,1.5}, {1.5,2}, {1,2}};

// Uncertain detection ranges (thicker border)
std::vector<Interval> detection_range_border = {{0.3,1.8}, {1,2.3}, {1,2.3}};

// Nominal FOV intervals
std::vector<Interval> fov = {{-M_PI/4., M_PI/4.}, {M_PI/4., 3.*M_PI/4.}, {-0.8*M_PI/2., 0.8*M_PI/2.}};

// Uncertain FOV intervals
std::vector<Interval> uncertain_fov = {{-M_PI/3.5, M_PI/3.5}, {M_PI/4., 3.*M_PI/4.}, {-M_PI/2., M_PI/2.}};

// Boxes to contract (initial uncertain object positions)
std::vector<IntervalVector> boxes_to_contract = {
    {{-1,0.5}, {-2.,-1.2}},
    {{-1.5,-1}, {-6.6,-5.2}},
    {{-6.5,-1}, {2.5,5}}
};



int main()
{
    // Generate the trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Display robot trajectory and TubeVector
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over each detection time
    for (int i=0; i<detection_time.size(); i++)
    {
        // Get robot position and heading at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Draw uncertain detection area (yellow) and nominal detection area (green)
        fig.draw_pie(px, py, detection_range_border[i], heading+uncertain_fov[i], "black[yellow]");
        fig.draw_pie(px, py, detection_range[i], heading+fov[i], "black[green]");

        // Draw initial object box
        fig.draw_box(boxes_to_contract[i],"black");

        // Create a SepDynPolar separator with both nominal and uncertain range/FOV intervals
        SepDynPolar my_sep(my_tube, detection_range[i], detection_range_border[i], fov[i], uncertain_fov[i], false);

        // Convert detection time to a degenerated interval
        Interval detection_time_interval(detection_time[i]);

        // Initialize 3D boxes for contraction ([x, y, t])
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;

        // Apply separator: contract inside/outside boxes
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display contracted boxes
        fig.draw_box(contracted_inside_box.subvector(0,1),"red");   // certainly inside
        fig.draw_box(contracted_outside_box.subvector(0,1),"blue"); // certainly outside

        // Display robot position at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7,6,-8.5,5);

    // Show figure without displaying robot at the end
    fig.show(0);

    // End drawing
    vibes::endDrawing();


	return EXIT_SUCCESS;
}