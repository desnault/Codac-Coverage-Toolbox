/********************************************************************************************
 * @file    example3_three_valued_border_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage contraction using SepDynDisk with uncertain border.
 *
 * @details
 * This example illustrates the use of the SepDynDisk class in a three-valued perception
 * context (covered / uncertain / not covered) using a border around the detection range.
 *
 * Scenario:
 * A robot equipped with a disk-shaped range sensor follows a known 2D trajectory.
 * Each detection has a range and an uncertain border thickness.  
 * An a priori uncertain object position is represented by a 2D box.
 *
 * For each object, we want to separate the parts of the box that are:
 *   - certainly covered (inside set),
 *   - certainly not covered (outside set),
 *   - in the uncertain border (between inside and outside).
 *
 * Goal:
 * Demonstrate how to:
 *   - Instantiate a SepDynDisk separator with a detection range and uncertain border,
 *   - Apply the separator on 3D boxes ([x], [y], [t]),
 *   - Visualize inside (red), outside (blue), and uncertain border (yellow/green) regions.
 *
 * Key points for beginners:
 * - SepDynDisk is a local separator acting on (x, y, t).
 * - Detection time is modeled as a degenerated interval [t, t].
 * - Separator splits the search space into inside, uncertain, and outside regions.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynDisk.h"

#include <vector>
#include <iostream>

using namespace codac;



/*==================================================================================
 * TRAJECTORY SETUP
 *==================================================================================*/

// Initial and final times of the trajectory
const double t0 = 0.55;
const double tf = 12;

// Time step between trajectory points
const double dt = 0.01;

// Time domain of the trajectory
const Interval tdomain(t0, tf);

// Temporal definition of the trajectory (x(t), y(t), heading(t))
TFunction path("( 5*cos(0.5*t)*sin(t) ; 5*cos(0.5*t)*cos(t) ; atan2(-2.5*sin(0.5*t)*cos(t) - 5*cos(0.5*t)*sin(t) , -2.5*sin(0.5*t)*sin(t) + 5*cos(0.5*t)*cos(t)))");

// Generate TrajectoryVector from TFunction
TrajectoryVector generate_trajectory()
{
    TrajectoryVector traj(tdomain, path, dt);
    traj[2] = traj[2].make_continuous(); // Ensure heading is continuous
    return traj;
}

// Convert TrajectoryVector to TubeVector
TubeVector generate_tube(TrajectoryVector &traj)
{
    TubeVector tube(traj, dt);
    return tube;
}



/*==================================================================================
 * DETECTION PARAMETERS
 *==================================================================================*/

// Detection times
std::vector<double> detection_time = {2.5, 6.5, 11.5};

// Detection ranges for each detection
std::vector<double> detection_range = {1, 2, 1.5};

// Border thickness around detection range
std::vector<double> detection_border = {0, 0.5, 1};

// Boxes representing uncertain object positions
std::vector<IntervalVector> boxes_to_contract = {{{0.5,1.5}, {-0.8,0.2}}, {{0,2}, {-6,-5.5}}, {{-6.5,-1}, {2.5,5}}};



int main()
{
    // Generate robot trajectory and TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Start VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over each detection time
    for (int i=0; i<detection_time.size(); i++)
    {
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);

        // Display detection range + uncertain border
        fig.draw_circle(px, py, detection_range[i] + detection_border[i], "black[yellow]");
        // Display certain detection range
        fig.draw_circle(px, py, detection_range[i], "black[green]");

        // Display initial box in black
        fig.draw_box(boxes_to_contract[i],"black");

        // Create SepDynDisk separator for three-valued perception
        SepDynDisk my_sep(my_tube, detection_range[i], detection_border[i], true);

        // Convert detection time to degenerated interval
        Interval detection_time_interval(detection_time[i]);

        // Prepare 3D IntervalVectors for inside and outside contractions
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;

        // Apply separator
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display contracted results
        fig.draw_box(contracted_inside_box.subvector(0,1), "red");   // inside
        fig.draw_box(contracted_outside_box.subvector(0,1), "blue"); // outside

        // Display robot position at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7,6,-8.5,5);

    // Show figure without displaying robot at the end
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}