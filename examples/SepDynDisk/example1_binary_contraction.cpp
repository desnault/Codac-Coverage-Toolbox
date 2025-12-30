/********************************************************************************************
 * @file    example1_binary_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage contraction using the SepDynDisk separator.
 *
 * @details
 * This example illustrates the use of the SepDynDisk class in a binary perception
 * context (covered / not covered) using a degenerated detection time.
 *
 * Scenario:
 * A robot equipped with a disk-shaped range sensor follows a known 2D trajectory.
 * Several static objects are located in the environment, each represented by a 2D
 * box of uncertain position.
 *
 * Using the SepDynDisk separator, we separate each box into:
 *   - Inside the perception set (certainly covered by the sensor),
 *   - Outside the perception set (certainly not covered by the sensor).
 *
 * Goal:
 * Demonstrate how to:
 *   - Instantiate a SepDynDisk separator with a disk range (binary logic),
 *   - Apply the separator on a 3D box ([x], [y], [t]),
 *   - Visualize the inside and outside contracted sets.
 *
 * Key points for beginners:
 * - SepDynDisk is a local separator acting on (x, y, t).
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

#include "SepDynDisk.h"

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

// Detection times (instants at which the robot detects objects)
std::vector<double> detection_time = {2.5, 6.5, 11.5};

// Detection ranges for the binary SepDynDisk sensor (disk radius)
std::vector<double> detection_range = {1, 2, 2.5};

// Prior boxes representing uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{0.5,1.5}, {-0.8,0.2}}, {{0,1.5}, {-6,-5.5}}, {{-6.5,-1}, {2.5,4}}};



int main()
{
    // Generate robot trajectory and corresponding TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over each detection time
    for (int i=0; i<detection_time.size(); i++)
    {
        // Compute robot position at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);

        // Display the detection disk (max range)
        fig.draw_circle(px, py, detection_range[i], "black[gray]");

        // Display prior object box
        fig.draw_box(boxes_to_contract[i], "black");

        // Instantiate SepDynDisk separator with binary perception logic
        SepDynDisk my_sep(my_tube, detection_range[i], true);

        // Convert detection time to degenerated interval
        Interval detection_time_interval(detection_time[i]);

        // Create 3D box [x, y, t] for "inside" contraction
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        // Create 3D box [x, y, t] for "outside" contraction
        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;

        // Apply the separator
        // - contracted_inside_box will contain points consistent with detection
        // - contracted_outside_box will contain points inconsistent with detection
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display the contracted boxes
        fig.draw_box(contracted_inside_box.subvector(0,1), "red");   // inside = covered
        fig.draw_box(contracted_outside_box.subvector(0,1), "blue"); // outside = not covered

        // Display robot pose at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Display figure without showing final robot pose
    fig.show(0);

    // End VIBes drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}