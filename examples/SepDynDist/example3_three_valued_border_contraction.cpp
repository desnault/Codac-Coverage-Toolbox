/********************************************************************************************
 * @file    example3_three_valued_contraction_border.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued coverage contraction using SepDynDist with a border width.
 *
 * @details
 * This example illustrates the use of the SepDynDist separator in a three-valued
 * perception context using a border width around the detection range.
 *
 * Scenario:
 * A robot equipped with a range sensor follows a known 2D trajectory. Several static
 * objects are detected at known instants along the trajectory. For each detection,
 * the measured distance is modeled as an interval, together with a border width
 * representing an uncertainty zone around this range.
 *
 * This uncertainty induces a three-valued perception logic:
 *   - certainly detected        (inside),
 *   - certainly not detected    (outside),
 *   - undetermined (boundary region).
 *
 * For each detection, an a priori uncertain object position is represented by a 2D box.
 * The SepDynDist separator is used to separate the parts of the box that are:
 *   - certainly consistent with the detection,
 *   - certainly inconsistent with the detection.
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynDist separator with a distance interval and a border width,
 *   - model three-valued coverage logic,
 *   - apply the separator on a 3D box ([x], [y], [t]),
 *   - visualize the inside and outside contracted sets.
 *
 * Key points for beginners:
 * - SepDynDist is a local separator acting on (x, y, t).
 * - A border width introduces an uncertainty zone around the detection range.
 * - This uncertainty naturally leads to three-valued logic.
 * - The detection time is modeled as a degenerated interval.
 * - No paving algorithm is used here, only direct contraction.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, separators)
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

// Detection times (exact instants, modeled later as degenerated intervals)
std::vector<double> detection_time = {2.5, 6.5, 11.5};

// Detection ranges (distance intervals for each detection)
std::vector<Interval> detection_range = {{0,1}, {1,2}, {1.5,2}};

// Border widths associated with each detection range
// This border models an uncertainty zone and induces three-valued logic
std::vector<double> detection_border = {0.5, 0.3, 0.3};

// Prior boxes representing uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{0.5,1.5}, {-0.8,1}}, {{0,1.5}, {-6,-5.5}}, {{-3.5,-2}, {1.5,2}}};



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

    // Process each detection independently
    for (int i=0; i<detection_time.size(); i++)
    {
        // Robot position at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);

        // Display detection geometry:
        // - green: certainly detected region
        // - yellow: uncertainty (border) region
        // - white: inner non-detected region (if defined)
        fig.draw_circle(px, py, detection_range[i].ub() + detection_border[i], "black[yellow]");
        fig.draw_circle(px, py, detection_range[i].ub(), "black[green]");
        fig.draw_circle(px, py, detection_range[i].lb(), "black[yellow]");
        if (detection_range[i].lb() - detection_border[i] > 0)
        {
            
            fig.draw_circle(px, py, detection_range[i].lb() - detection_border[i], "black[white]");
        }

        // Display the prior object position box
        fig.draw_box(boxes_to_contract[i],"black");

        // Create the separator with three-valued logic using a border width
        SepDynDist my_sep(my_tube, detection_range[i], detection_border[i], true);

        // Detection time modeled as a degenerated interval
        Interval detection_time_interval(detection_time[i]);
        
        // Build 3D box [x, y, t] for inside contraction
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;
        
        // Build 3D box [x, y, t] for outside contraction
        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;
        
        // Apply the separator:
        // - inside  -> certainly detected
        // - outside -> certainly not detected
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display results
        fig.draw_box(contracted_inside_box.subvector(0,1),"red");
        fig.draw_box(contracted_outside_box.subvector(0,1),"blue");

        // Display robot pose at detection time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }
    
    // Set axis limits for better visualization
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without displaying the robot at the end
    fig.show(0);

    // Close drawing
    vibes::endDrawing();

    return EXIT_SUCCESS;
}