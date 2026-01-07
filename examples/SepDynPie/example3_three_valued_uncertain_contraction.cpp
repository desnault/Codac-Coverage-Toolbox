/********************************************************************************************
 * @file    example3_three_valued_uncertain_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Three-valued (inside / outside / uncertain) contraction using the SepDynPie separator.
 *
 * @details
 * This example demonstrates the use of the SepDynPie class in a three-valued
 * perception context, where sensor measurements are affected by uncertainty.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * At several detection times, a detection is reported, but both the range and the
 * field of view (FOV) are uncertain.
 *
 * Each detection is modeled by:
 *   - a nominal detection pie (certain detection),
 *   - a larger uncertain detection pie (possible detection).
 *
 * An object position is represented by an a priori uncertain 2D box.
 * Using the SepDynPie separator, this box is split into:
 *   - an inside part: certainly detected,
 *   - an outside part: certainly not detected,
 *   - an implicit uncertain part: neither certainly inside nor certainly outside.
 *
 * Goal:
 * Demonstrate how to:
 *   - model three-valued perception using SepDynPie,
 *   - represent uncertainty on both distance and angular measurements,
 *   - contract a spatial box using interval-based separation.
 *
 * Key points for beginners:
 * - SepDynPie acts on a 3D space (x, y, t).
 * - The detection time is modeled as a degenerated interval [t, t].
 * - The FOV is expressed in the robot frame and moves with the robot heading.
 * - Uncertainty is handled by enlarging both the distance and angular domains.
 *
 * Dependencies:
 * - CODAC (intervals, trajectories, contractors, separators)
 * - IBEX (interval arithmetic)
 * - Eigen3 (linear algebra backend)
 * - VIBes (visualization)
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynPie.h"

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

// Detection times (exact instants)
std::vector<double> detection_time = {2.5, 6.5, 11.5};

// Nominal detection ranges
std::vector<double> detection_range = {1.0, 2.0, 1.5};

// Uncertainty margins on the detection range
// The effective uncertain range is:
//   [0, detection_range + detection_range_border]
std::vector<double> detection_range_border = {0.0, 0.5, 0.2};

// Nominal detection FOVs (robot frame)
// These define the angles for which detection is certain.
std::vector<Interval> fov = {{-M_PI/4., M_PI/4.}, {M_PI/4., 3.*M_PI/4.}, {-0.8*M_PI/2., 0.8*M_PI/2.}};

// Uncertain detection FOVs (robot frame)
// These define the angular domain where detection is possible but uncertain.
std::vector<Interval> uncertain_fov = {{-M_PI/2., M_PI/2.}, {M_PI/4., 3.*M_PI/4.}, {-M_PI/2., M_PI/2.}};

// Prior uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{0.,1.5}, {-0.8,0.2}}, {{-2,-1}, {-8,-6}}, {{-6.5,-1}, {2.5,5}}};



int main()
{
    // Generate the robot trajectory and associated tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over all detections
    for (int i=0; i<detection_time.size(); i++)
    {
        // Robot pose at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Display uncertain detection area (possible detection)
        fig.draw_pie(px, py, Interval(0, detection_range[i] + detection_range_border[i]), heading+uncertain_fov[i], "black[yellow]");

        // Display nominal detection area (certain detection)
        fig.draw_pie(px, py, Interval(0, detection_range[i]), heading+fov[i], "black[green]");

        // Display initial object uncertainty box
        fig.draw_box(boxes_to_contract[i],"black");

        // Create the SepDynPie separator
        // This separator encodes a three-valued detection model:
        //   - inside  : certainly detected,
        //   - outside : certainly not detected,
        //   - unknown : uncertain (between both).
        SepDynPie my_sep(my_tube, detection_range[i], detection_range_border[i], fov[i], uncertain_fov[i], true);

        // Detection time as a degenerated interval
        Interval detection_time_interval(detection_time[i]);
        
        // Inside contraction box [x, y, t]
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        // Outside contraction box [x, y, t]
        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;
        
        // Apply the separator
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display contracted sets
        fig.draw_box(contracted_inside_box.subvector(0,1),"red");
        fig.draw_box(contracted_outside_box.subvector(0,1),"blue");
        
        // Display robot pose
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    /// Set visualization limits
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show result (without final robot pose)
    fig.show(0);
    vibes::endDrawing();

	return EXIT_SUCCESS;
}