/********************************************************************************************
 * @file    example1_binary_contraction.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date    2025
 * @brief   Binary coverage contraction using the SepDynPie separator.
 *
 * @details
 * This example illustrates the use of the SepDynPie class in a binary perception
 * context (covered / not covered) using degenerated detection times.
 *
 * Scenario:
 * A robot equipped with an oriented range sensor follows a known 2D trajectory.
 * At several instants along the trajectory, a detection is triggered.
 * Each detection is characterized by:
 *   - a maximum detection range,
 *   - a field of view (FOV) aperture,
 *   - an angular offset of the FOV in the robot frame.
 *
 * The detected objects are static and their positions are initially uncertain,
 * represented by 2D boxes.
 *
 * Using the SepDynPie separator, we separate the parts of each box that are:
 *   - certainly compatible with the detection (inside set),
 *   - certainly incompatible with the detection (outside set).
 *
 * Compared to SepDynPolar:
 * - SepDynPie models a detection "pie":
 *     distance ∈ [0, r_max] and angle ∈ FOV
 * - There is no minimum range (r_min = 0),
 *   which simplifies the definition of the outside area.
 *
 * Goal:
 * Demonstrate how to:
 *   - instantiate a SepDynPie separator,
 *   - define the FOV in the robot frame,
 *   - apply the separator on a 3D box ([x], [y], [t]),
 *   - visualize the inside and outside contracted sets.
 *
 * Key points for beginners:
 * - SepDynPie is a local separator acting on (x, y, t).
 * - Detection times are modeled as degenerated intervals [t, t].
 * - The FOV is defined in the robot frame and moves with the robot heading.
 * - The separator splits space into "inside" (red) and "outside" (blue).
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

// Detection times (exact instants, modeled later as degenerated intervals)
std::vector<double> detection_time = {2.2, 6.2, 11.5};

// Maximum detection ranges (one per detection)
std::vector<double> detection_range = {1.0, 2.0, 2.5};

// Field of view apertures (in radians)
// Each FOV is symmetric around its offset direction
std::vector<double> detection_fov = {M_PI/2., 3*M_PI/4. , 2.*M_PI};


// Angular offsets of the FOV in the robot frame
//
// IMPORTANT:
// - Angles are expressed in the robot frame.
// - 0 rad corresponds to the robot heading direction.
// - Positive angles rotate counter-clockwise.
// - The effective FOV interval is:
//     [fov_offset - fov/2 , fov_offset + fov/2]
std::vector<double> detection_fov_offset = {0, M_PI/2., 0};

// Prior uncertain object positions ([x], [y])
std::vector<IntervalVector> boxes_to_contract = {{{0.2,1.2}, {-1.6,-1.2}}, {{0,1}, {-7.5,-6.5}}, {{-6.5,-1}, {2.5,5}}};



int main()
{
    // Generate the robot trajectory and its associated tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Loop over each detection
    for (int i=0; i<detection_time.size(); i++)
    {
        // Robot pose at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Display the detection pie:
        // - distance ∈ [0, r_max]
        // - angle ∈ heading + robot-frame FOV
        fig.draw_pie(px, py, Interval(0,detection_range[i]), Interval(detection_fov_offset[i]+heading).inflate(detection_fov[i]/2.),"black[gray]");

        // Display initial uncertain object position
        fig.draw_box(boxes_to_contract[i],"black");

        // Create the SepDynPie separator
        SepDynPie my_sep(my_tube, detection_range[i], detection_fov[i], detection_fov_offset[i], true);

        // Detection time as a degenerated interval
        Interval detection_time_interval(detection_time[i]);
        
        // Box for inside contraction ([x, y, t])
        IntervalVector contracted_inside_box(3);
        contracted_inside_box[0] = boxes_to_contract[i][0];
        contracted_inside_box[1] = boxes_to_contract[i][1];
        contracted_inside_box[2] = detection_time_interval;

        // Box for outside contraction ([x, y, t])
        IntervalVector contracted_outside_box(3);
        contracted_outside_box[0] = boxes_to_contract[i][0];
        contracted_outside_box[1] = boxes_to_contract[i][1];
        contracted_outside_box[2] = detection_time_interval;
        
        // Apply the separator
        my_sep.separate(contracted_outside_box, contracted_inside_box);

        // Display results:
        // - red  : certainly detected
        // - blue : certainly not detected
        fig.draw_box(contracted_inside_box.subvector(0,1),"red");
        fig.draw_box(contracted_outside_box.subvector(0,1),"blue");

        // Display robot pose
        fig.draw_vehicle(detection_time[i], &my_traj, 0.5);
    }

    // Set axis limits for readability
    fig.axis_limits(-7, 6, -8.5, 5);

    // Show figure without final robot pose
    fig.show(0);
    vibes::endDrawing();

	return EXIT_SUCCESS;
}