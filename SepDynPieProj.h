/*
 * @file SepDynPieProj.h
 * @author Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date 2026
 *
 * @brief
 * SepDynPieProj is a high-level class designed for beginners to compute
 * the coverage area of a moving vehicle equipped with an oriented range sensor.
 *
 * The perception model combines:
 *  - a distance constraint (range),
 *  - an angular constraint (field of view, FOV),
 *  - and the vehicle heading along a known trajectory.
 *
 * The covered area corresponds to a pie-shaped sector attached to the vehicle
 * and moving with it over time.
 *
 * This class integrates:
 *  - the SepDynPie separator (local space-time consistency),
 *  - with the SepProj projection (integration over time),
 * into a single easy-to-use object.
 *
 * Beginners can directly use SepDynPieProj to compute the area covered
 * by an oriented sensor along a trajectory, without manipulating low-level
 * separators or projection operators.
 *
 * Advanced users can manually combine SepDynPie with SepProj
 * for finer control.
 *
 * The class supports:
 *  - binary coverage (inside / outside),
 *  - three-valued coverage (inside / uncertain / outside),
 * depending on the constructor used.
 */

#ifndef __SEP_DYN_PIE_PROJ_H___
#define __SEP_DYN_PIE_PROJ_H___

#include "codac.h"
#include "SepDynPie.h"

using namespace codac;

/*
 * @class SepDynPieProj
 * @brief
 * High-level wrapper for projecting the detection area of a moving vehicle
 * equipped with an oriented range sensor in a pie-shaped coverage pattern.
 *
 * The vehicle trajectory must contain at least:
 *  - x-position,
 *  - y-position,
 *  - heading (orientation).
 *
 * Robot frame and angle convention (VERY IMPORTANT):
 *
 * At each time t, the vehicle defines a local robot frame:
 *  - origin   : (x(t), y(t))
 *  - x-axis   : forward direction of the vehicle
 *  - y-axis   : left direction
 *
 * The heading θ(t) is the orientation of the robot frame
 * with respect to the global frame, and angles follow the
 * standard mathematical convention:
 *  - angles increase counterclockwise,
 *  - θ = 0 means facing the global +x direction.
 *
 * The field-of-view angle interval `delta` is defined in the ROBOT FRAME:
 *  - delta = 0 corresponds to the forward direction,
 *  - positive angles correspond to the left side,
 *  - negative angles correspond to the right side.
 *
 * At time t, the actual detection angles in the WORLD FRAME are:
 *      θ(t) + delta
 *
 * Examples:
 *  - delta = [-π/4, π/4] : forward-looking sensor (90° FOV)
 *  - delta = [π/2, π]   : left-facing sensor
 *  - delta = [-π, π]    : omnidirectional angular coverage
 *
 * IMPORTANT:
 *  - delta should normally be defined in [-π, π]
 *  - the class internally applies a modulo 2π operation
 *    as a safety mechanism for beginner users
 *
 * This class performs runtime checks and prints warnings
 * when parameters are inconsistent or suboptimal.
 */
class SepDynPieProj : public SepProj
{
    public:
        /*
         * @brief Binary coverage constructor using a scalar FOV
         *
         * @param traj The TubeVector representing the vehicle trajectory
         *             (x, y, heading, ...).
         * @param range Detection range (must be > 0).
         * @param fov Field-of-view angle (in radians, robot frame, positive).
         * @param t_proj Time interval over which the coverage is projected.
         * @param eps_proj Projection precision (must be > 0).
         * @param fov_offset Angular offset of the FOV (in radians, default 0).
         * @param overpass_warning If true, disables runtime warnings.
         *
         * This constructor corresponds to binary coverage:
         *  - inside  : certainly detected,
         *  - outside : certainly not detected.
         *
         * Warnings:
         *  - If traj has more than 3 dimensions, only (x, y, heading) are used.
         *  - If fov ≥ 2π, better performance can be achieved using
         *    SepDynDiskProj.
         */
        SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset = 0., bool overpass_warning = false);
        
        /*
         * @brief Binary coverage constructor using an angular Interval
         *
         * @param traj The TubeVector representing the vehicle trajectory
         *             (x, y, heading, ...).
         * @param range Detection range (must be > 0).
         * @param delta Field-of-view interval (in radians, robot frame).
         * @param t_proj Time interval over which the coverage is projected.
         * @param eps_proj Projection precision (must be > 0).
         * @param overpass_warning If true, disables runtime warnings.
         *
         * This constructor corresponds to binary coverage:
         *  - inside  : certainly detected,
         *  - outside : certainly not detected.
         *
         * Warnings:
         *  - If delta width ≥ 2π, the sensor becomes omnidirectional and
         *    SepDynDiskProj is more appropriate.
         */
        SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Three-valued coverage constructor (inside / uncertain / outside)
         *
         * @param traj The TubeVector representing the vehicle trajectory
         *             (x, y, heading, ...).
         * @param range Detection range where detection is certain.
         * @param range_border Extended range modeling uncertainty (≥ 0).
         * @param delta Field-of-view interval where detection is certain.
         * @param delta_border Extended FOV interval modeling uncertainty.
         * @param t_proj Time interval over which the coverage is projected.
         * @param eps_proj Projection precision (must be > 0).
         * @param overpass_warning If true, disables runtime warnings.
         *
         * The following must hold:
         *  - range_border ≥ 0
         *  - delta ⊆ delta_border (after modulo 2π)
         *
         * This constructor enables three-valued logic:
         *  - inside    : certainly detected,
         *  - uncertain : possibly detected,
         *  - outside   : certainly not detected.
         */
        SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
    
    private:
        // Internal separator used for projection, fully managed by this class
        SepDynPie _sep;
};

# endif