/*
 * @file SepDynDisk.cpp
 * @brief Implementation of the SepDynDisk local separator for disk-based perception.
 *
 * This file contains the implementation of the SepDynDisk class, a separator derived
 * from codac::SepCtcPair. SepDynDisk computes the perceived area for a robot with
 * a disk-shaped range sensor following a 2D trajectory.
 *
 * The separator represents:
 *   - Inside the perception set (guaranteed perceived area) using _ctcin (CtcDynDist)
 *   - Outside the perception set (guaranteed not perceived) using _ctcout (CtcDynDist)
 *
 * SepDynDisk supports both binary logic (perceived/not-perceived) and three-valued logic
 * (perceived / uncertain / not perceived), depending on which constructor is used.
 *
 * Key implementation notes:
 *   - The trajectory is assumed thin (without uncertainty) and only the first two
 *     components are used.
 *   - _ctcin represents the area guaranteed to be perceived.
 *   - _ctcout represents the area guaranteed to be not perceived.
 *   - Compared to SepDynDist, no inner radius exists, so _ctcout is simpler and
 *     no CtcUnion is required.
 *
 * See SepDynDisk.h for full class and constructor documentation.
 */

#include "SepDynDisk.h"
#include <iostream>

namespace 
{
    // -------------------------
    // ---- _ctcin helpers -----
    // -------------------------

    /*
     * @brief Verify inputs and create _ctcin for binary perception logic.
     * @param traj TubeVector of robot trajectory (dimension >=2)
     * @param range Maximum perception range
     * @param overpass_warning flag
     * @return CtcDynDist representing the guaranteed perceived area
     *
     * Geometric interpretation:
     *   - The guaranteed perceived area corresponds to points whose distance d
     *     to the robot at time t satisfies 0 <= d <= range
     *     with d = sqrt((px-x)^2 + (py-y)^2)
    */
    CtcDynDist verify_and_generate_ctcin_init(const TubeVector& traj, const double range, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range
        if (range<=0)
        {
            throw std::range_error("[ERROR] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, bool overpass_warning): The input double range must be greater than 0");
        }

        // Create contractor for guaranteed perceived area
        return CtcDynDist(traj, Interval(0, range), true);
    }

    /*
     * @brief Verify inputs and create _ctcin for three-valued perception logic.
     * @param traj TubeVector of robot trajectory
     * @param range Maximum radius for sure coverage
     * @param border_width Width of uncertain boundary
     * @param overpass_warning flag
     * @return CtcDynDist representing the guaranteed perceived area plus uncertain border
    */
    CtcDynDist verify_and_generate_ctcin_init2(const TubeVector& traj, const double range, const double border_width, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range
        if (range<=0)
        {
            throw std::range_error("[ERROR] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify border width
        if (border_width<0)
        {
            throw std::range_error("[ERROR] SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning): The input double border_width must be greater than or equal to 0");
        }

        // Create contractor for guaranteed perceived area
        return CtcDynDist(traj, Interval(0, range + border_width), true);
    }

    // -------------------------
    // ---- _ctcout helpers ----
    // -------------------------

    /*
     * @brief Generate _ctcout for binary perception logic.
     * @param traj TubeVector
     * @param range Maximum range
     * @param overpass_warning flag
     * @return CtcDynDist representing guaranteed not-perceived area
     *
     * Semantics:
     *   - Outside region corresponds to distances larger than range
    */
    CtcDynDist generate_ctcout_init(const TubeVector& traj, const double range, bool overpass_warning)
    {
        return CtcDynDist(traj, Interval(range,oo), true);
    }

    /*
     * @brief Generate _ctcout for three-valued perception logic.
     * @param traj TubeVector
     * @param range Maximum range for sure coverage
     * @param border_width Width of uncertain border
     * @param overpass_warning flag
     * @return CtcDynDist representing guaranteed not-perceived area
    */
    CtcDynDist generate_ctcout_init2(const TubeVector& traj, const double range, const double border_width, bool overpass_warning)
    {
        return CtcDynDist(traj, Interval(range,oo), true);
    }
}

// -------------------------
// ---- Constructors ----
// -------------------------

/*
 * @brief Constructor: binary perception
 * @param traj TubeVector of trajectory
 * @param range Maximum disk radius
 * @param overpass_warning flag
*/
SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init(traj, range, overpass_warning)),
_ctcout(generate_ctcout_init(traj, range, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}

/*
 * @brief Constructor: three-valued perception
 * @param traj TubeVector
 * @param range Maximum radius for sure coverage
 * @param border_width Width of uncertain border
 * @param overpass_warning flag
*/
SepDynDisk::SepDynDisk(const TubeVector& traj, const double range, const double border_width, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init2(traj, range, border_width, overpass_warning)),
_ctcout(generate_ctcout_init2(traj, range, border_width, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}