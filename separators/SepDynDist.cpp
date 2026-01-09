/*
 * @file SepDynDist.cpp
 * @brief Implementation of the SepDynDist local separator for dynamic range-based perception.
 *
 * This file contains the implementation of the SepDynDist class, a separator derived
 * from codac::SepCtcPair. SepDynDist computes the perceived area for a robot with
 * a range sensor following a 2D trajectory.
 *
 * The separator represents:
 *   - Inside the perception set (guaranteed perceived area) using _ctcin (CtcDynDist)
 *   - Outside the perception set (guaranteed not perceived) using _ctcout (CtcUnion)
 *     which combines two CtcDynDist contractors for the inner and outer unknown zones.
 *
 * SepDynDist supports both binary logic (perceived/not-perceived) and three-valued logic
 * (perceived / uncertain / not perceived), depending on which constructor is used.
 *
 * Key implementation notes:
 *   - The trajectory is assumed thin (without uncertainty) and only the first two
 *     components are used.
 *   - _ctcin represents the area guaranteed to be perceived.
 *   - _ctcout represents the area guaranteed to be not perceived.
 *   - Constructor variations allow specifying:
 *       - Range as doubles or Interval
 *       - Uncertainty zone with border_width or uncertain_range
 *
 * See SepDynDist.h for full class and constructor documentation.
*/

#include "SepDynDist.h"
#include <iostream>

namespace 
{
    // -------------------------
    // ---- _ctcin helpers -----
    // -------------------------

    /*
     * @brief Verify inputs and create _ctcin for the binary range constructor.
     * @param traj TubeVector of robot trajectory (dimension >=2)
     * @param r_in Minimum perception range
     * @param r_out Maximum perception range
     * @param overpass_warning flag to suppress warnings
     * @return CtcDynDist instance representing the guaranteed perceived area
     *
     * Geometric interpretation:
     *   - The guaranteed perceived area corresponds to points whose distance d
     *     to the robot at time t satisfies r_in <= d <= r_out,
     *     with d = sqrt((px-x)^2 + (py-y)^2)
    */
    CtcDynDist verify_and_generate_ctcin_init(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range values
        if (r_in>r_out)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning): The double r_out must be greater or equal to the double r_in");
        }
        if (r_in<0)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning): The double r_in must be greater or equal to 0");
        }
        if (r_in==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning): You initialized an instance of class SepDynDist with r_in = 0 => Please use the SepDynDisk class for better performances" <<std::endl;
        }

        // Create contractor for guaranteed perceived area
        return CtcDynDist(traj, Interval(r_in,r_out), true);
    }

    /*
     * @brief Verify inputs and create _ctcin for interval range constructor (binary logic).
     * @param traj TubeVector of robot trajectory
     * @param range Interval representing perception range [r_min, r_max]
     * @param overpass_warning flag to suppress warnings
     * @return CtcDynDist for guaranteed perceived area
    */
    CtcDynDist verify_and_generate_ctcin_init2(const TubeVector& traj, const Interval range, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning): You initialized an instance of class SepDynDist with the lower bound of the range being equal to 0 => Please use the SepDynDisk class for better performances" <<std::endl;
        }

        // Create contractor for guaranteed perceived area
        return CtcDynDist(traj, range, true);
    }

    /*
     * @brief Verify inputs and create _ctcin for three-valued logic using border_width.
     * @param traj TubeVector of robot trajectory
     * @param range Interval representing perceived area
     * @param border_width width of uncertain boundary
     * @param overpass_warning flag
     * @return CtcDynDist representing perceived area within uncertain boundary
     *
     * Geometric interpretation:
     *   - Perceived area: range
     *   - Uncertain boundary: range inflated by border_width
     *   - Outside: anything beyond the uncertain boundary
    */
    CtcDynDist verify_and_generate_ctcin_init3(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning): You initialized an instance of class SepDynDist with the lower bound of the range being equal to 0 => Please use the SepDynDisk class for better performances" <<std::endl;
        }

        // Verify the border value
        if (border_width<0)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning): The input double border_width must be greater than or equal to 0");
        }
        if (range.lb()-border_width<0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning): The width of the border is too large compare to the range lower bound (substracting the border width with the range lower bound results in a value lower than 0). The lower bound of the uncertain border will therefore be truncated to 0" <<std::endl;
        }

        // Create contractor for guaranteed perceived area
        Interval extended_range = Interval(range).inflate(border_width) & Interval(0,oo);
        return CtcDynDist(traj, extended_range, true);
    }

    /*
     * @brief Verify inputs and create _ctcin for three-valued logic with uncertain_range.
     * @param traj TubeVector of robot trajectory
     * @param range Interval representing perceived area
     * @param uncertain_range Interval representing boundary of uncertainty
     * @param overpass_warning flag
     * @return CtcDynDist representing guaranteed peceived area
    */
    CtcDynDist verify_and_generate_ctcin_init4(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning)
    {
        // Verify trajectory dimensions
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Verify range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning): You initialized an instance of class SepDynDist with the lower bound of the range being equal to 0 => Please use the SepDynDisk class for better performances" <<std::endl;
        }

        // Verify uncertain range interval
        if (!uncertain_range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning): The input Interval uncertain_range must be a subset of [0,+inf]");
        }
        if (!range.is_subset(uncertain_range))
        {
            throw std::range_error("[ERROR] SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning): The input Interval range must be a subset of the Interval uncertain_range");
        }

        // Create contractor for guaranteed perceived area
        return CtcDynDist(traj, uncertain_range, true);
    }

    // -------------------------
    // ---- _ctcout helpers ----
    // -------------------------

    /*
     * @brief Generate _ctcout for binary constructor with double inputs.
     * @param traj TubeVector
     * @param r_in minimum range
     * @param r_out maximum range
     * @param overpass_warning flag
     * @return CtcUnion representing guaranteed not-perceived area
     *
     * Semantics:
     *   - Two contractors:
     *       1) [0, r_in] -> guaranteed not-perceived inside
     *       2) [r_out, +inf] -> guaranteed not-perceived outside
    */
    CtcUnion generate_ctcout_init(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning)
    {
        CtcDynDist ctc_inside(traj, Interval(0, r_in), true);
        CtcDynDist ctc_outside(traj, Interval(r_out, oo), true);
        
        return CtcUnion(ctc_inside, ctc_outside);
    }
    
    // The other _ctcout helpers are similar, just adjusting input intervals
    CtcUnion generate_ctcout_init2(const TubeVector& traj, const Interval range, bool overpass_warning)
    {
        CtcDynDist ctc_inside(traj, Interval(0, range.lb()), true);
        CtcDynDist ctc_outside(traj, Interval(range.ub(), oo), true);
        
        return CtcUnion(ctc_inside, ctc_outside);
    }

    // The other _ctcout helpers are similar, just adjusting input intervals
    CtcUnion generate_ctcout_init3(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning)
    {
        CtcDynDist ctc_inside(traj, Interval(0, range.lb()), true);
        CtcDynDist ctc_outside(traj, Interval(range.ub(), oo), true);
        
        return CtcUnion(ctc_inside, ctc_outside);
    }

    // The other _ctcout helpers are similar, just adjusting input intervals
    CtcUnion generate_ctcout_init4(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning)
    {
        CtcDynDist ctc_inside(traj, Interval(0, range.lb()), true);
        CtcDynDist ctc_outside(traj, Interval(range.ub(), oo), true);
        
        return CtcUnion(ctc_inside, ctc_outside);
    }
}

// -------------------------
// ---- Constructors ----
// -------------------------

/*
 * @brief Constructor: binary perception using double r_in/r_out.
 * @param traj TubeVector of trajectory
 * @param r_in minimum perception range
 * @param r_out maximum perception range
 * @param overpass_warning flag
*/
SepDynDist::SepDynDist(const TubeVector& traj, const double r_in, const double r_out, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init(traj, r_in, r_out, overpass_warning)),
_ctcout(generate_ctcout_init(traj, r_in, r_out, overpass_warning)),
SepCtcPair(_ctcout,_ctcin) 
{}

/*
 * @brief Constructor: binary perception using interval.
 * @param traj TubeVector
 * @param range perception interval
 * @param overpass_warning flag
*/
SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init2(traj, range, overpass_warning)),
_ctcout(generate_ctcout_init2(traj, range, overpass_warning)),
SepCtcPair(_ctcout,_ctcin) 
{}

/*
 * @brief Constructor: three-valued logic with border width.
 * @param traj TubeVector
 * @param range perceived area
 * @param border_width width of uncertain area
 * @param overpass_warning flag
*/
SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const double border_width, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init3(traj, range, border_width, overpass_warning)),
_ctcout(generate_ctcout_init3(traj, range, border_width, overpass_warning)),
SepCtcPair(_ctcout,_ctcin) 
{}


/*
 * @brief Constructor: three-valued logic with uncertain range.
 * @param traj TubeVector
 * @param range perceived area
 * @param uncertain_range interval of uncertain area
 * @param overpass_warning flag
*/
SepDynDist::SepDynDist(const TubeVector& traj, const Interval range, const Interval uncertain_range, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init4(traj, range, uncertain_range, overpass_warning)),
_ctcout(generate_ctcout_init4(traj, range, uncertain_range, overpass_warning)),
SepCtcPair(_ctcout,_ctcin) 
{}