/*
 * @file SepDynPolar.cpp
 * @brief Implementation of the SepDynPolar local separator for dynamic polar perception.
 *
 * This file contains the implementation of the SepDynPolar class, a separator derived
 * from codac::SepCtcPair. SepDynPolar computes the perceived area for a robot with
 * an oriented range sensor (polar sensor) following a 2D trajectory.
 *
 * The separator represents:
 *   - Inside the perception set (guaranteed perceived area) using _ctcin (CtcDynPolar)
 *   - Outside the perception set (guaranteed not perceived) using _ctcout (CtcUnion)
 *
 * SepDynPolar supports both binary logic (perceived/not-perceived) and three-valued logic
 * (perceived / uncertain / not perceived), depending on the constructor used.
 *
 * Key implementation notes:
 *   - The trajectory is assumed thin (without uncertainty) and only the first three
 *     components are used (x, y, t or x, y, heading)
 *   - _ctcin represents the area guaranteed to be perceived
 *   - _ctcout represents the area guaranteed to be not perceived
 *   - Constructor variations allow specifying:
 *       - Range as Interval
 *       - Angular field of view (delta)
 *       - Uncertainty with uncertain_range / uncertain_delta
 *
 * See SepDynPolar.h for full class and constructor documentation.
 */

#include "SepDynPolar.h"
#include "CtcDynDist.h"

namespace 
{
    // -------------------------
    // ---- Helpers ----
    // -------------------------

    /*
     * @brief Normalize an angular interval modulo 2π.
     *
     * This function maps an angular interval to an equivalent interval whose
     * mean lies within [-π, π]. It is used to avoid angular discontinuities
     * when dealing with polar constraints.
     *
     * If the width of the interval is greater than or equal to 2π, the angle
     * fully wraps around the circle and the result is set to [-π, π].
     *
     * @param delta Angular interval to normalize.
     * @return Interval equivalent to delta modulo 2π.
     */
    Interval modulo2pi(const Interval delta)
    {
        // If the field of view spans the full circle (or more),
        // angular constraints are meaningless
        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            // Return the full angular domain [-π, π]
            return (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI;
        }
        
        // Otherwise, ensure that the interval mean lies within [-π, π]
        Interval mod_delta(delta);
        double mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;

        // Shift interval by multiples of 2π until mean is within bounds
        while(mean_angle>M_PI)
        {
            mod_delta = mod_delta - 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }
        
        // Shift interval by multiples of -2π until mean is within bounds
        while(mean_angle<-M_PI)
        {
            mod_delta = mod_delta + 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }

        // Return the interval delta modulo 2π and centered on [-π, π]
        return mod_delta;
    }

    // -------------------------
    // ---- _ctcin creation ----
    // -------------------------

    /*
     * @brief Verify inputs and create _ctcin for binary polar perception.
     */
    CtcDynPolar verify_and_generate_ctcin_init(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning)
    {
        // Verify trajectory dimension
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify distance range
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning): You initialized an instance of class SepDynPolar with the lower bound of the range being equal to 0 => Please use the SepDynPie class for better performances" <<std::endl;
        }

        // Verify delta
        Interval mod_delta = modulo2pi(delta);
        if (delta.ub()-delta.lb()>=2*M_PI && !overpass_warning)
        {
            std::cerr<<"[WARNING] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning): You initialized an instance of class SepDynPolar with an Interval delta with a width greater than or equal to 2*PI. Please use the SepDynDist class for better performances."<<std::endl;
        }

        return CtcDynPolar(traj, range, mod_delta, true);
    }

    /*
     * @brief Verify inputs and create _ctcin for three-valued polar perception.
     */
    CtcDynPolar verify_and_generate_ctcin_init2(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning)
    {
        // Verify trajectory dimension
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify distance range
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if (range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): You initialized an instance of class SepDynPolar with the lower bound of the range being equal to 0 => Please use the SepDynPie class for better performances" <<std::endl;
        }

        // Verify uncertain range
        if (!uncertain_range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input Interval uncertain_range must be a subset of [0,+inf]");
        }
        if (!range.is_subset(uncertain_range))
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input Interval range must be a subset of the Interval uncertain_range");
        }

        // Verify delta
        Interval mod_delta = modulo2pi(delta);
        if (delta.ub()-delta.lb()>=2*M_PI && !overpass_warning)
        {
            std::cerr<<"[WARNING] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): You initialized an instance of class SepDynPolar with an Interval delta with a width greater than or equal to 2*PI. Please use the SepDynDist class for better performances."<<std::endl;
        }

        // Verify uncertain_delta
        Interval mod_uncertain_delta = modulo2pi(uncertain_delta);
        if (!mod_delta.is_subset(mod_uncertain_delta))
        {
            throw std::range_error("[ERROR] SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning): The input Interval delta must be a subset of the Interval uncertain_delta");
        }

        return CtcDynPolar(traj, uncertain_range, mod_uncertain_delta, true);
    }

    // -------------------------
    // ---- _ctcout creation ----
    // -------------------------

    /*
     * @brief Generate _ctcout for binary polar perception.
     */
    CtcUnion generate_ctcout_init(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning)
    {
        // Outside distance zone
        CtcDynDist ctc_upper_range(traj, Interval(range.ub(),oo), true);
        CtcDynDist ctc_lower_range(traj, Interval(0, range.lb()), true);
        CtcUnion ctc_outside(ctc_lower_range, ctc_upper_range);

        // Outside angular zone
        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            return ctc_outside;
        }
        
        Interval compl_delta = Interval(delta.ub(), delta.lb() + 2*M_PI);
        compl_delta = modulo2pi(compl_delta);
        CtcDynPolar ctc_complementary(traj, range, compl_delta, true);

        // Contractor for outside area
        return CtcUnion(ctc_outside, ctc_complementary);
    }

    /*
     * @brief Generate _ctcout for three-valued polar perception.
     */
    CtcUnion generate_ctcout_init2(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning)
    {
        // Outside distance zone
        CtcDynDist ctc_upper_range(traj, Interval(range.ub(),oo), true);
        CtcDynDist ctc_lower_range(traj, Interval(0, range.lb()), true);
        CtcUnion ctc_outside(ctc_lower_range, ctc_upper_range);

        // Outside angular zone
        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            return ctc_outside;
        }
        
        Interval compl_delta = Interval(delta.ub(), delta.lb() + 2*M_PI);
        compl_delta = modulo2pi(compl_delta);
        CtcDynPolar ctc_complementary(traj, range, compl_delta, true);
        
        // Contractor for outside area
        return CtcUnion(ctc_outside, ctc_complementary);
    }
}

// -------------------------
// ---- Constructors ----
// -------------------------

SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval delta, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init(traj, range, delta, overpass_warning)),
_ctcout(generate_ctcout_init(traj, range, delta, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}

SepDynPolar::SepDynPolar(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, bool overpass_warning):
_ctcin(verify_and_generate_ctcin_init2(traj, range, uncertain_range, delta, uncertain_delta, overpass_warning)),
_ctcout(generate_ctcout_init2(traj, range, uncertain_range, delta, uncertain_delta, overpass_warning)),
SepCtcPair(_ctcout,_ctcin)
{}
