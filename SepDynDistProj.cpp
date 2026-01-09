/*
 * @file SepDynDistProj.cpp
 * @brief Implementation of SepDynDistProj, a high-level class for projecting
 *        the coverage of a moving object along a trajectory using a ranging sensor.
 *
 * This class combines the SepDynDist separator with the SepProj projection, 
 * making it easy for beginners to compute coverage over a trajectory.
 *
 * See SepDynDistProj.h for full class and constructor documentation.
*/

#include "SepDynDistProj.h"

namespace 
{
    // -------------------------------
    // Helper functions for input checks
    // -------------------------------

    // Check that the projection time interval is within the trajectory time domain
    const Interval verify_t_proj(const Interval t_proj, const TubeVector& traj)
    {
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(..., const Interval& t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        return t_proj;
    }
    
    // Check that the projection precision is positive
    const double verify_eps_proj(const double eps_proj)
    {
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(..., const Interval& t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }
        
        return eps_proj;
    }

    // -------------------------------
    // Functions to generate SepDynDist separators
    // -------------------------------

    // Binary coverage constructor using r_in and r_out
    SepDynDist generate_sep(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check r_in and r_out
        if(r_in>r_out)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning): The double r_out must be greater or equal to the double r_in");
        }
        if(r_in<0)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning): The double r_in must be greater or equal to 0");
        }
        if(r_in==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynDistProj with r_in = 0 => Please use the SepDynDiskProj class for better performances" <<std::endl;
        }

        // Create the separator
        return SepDynDist(traj, r_in, r_out, true);
    }

    // Binary coverage constructor using a single range
    SepDynDist generate_sep2(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if(range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynDistProj with the lower bound of the range being equal to 0 => Please use the SepDynDiskProj class for better performances" <<std::endl;
        }

        // Create the separator
        return SepDynDist(traj, range, true);
    }

    // Three-valued coverage using range and border_width
    SepDynDist generate_sep3(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if(range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynDistProj with the lower bound of the range being equal to 0 => Please use the SepDynDiskProj class for better performances" <<std::endl;
        }

        // Check border size
        if (border_width<0)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double border_width must be greater than or equal to 0");
        }
        if (range.lb()-border_width<0 && !overpass_warning)
        {
            std::cerr<< "[WARNING]  SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The width of the border is too large compare to the range lower bound (substracting the border width with the range lower bound results in a value lower than 0). The lower bound of the uncertain border will therefore be truncated to 0" <<std::endl;
        }

        // Create the separator
        return SepDynDist(traj, range, border_width, true);
    }

    // Three-valued coverage using range and uncertain_range
    SepDynDist generate_sep4(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check range interval
        if (!range.is_subset(Interval(0,oo))) 
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of [0,+inf]");
        }
        if(range.lb()==0 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynDistProj with the lower bound of the range being equal to 0 => Please use the SepDynDiskProj class for better performances" <<std::endl;
        }

        // Check uncertain range interval
        if (!uncertain_range.is_subset(Interval(0,oo))) {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval uncertain_range must be a subset of [0,+inf]");
        }
        if(!range.is_subset(uncertain_range))
        {
            throw std::range_error("[ERROR] SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval range must be a subset of the Interval uncertain_range");
        }

        // Create the separator
        return SepDynDist(traj, range, uncertain_range, true);
    }
}


// -------------------------------
// Constructor definitions
// -------------------------------


// Binary coverage (r_in/r_out)
SepDynDistProj::SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(generate_sep(traj, r_in, r_out, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj, traj), verify_eps_proj(eps_proj))
{}

// Binary coverage (single range)
SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(generate_sep2(traj, range, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj, traj), verify_eps_proj(eps_proj))
{}


// Three-valued coverage (range + border_width)
SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(generate_sep3(traj, range, border_width, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj, traj), verify_eps_proj(eps_proj))
{}

// Three-valued coverage (range + uncertain_range)
SepDynDistProj::SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(generate_sep4(traj, range, uncertain_range, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj, traj), verify_eps_proj(eps_proj))
{}