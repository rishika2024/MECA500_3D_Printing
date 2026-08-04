#ifndef GCODE_INCLUDE_GUARD_HPP
#define GCODE_INCLUDE_GUARD_HPP

#include <string>
#include <vector>

namespace gcode
{

    struct Move
    {
        std::string cmd = "";
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double i = 0.0;
        double j = 0.0;
        double e = 0.0;
        double f = 0.0;        
        bool has_x = false;
        bool has_y = false;
        bool has_z = false;
        bool has_i = false;
        bool has_j = false;
        bool has_e = false;
        bool has_f = false;
    };

    struct Program
    {
        std::vector<Move> moves;
        // Reflects the M82/M83 declaration found in the file (defaults to
        // false/absolute if the file never declares one, matching the
        // historical convention). Move.e is always normalized to a
        // per-move delta by parse() regardless of this flag -- this is
        // for the caller to know which mode to assert on the real
        // printer to match what these deltas assume, instead of
        // hardcoding an assumption independently of what the file says.
        bool e_relative_mode = false;

        void add(const std::string& line);
        void clear();
        size_t size() const;
    };

    Move parse_line(const std::string& line);

    Program parse(const std::string& text);

} // namespace gcode

#endif