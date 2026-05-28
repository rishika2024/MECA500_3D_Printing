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
    };

    struct Program
    {
        std::vector<Move> moves;

        void add(const std::string& line);
        void clear();
        size_t size() const;
    };

    Move parse_line(const std::string& line);

    Program parse(const std::string& text);

} // namespace gcode

#endif