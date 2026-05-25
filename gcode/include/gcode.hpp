#ifndef GCODE_INCLUDE_GUARD_HPP
#define GCODE_INCLUDE_GUARD_HPP

#include <string>
#include <vector>
#include <iosfwd>

namespace gcode
{

    /// \brief A single G-code move command
    struct GcodeMove
    {
        /// \brief command type: "G1", "G2", "G3"
        std::string command = "G1";

        /// \brief position in mm
        double x = 0.0, y = 0.0, z = 0.0;

        /// \brief arc center offsets in mm (G2/G3 only)
        double i = 0.0, j = 0.0;

        /// \brief feedrate in mm/min
        double f = 600.0;

        /// \bool
        bool has_x = false;
        bool has_y = false;
        bool has_z = false;
    
        bool has_i = false;
        bool has_j = false;
    
    };

    /// \brief A full G-code program (list of moves)
    struct GcodeProgram
    {
        std::vector<GcodeMove> moves;
    
    };

    /// \brief Read a single G-code line into a GcodeMove
    /// Supports formats like "G1 X50 Y-30 Z0.2 F600"
    /// or "G2 X-5 Y0 I-20 J0 F600"
    /// \param is the input stream
    /// \param move the move to populate
    /// \returns the input stream
    std::istream& operator>>(std::istream& is, GcodeMove& move);    

    /// \brief Read a full G-code program (multiple lines)
    /// Skips comments (lines starting with ;) and non-motion commands
    /// \param is the input stream
    /// \param program the program to populate
    /// \returns the input stream
    std::istream& operator>>(std::istream& is, GcodeProgram& program);   

    /// \brief Parse a G-code string into a program
    /// \param gcode_text raw G-code as a string
    /// \returns parsed GcodeProgram
    GcodeProgram parse(const std::string& gcode_text);

} // namespace gcode

#endif