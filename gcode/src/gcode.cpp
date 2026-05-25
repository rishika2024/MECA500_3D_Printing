#include <sstream>
#include <string>
#include "gcode.hpp"

namespace gcode
{

    std::istream& operator>>(std::istream& is, GcodeMove& move)
    {
        std::string line;
        if (!std::getline(is, line)) return is;

        // Skip empty and comments
        if (line.empty() || line[0] == ';') {
            move.command = "";
            return is;
        }

        // Split line into tokens and check each one
        std::istringstream tokens(line);
        std::string token;

        while (tokens >> token) {
            char letter = token[0];
            std::string value = token.substr(1);

            if (letter == 'G') {
                move.command = "G" + value;
                if (move.command == "G0") move.command = "G1"; //G1 and G0 are both linear moves, we will treat them the same
            }
            else if (letter == 'X') { 
                move.x = std::stod(value); 
                move.has_x = true;
            }
            else if (letter == 'Y') {
                move.y = std::stod(value); 
                move.has_y = true;
            }
            else if (letter == 'Z') {
                move.z = std::stod(value);
                move.has_z = true;
            }
            else if (letter == 'I') {
                move.i = std::stod(value);
                move.has_i = true;
            }
            else if (letter == 'J') {
                move.j = std::stod(value);
                move.has_j = true;
            }
            else if (letter == 'F') {
                move.f = std::stod(value); 
            }
            // skip anything else (M commands, comments, etc.)
        }

        return is;
    }

    std::istream& operator>>(std::istream& is, GcodeProgram& program)
    {
        program.moves.clear();
        GcodeMove move;

        while (is >> move) {
            if (move.command.empty()) continue;
            program.moves.push_back(move);
            move = GcodeMove{};
        }
        return is;
    }

    GcodeProgram parse(const std::string& gcode_text)
    {
        GcodeProgram program;
        std::istringstream stream(gcode_text);
        stream >> program;
        return program;
    }

} // namespace gcode