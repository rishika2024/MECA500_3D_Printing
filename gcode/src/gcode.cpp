#include <sstream>
#include "gcode.hpp"

namespace gcode
{

    Move parse_line(const std::string& line)
    {
        Move move;

        if (line.empty() || line[0] == ';') return move;

        std::istringstream tokens(line);
        std::string token;

        while (tokens >> token) {
            char letter = token[0];
            std::string value = token.substr(1);

            if (letter == 'G') {
                move.cmd = "G" + value;                
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
            else if (letter == 'E') {
                move.e = std::stod(value);
                move.has_e = true;
            }  
            else if (letter == 'F') {
                move.f = std::stod(value);
                move.has_f = true;
            }          
        }

        return move;
    }

    void Program::add(const std::string& line)
    {
        Move move = parse_line(line);
        if (!move.cmd.empty()) {
            moves.push_back(move);
        }
    }

    void Program::clear() { moves.clear(); }

    size_t Program::size() const { return moves.size(); }

    Program parse(const std::string& text)
{
    Program program;
    // Replace literal \n with actual newlines
    std::string clean = text;
    size_t pos = 0;
    while ((pos = clean.find("\\n", pos)) != std::string::npos) {
        clean.replace(pos, 2, "\n");
    }

    std::istringstream stream(clean);
    std::string line;

    bool relative_mode = false;
    double cur_x = 0.0, cur_y = 0.0, cur_z = 0.0;

    while (std::getline(stream, line)) {
        Move move = parse_line(line);
        if (move.cmd.empty()) continue;

        // G90 = absolute mode, G91 = relative mode
        if (move.cmd == "G90") {
            relative_mode = false;
            continue; 
        }
        if (move.cmd == "G91") {
            relative_mode = true;
            continue;
        }

        if (relative_mode) {
            if (move.has_x) {
                move.x = cur_x + move.x;
            }
            else {
                move.x = cur_x;
            }
            if (move.has_y) {
                move.y = cur_y + move.y;
            }
            else {
                move.y = cur_y;
            }
            if (move.has_z) {
                move.z = cur_z + move.z;
            }
            else {
                move.z = cur_z;
            }
        }
        else {
            if (!move.has_x) {
                move.x = cur_x;
            }
            if (!move.has_y) {
                move.y = cur_y;
            }
            if (!move.has_z) {
                move.z = cur_z;
            }
        }

        cur_x = move.x;
        cur_y = move.y;
        cur_z = move.z;

        program.moves.push_back(move);
    }

    return program;
}

} // namespace gcode