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
            }
            else if (letter == 'Y') {
                move.y = std::stod(value);
            }
            else if (letter == 'Z') {
                move.z = std::stod(value);
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

    while (std::getline(stream, line)) {
        program.add(line);
    }

    return program;
}

} // namespace gcode