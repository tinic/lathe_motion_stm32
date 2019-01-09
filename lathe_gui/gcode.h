#ifndef GCODE_H
#define GCODE_H

#include <string>
#include <vector>
#include <map>

class GCodeParser
{
public:
    GCodeParser();

    void loadAndParse(std::ifstream &t);

    const std::vector<std::string> &intermediate_code() const;
    const std::vector<std::string> &g_code() const;

private:
    void output_intermediate(
                double target_x_pos,
                double target_y_pos,
                double target_z_pos,
                double feed_rate,
                bool wait_for_index);

    void arc_xz(double position_x, double position_y, double position_z,
                       double target_x, double target_y, double target_z,
                       double offset_x, double offset_y, double offset_z,
                       double feed_rate,
                       bool is_clockwise_arc);

       std::vector<std::string> _intcode;
       std::vector<std::string> _gcode;

       std::map<size_t,size_t> _blockmap;
       std::map<size_t,size_t> _intmap;

       size_t current_block;
       int32_t current_bytes;

       int32_t prev_z_mul;
       int32_t prev_z_div;
       int32_t prev_x_mul;
       int32_t prev_x_div;
       int32_t prev_d_mul;
       int32_t prev_d_div;

};

#endif // GCODE_H
