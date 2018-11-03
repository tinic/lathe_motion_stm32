#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

struct cycle_entry {
    int32_t target_axs;
    int32_t target_pos;

    int32_t stepper_mul_z;
    int32_t stepper_div_z;

    int32_t stepper_mul_x;
    int32_t stepper_div_x;

    int32_t stepper_mul_d;
    int32_t stepper_div_d;

    int32_t wait_for_index_zero;
};

static std::vector<cycle_entry> entries;

inline double mm_to_step_z(double mm) {
	mm *= 3200 / (25.4 / 12.0); return mm;
}

inline double mm_to_step_x(double mm) {
	mm *= 3200 / (25.4 / 10.0); return mm;
}

inline double mm_to_step_d(double mm) {
	mm *= 3200 / (25.4 / 12.0); return mm;
}

void add_45_move(double pos, double speedz, double speedx) {

	const double motor_steps_per_rev = 3200.0;
	const double encoder_steps_per_rev = 2880.0;
	
	const double lead_screw_tpi_z = 12.0;
	const double lead_screw_tpi_x = 10.0;
	const double lead_screw_tpi_d = 12.0;

	entries.push_back({
		int32_t(0),
		int32_t(mm_to_step_z(pos)),
		int32_t(speedz * lead_screw_tpi_z * motor_steps_per_rev * 10.0), 
		int32_t(25.4 * encoder_steps_per_rev * 10.0), 
		int32_t(speedx * lead_screw_tpi_x * motor_steps_per_rev * 10.0), 
		int32_t(25.4 * encoder_steps_per_rev * 10.0), 
		int32_t(0), 
		int32_t(1), 
		int32_t(0)});
}

void add_move(int32_t axis, double pos, double speed, bool wait) {

	const double motor_steps_per_rev = 3200.0;
	const double encoder_steps_per_rev = 2880.0;
	
	const double lead_screw_tpi_z = 12.0;
	const double lead_screw_tpi_x = 10.0;
	const double lead_screw_tpi_d = 12.0;

	switch (axis) {
		case 0: {
		entries.push_back({
			int32_t(0),
			int32_t(mm_to_step_z(pos)),
			int32_t(speed * lead_screw_tpi_z * motor_steps_per_rev * 10.0), 
			int32_t(25.4 * encoder_steps_per_rev * 10.0), 
			int32_t(0), 
			int32_t(1), 
			int32_t(0), 
			int32_t(1), 
			int32_t(wait ? 1 : 0)});
		} break;
		case 1: {
		entries.push_back({
			int32_t(1),
			int32_t(mm_to_step_x(pos)),
			int32_t(0), 
			int32_t(1), 
			int32_t(speed * lead_screw_tpi_x * motor_steps_per_rev * 10.0), 
			int32_t(25.4 * encoder_steps_per_rev * 10.0), 
			int32_t(0), 
			int32_t(1), 
			int32_t(wait ? 1 : 0)});
		} break;
		case 2: {
		entries.push_back({
			int32_t(2),
			int32_t(mm_to_step_d(pos)),
			int32_t(0), 
			int32_t(1), 
			int32_t(0), 
			int32_t(1), 
			int32_t(speed * lead_screw_tpi_d * motor_steps_per_rev * 10.0), 
			int32_t(25.4 * encoder_steps_per_rev * 10.0), 
			int32_t(wait ? 1 : 0)});
		} break;
	}
}


void convert_and_print() {

	std::stringstream ss;

	ss << std::hex;
	ss << std::right;
	ss << std::uppercase;
	ss << std::setfill('0');

	ss << "CR\n";
	
	for (int32_t c = 0; c < entries.size(); c++) {

		ss << 'C';

		switch(entries[c].target_axs) {
			case 0: {
				ss << 'Z';
			} break;
			case 1: {
				ss << 'X';
			} break;
			case 2: {
				ss << 'D';
			} break;
		}

		ss << std::setw(8);
		ss << entries[c].target_pos;

		ss << std::setw(8);
		ss << entries[c].stepper_mul_z;
		ss << std::setw(8);
		ss << entries[c].stepper_div_z;

		ss << std::setw(8);
		ss << entries[c].stepper_mul_x;
		ss << std::setw(8);
		ss << entries[c].stepper_div_x;

		ss << std::setw(8);
		ss << entries[c].stepper_mul_d;
		ss << std::setw(8);
		ss << entries[c].stepper_div_d;

		ss << std::setw(8);
		ss << entries[c].wait_for_index_zero;

		ss << "\n";
	}

    ss << "CS\n";

	std::cout << ss.str();
}

const double cut_speed = 25.4 / 18.0;

int main() {

#if 1
	// move to start
	add_move(1, 10.0000, +cut_speed, false);

	for (int32_t c=0; c<15; c++) {
    	add_move(1, std::max(3.5000,7.9375 - double(c) * 0.3), -cut_speed, false);
		add_move(0, +22.0000 - double(c) * 0.2, +0.100, false);
    	add_move(1,  10.0000, +cut_speed, false);
		add_move(0,   0.0000, -cut_speed, false);
    }

	add_move(1,  3.500, -cut_speed, false);
    add_move(0, +5.000, +0.100, false);
    add_45_move(+5.750, +0.100, -0.100);

	add_move(0, +19.000, +0.100, false);
	add_move(1,  10.000, +cut_speed, false);
	add_move(0,   0.000, -cut_speed, false);

#else

	// move to start
	add_move(1, 10.0000, +cut_speed, false);

	for (int32_t c=0; c<3; c++) {
    	add_move(1, std::max(7.45000,7.9375 - double(c) * 0.4), -cut_speed, false);
		add_move(0, +20.0000, +0.100, false);
    	add_move(1,  10.0000, +cut_speed, false);
		add_move(0,   0.0000, -cut_speed, false);
    }

	add_move(1,  7.5000, -cut_speed, false);

	for (int32_t c=0; c<25; c++) {
		// cut thread
		add_move(0, +17.5000, cut_speed, true );
	
		// dwell
		add_move(2, + 2.0000, +cut_speed, false);
		add_move(2,   0.0000, -cut_speed, false);

		// pull back
		add_move(1,  12.9375, +cut_speed, false);
		// move to start
		add_move(0,   0.0000, -cut_speed, false);
		// move to next cut
		add_move(1, std::max(6.4000,7.5000 - double(c) * 0.05), -cut_speed, false);
	}

  	add_move(1,  10.0000, +cut_speed, false);

	for (int32_t c=0; c<4; c++) {
    	add_move(1, std::max(6.500,7.500 - double(c) * 0.4), -cut_speed, false);
		add_move(0, + 8.5000, +0.100, false);
    	add_move(1,  10.0000, +cut_speed, false);
		add_move(0,   0.0000, -cut_speed, false);
    }

#endif

	convert_and_print();
	
	return 0;
}
