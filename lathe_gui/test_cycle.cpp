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
	mm /= 25.4 * 12.0; mm *= 3200; return mm;
}

inline double mm_to_step_x(double mm) {
	mm /= 25.4 * 10.0; mm *= 3200; return mm;
}

inline double mm_to_step_d(double mm) {
	mm /= 25.4 *  8.0; mm *= 3200; return mm;
}

void add_move(int32_t axis, double distance, double speed, bool wait) {

	const double motor_steps_per_rev = 3200.0;
	const double encoder_steps_per_rev = 2880.0;
	
	const double lead_screw_tpi_z = 12.0;
	const double lead_screw_tpi_x = 10.0;
	const double lead_screw_tpi_d =  8.0;

	switch (axis) {
		case 0: {
		entries.push_back({
			int32_t(0),
			int32_t(mm_to_step_z(distance)),
			int32_t(0.5 * lead_screw_tpi_z * motor_steps_per_rev * 10.0), 
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
			int32_t(mm_to_step_x(distance)),
			int32_t(0), 
			int32_t(1), 
			int32_t(0.5 * lead_screw_tpi_x * motor_steps_per_rev * 10.0), 
			int32_t(25.4 * encoder_steps_per_rev * 10.0), 
			int32_t(0), 
			int32_t(1), 
			int32_t(wait ? 1 : 0)});
		} break;
		case 2: {
		entries.push_back({
			int32_t(2),
			int32_t(mm_to_step_d(distance)),
			int32_t(0), 
			int32_t(1), 
			int32_t(0), 
			int32_t(1), 
			int32_t(0.5 * lead_screw_tpi_d * motor_steps_per_rev * 10.0), 
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

	ss << 'C';

	for (int32_t c = 0; c < entries.size(); c++) {

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

	}

	std::cout << ss.str() << '\n';
}

int main() {
	// move to start
	add_move(1, - 7.9375, 0.500, false);

	for (int32_t c=0; c<10; c++) {
		// cut thread
		add_move(0, +40.0000, 5.625, true );
	
		// dwell
		add_move(2, +10.0000, 0.500, false);
		add_move(2, -10.0000, 0.500, false);

		// pull back
		add_move(1, -12.9375, 0.500, false);
		// move to start
		add_move(0,   0.0000, 0.500, false);
		// move to next cut
		add_move(1, - 7.9375 + double(c) * 0.01, 0.500, false);
	}

	convert_and_print();
	
	return 0;
}
