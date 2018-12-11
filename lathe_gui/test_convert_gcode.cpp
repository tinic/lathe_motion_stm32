#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include <math.h>

using namespace std;

namespace gpr {

enum address_type { ADDRESS_TYPE_INTEGER = 0, ADDRESS_TYPE_DOUBLE };

union addr_value {
  double dbl_val;
  int int_val;
};

class addr {
private:
  address_type ad_type;
  addr_value val;

public:
  addr(const address_type &p_ad_type, const addr_value &p_val) {
    ad_type = p_ad_type;
    val = p_val;
  }

  address_type tp() const { return ad_type; }

  double double_value() const {
    assert(tp() == ADDRESS_TYPE_DOUBLE);

    return val.dbl_val;
  }

  int int_value() const {
    assert(tp() == ADDRESS_TYPE_INTEGER);

    return val.int_val;
  }

  bool equals(const addr &other) const {
    if (other.tp() != tp()) {
      return false;
    }

    if (other.tp() == ADDRESS_TYPE_DOUBLE) {
      return other.double_value() == double_value();
    }

    return other.int_value() == int_value();
  }

  void print(std::ostream &out) const {
    if (tp() == ADDRESS_TYPE_DOUBLE) {
      out << double_value();
      return;
    }

    out << int_value();
  }
};

enum chunk_type {
  CHUNK_TYPE_COMMENT,
  CHUNK_TYPE_WORD_ADDRESS,
  CHUNK_TYPE_PERCENT,
  CHUNK_TYPE_WORD
};

struct comment_data {
  char left_delim;
  char right_delim;
  std::string comment_text;

  comment_data() : left_delim('('), right_delim(')'), comment_text("") {}
  comment_data(const char p_left_delim, const char p_right_delim,
               const std::string &p_comment_text)
      : left_delim(p_left_delim), right_delim(p_right_delim),
        comment_text(p_comment_text) {}
};

struct word_address_data {
  char wd;
  addr adr;

  word_address_data() : wd('\0'), adr(ADDRESS_TYPE_INTEGER, {-1}) {}

  word_address_data(const char p_wd, const addr p_adr) : wd(p_wd), adr(p_adr) {}
};

// chunk is the class that represents all data that can appear in a block.
// A chunk can be either a comment or a word-address pair. The vaue of the
// field chunk_tp is CHUNK_TYPE_COMMENT if the chunk is a comment and is
// CHUNK_TYPE_WORD_ADDRESS if the chunk is a word-address pair
// For example in the G-code program below:
//      (*** Toolpath 1 ***)
//      G0 X0.0 Y0.0 Z0.0
//      G1 X1.0 F23.0
//      G1 Z-1.0 F10.0
// The program consits of 4 blocks (block is just G-code speak for line).
// The first block contains 1 chunk, which is the comment "(*** Toolpath 1
// ***)". The second block contains 4 chunks, each of which is a pair of a word
// (a character) and an address (a number). The 4 chunks are:
// G0, X0.0, Y0.0, and Z0.0. In G0 the word is 'G' and the address is '0'.
// In X0.0 the word is 'X' and the address is '0.0', and so on.
class chunk {
private:
  chunk_type chunk_tp;

  // Comment fields;
  comment_data cd;

  // Word-address fields
  word_address_data wad;

  // Isolated word fields
  char single_word;

public:
  chunk() : chunk_tp(CHUNK_TYPE_PERCENT) {}
  virtual ~chunk() {}

  chunk(const char c) : chunk_tp(CHUNK_TYPE_WORD), single_word(c) {}

  chunk(const char p_left_delim, const char p_right_delim,
        const std::string &p_comment_text)
      : chunk_tp(CHUNK_TYPE_COMMENT),
        cd(p_left_delim, p_right_delim, p_comment_text) {}

  chunk(const char p_wd, const addr p_adr)
      : chunk_tp(CHUNK_TYPE_WORD_ADDRESS), cd('(', ')', ""), wad(p_wd, p_adr) {}

  chunk_type tp() const { return chunk_tp; }

  char get_left_delim() const {
    assert(tp() == CHUNK_TYPE_COMMENT);
    return cd.left_delim;
  }

  char get_right_delim() const {
    assert(tp() == CHUNK_TYPE_COMMENT);
    return cd.right_delim;
  }

  std::string get_comment_text() const {
    assert(tp() == CHUNK_TYPE_COMMENT);
    return cd.comment_text;
  }

  char get_word() const {
    assert(tp() == CHUNK_TYPE_WORD_ADDRESS);
    return wad.wd;
  }

  addr get_address() const {
    assert(tp() == CHUNK_TYPE_WORD_ADDRESS);
    return wad.adr;
  }

  char get_single_word() const {
    assert(tp() == CHUNK_TYPE_WORD);
    return single_word;
  }

  bool equals_word_address(const chunk &other_addr) const {
    assert(other_addr.tp() == CHUNK_TYPE_WORD_ADDRESS);

    return (get_word() == other_addr.get_word()) &&
           (get_address().equals(other_addr.get_address()));
  }

  bool equals_comment(const chunk &other_comment) const {
    assert(other_comment.tp() == CHUNK_TYPE_COMMENT);

    return (get_comment_text() == other_comment.get_comment_text()) &&
           (get_left_delim() == other_comment.get_left_delim()) &&
           (get_right_delim() == other_comment.get_right_delim());
  }

  virtual bool equals(const chunk &other) const {
    if (other.tp() != tp()) {
      return false;
    }

    if (tp() == CHUNK_TYPE_WORD_ADDRESS) {
      return equals_word_address(other);
    } else if (tp() == CHUNK_TYPE_COMMENT) {
      return equals_comment(other);
    } else if (tp() == CHUNK_TYPE_PERCENT) {
      // Any 2 percent chunks are always equal
      return true;
    } else {
      assert(false);
    }
  }

  void print_comment(std::ostream &stream) const {
    stream << get_left_delim() << get_comment_text() << get_right_delim();
  }

  void print_word_address(std::ostream &stream) const {
    stream << wad.wd;
    wad.adr.print(stream);
  }

  void print_word(std::ostream &stream) const { stream << get_single_word(); }

  void print(std::ostream &stream) const {
    if (tp() == CHUNK_TYPE_COMMENT) {
      print_comment(stream);
    } else if (tp() == CHUNK_TYPE_WORD_ADDRESS) {
      print_word_address(stream);
    } else if (tp() == CHUNK_TYPE_PERCENT) {
      stream << "%";
    } else if (tp() == CHUNK_TYPE_WORD) {
      print_word(stream);
    } else {
      assert(false);
    }
  }
};

chunk make_comment(const char start_delim, const char end_delim,
                   const std::string &comment_text);

chunk make_isolated_word(const char c);
chunk make_word_int(const char c, const int i);
chunk make_word_double(const char c, const double i);
chunk make_percent_chunk();

bool operator==(const chunk &l, const chunk &r);
bool operator!=(const chunk &l, const chunk &r);

std::ostream &operator<<(std::ostream &stream, const chunk &ic);

// A block is really just a line of code, so for example the following program:
//      (*** Toolpath 1 ***)
//      G0 X0.0 Y0.0 Z0.0
//      G1 X1.0 F23.0
//      G1 Z-1.0 F10.0
// consists of 4 blocks
class block {
protected:
  bool has_line_no;
  int line_no;
  bool slashed_out;
  std::vector<chunk> chunks;

  // Used to make viewing more convenient during debugging
  std::string debug_text;

public:
  block(const int p_line_no, const bool p_slashed_out,
        const std::vector<chunk> p_chunks)
      : has_line_no(true), line_no(p_line_no), slashed_out(p_slashed_out),
        chunks(p_chunks) {}

  block(const bool p_slashed_out, const std::vector<chunk> p_chunks)
      : has_line_no(false), line_no(-1), slashed_out(p_slashed_out),
        chunks(p_chunks) {}

  block(const block &other)
      : has_line_no(other.has_line_no), line_no(other.line_no),
        slashed_out(other.slashed_out) {

    for (size_t i = 0; i < other.chunks.size(); i++) {
      chunks.push_back(other.chunks[i]);
    }
  }

  block &operator=(const block &other) {
    has_line_no = other.has_line_no;
    line_no = other.line_no;
    slashed_out = other.slashed_out;
    for (size_t i = 0; i < other.chunks.size(); i++) {
      chunks.push_back(other.chunks[i]);
    }

    return *this;
  }

  std::string to_string() const {
    std::ostringstream ss;
    this->print(ss);
    return ss.str();
  }

  // Call this function on a block to set the variable debug_text.
  // This is useful when you want
  // to view information about the block in the debugger
  void set_debug_text(const std::string &text) { debug_text = text; }

  // Default version of set_debug_text that sets the debug text string
  // to a string representation of the block
  void set_debug_text() { set_debug_text(this->to_string()); }

  void print(std::ostream &stream) const {
    if (has_line_number()) {
      stream << "N" << line_number() << " ";
    }
    for (auto i : *this) {
      stream << i << " ";
    }
  }

  int size() const { return chunks.size(); }

  const chunk &get_chunk(const int i) const {
    assert(i < size());

    return chunks[i];
  }

  bool is_deleted() const { return slashed_out; }

  bool has_line_number() const { return has_line_no; }

  int line_number() const {
    assert(has_line_number());
    return line_no;
  }

  std::vector<chunk>::const_iterator begin() const {
    return std::begin(chunks);
  }
  std::vector<chunk>::const_iterator end() const { return std::end(chunks); }

  std::vector<chunk>::iterator begin() { return std::begin(chunks); }
  std::vector<chunk>::iterator end() { return std::end(chunks); }
};

class gcode_program {
protected:
  std::vector<block> blocks;

public:
  gcode_program(const std::vector<block> &p_blocks) : blocks(p_blocks) {}

  int num_blocks() const { return blocks.size(); }

  block get_block(const size_t i) {
    assert(i < blocks.size());
    return blocks[i];
  }

  std::vector<block>::const_iterator begin() const {
    return std::begin(blocks);
  }
  std::vector<block>::const_iterator end() const { return std::end(blocks); }

  std::vector<block>::iterator begin() { return std::begin(blocks); }
  std::vector<block>::iterator end() { return std::end(blocks); }
};

std::ostream &operator<<(std::ostream &stream, const block &block);

std::ostream &operator<<(std::ostream &stream, const gcode_program &program);

addr make_int_address(const int i);

addr make_double_address(const double i);

template <typename T> struct parse_stream {
  size_t i;
  vector<T> s;

  template <typename R> parse_stream<T>(R v) : s(v.begin(), v.end()) { i = 0; }

  T next() { return s[i]; }

  int chars_left() const { return i < s.size(); }

  parse_stream<T> &operator++(int) {
    i++;
    return *this;
  }

  parse_stream<T> &operator--(int) {
    i--;
    return *this;
  }

  typename vector<T>::const_iterator end() { return s.end(); }

  typename vector<T>::const_iterator begin() { return s.begin(); }

  typename vector<T>::const_iterator remaining() { return s.begin() + i; }
};

typedef parse_stream<char> parse_state;

bool is_num_char(const char c) {
  return (isdigit(c) || (c == '.') || (c == '-'));
}

void ignore_whitespace(parse_state &s) {
  while (s.chars_left() && (isspace(s.next()) || s.next() == '\r')) {
    s++;
  }
}

string string_remaining(parse_state &ps) {
  return string(ps.remaining(), ps.end());
}

void parse_char(char c, parse_state &s) {
  if (s.next() == c) {
    s++;
    return;
  }
  cout << "Cannot parse char " << c << " from string " << string_remaining(s)
       << endl;
  assert(false);
}

double parse_double(parse_stream<string> &s) {

  double v = stod(s.next());

  s++;

  return v;
}

int parse_int(parse_stream<string> &s) {

  int i = stoi(s.next());

  s++;

  return i;
}

addr parse_address(char c, parse_stream<string> &s) {
  switch (c) {
  case 'X':
  case 'Y':
  case 'Z':
  case 'A':
  case 'B':
  case 'C':
  case 'U':
  case 'V':
  case 'W':
  case 'I':
  case 'J':
  case 'K':
  case 'F':
  case 'R':
  case 'Q':
  case 'S':
  case 'x':
  case 'y':
  case 'z':
  case 'a':
  case 'b':
  case 'c':
  case 'u':
  case 'v':
  case 'w':
  case 'i':
  case 'j':
  case 'k':
  case 'f':
  case 'r':
  case 's':
  case 'q':
  case 'E':
    return make_double_address(parse_double(s));
  case 'G':
  case 'H':
  case 'M':
  case 'N':
  case 'O':
  case 'T':
  case 'P':
  case 'D':
  case 'L':
  case 'g':
  case 'h':
  case 'm':
  case 'n':
  case 'o':
  case 't':
  case 'p':
  case 'd':
  case 'l':
    return make_int_address(parse_int(s));
  default:
    cout << "Invalid c = " << c << endl;
    cout << "Invalid c as int = " << ((int)c) << endl;
    cout << "Is EOF? " << (((int)c) == EOF) << endl;
    assert(false);
  }
}

string parse_line_comment_with_delimiter(string sc, parse_stream<string> &s) {
  string text = "";
  while (s.chars_left()) {
    text += s.next();
    s++;
  }

  return text;
}

string parse_comment_with_delimiters(char sc, char ec, parse_state &s) {
  int depth = 0;
  string text = "";
  do {
    if (s.next() == sc) {
      depth++;
      text += s.next();
    } else if (s.next() == ec) {
      depth--;
      text += s.next();
    } else {
      text += s.next();
    }
    s++;
  } while (s.chars_left() && depth > 0);

  return text;
}

string parse_comment_with_delimiters(string sc, string ec,
                                     parse_stream<string> &s) {
  int depth = 0;
  string text = "";
  do {
    if (s.next() == sc) {
      depth++;
    } else if (s.next() == ec) {
      depth--;
    } else {
      text += s.next();
    }
    s++;
  } while (s.chars_left() && depth > 0);

  return text;
}

chunk parse_isolated_word(parse_stream<string> &s) {
  assert(s.chars_left());
  assert(s.next().size() == 1);

  char c = s.next()[0];
  s++;

  return make_isolated_word(c);
}

chunk parse_word_address(parse_stream<string> &s) {
  assert(s.chars_left());
  assert(s.next().size() == 1);

  char c = s.next()[0];
  s++;

  addr a = parse_address(c, s);
  return chunk(c, a);
}

chunk parse_chunk(parse_stream<string> &s) {
  assert(s.chars_left());

  if (s.next()[0] == '[') {
    string cs = s.next();
    s++;
    return chunk('[', ']', cs.substr(1, cs.size() - 2));
  } else if (s.next()[0] == '(') {

    string cs = s.next();
    s++;
    return chunk('(', ')', cs.substr(1, cs.size() - 2));

  } else if (s.next() == "%") {
    s++;
    return make_percent_chunk();
  } else if (s.next() == ";") {
    s++;
    string cs = parse_line_comment_with_delimiter(";", s);
    return chunk(';', ';', cs);
  } else {
    string next_next = *(s.remaining() + 1);

    if (!is_num_char(next_next[0])) {
      return parse_isolated_word(s);
    }
    return parse_word_address(s);
  }
}

bool parse_slash(parse_state &s) {
  if (s.next() == '/') {
    s++;
    return true;
  }

  return false;
}

bool is_slash(const string &s) {
  if (s.size() != 1) {
    return false;
  }

  return s[0] == '/';
}

bool parse_slash(parse_stream<string> &s) {
  if (is_slash(s.next())) {
    s++;
    return true;
  }

  return false;
}

std::pair<bool, int> parse_line_number(parse_stream<string> &s) {
  if (s.next() == "N") {
    s++;

    int ln = parse_int(s);

    return std::make_pair(true, ln);
  }
  return std::make_pair(false, -1);
}

block parse_tokens(const std::vector<string> &tokens) {

  if (tokens.size() == 0) {
    return block(false, {});
  }

  parse_stream<string> s(tokens);
  vector<chunk> chunks;
  bool is_slashed = parse_slash(s);

  std::pair<bool, int> line_no = parse_line_number(s);

  while (s.chars_left()) {
    chunk ch = parse_chunk(s);
    chunks.push_back(ch);
  }

  if (line_no.first) {
    return block(line_no.second, is_slashed, chunks);
  } else {
    return block(is_slashed, chunks);
  }
}

std::string digit_string(parse_state &s) {
  string num_str = "";

  while (s.chars_left() && is_num_char(s.next())) {
    num_str += s.next();
    s++;
  }

  return num_str;
}

std::string lex_token(parse_state &s) {
  assert(s.chars_left());

  char c = s.next();
  string next_token = "";

  if (is_num_char(c)) {
    return digit_string(s);
  }

  switch (c) {

  case '(':
    return parse_comment_with_delimiters('(', ')', s);

  case '[':
    return parse_comment_with_delimiters('[', ']', s);

  case ')':
    assert(false);

  case ']':
    assert(false);

  default:
    next_token = c;
    s++;
    return next_token;
  }
}

std::vector<std::string> lex_block(const std::string &block_text) {
  parse_state s(block_text);

  vector<string> tokens;

  ignore_whitespace(s);

  while (s.chars_left()) {
    ignore_whitespace(s);

    if (s.chars_left()) {
      string token = lex_token(s);
      tokens.push_back(token);
    }
  }

  return tokens;
}

vector<block> lex_gprog(const string &str) {
  vector<block> blocks;
  string::const_iterator line_start = str.begin();
  string::const_iterator line_end;

  while (line_start < str.end()) {
    line_end = find(line_start, str.end(), '\n');
    string line(line_start, line_end);

    if (line.size() > 0) {

      vector<string> line_tokens = lex_block(line);

      block b = parse_tokens(line_tokens);
      blocks.push_back(b);
    }

    line_start += line.size() + 1;
  }
  return blocks;
}

gcode_program parse_gcode(const std::string &program_text) {
  auto blocks = lex_gprog(program_text);
  return gcode_program(blocks);
}

gcode_program parse_gcode_saving_block_text(const std::string &program_text) {
  auto blocks = lex_gprog(program_text);
  for (auto &b : blocks) {
    b.set_debug_text();
  }
  return gcode_program(blocks);
}

ostream &operator<<(ostream &stream, const chunk &ic) {
  ic.print(stream);
  return stream;
}

ostream &operator<<(ostream &stream, const block &block) {
  block.print(stream);
  return stream;
}

ostream &operator<<(ostream &stream, const gcode_program &program) {
  for (auto b : program) {
    stream << b << endl;
  }
  return stream;
}

addr make_int_address(const int i) {
  addr_value v;
  v.int_val = i;
  return addr{ADDRESS_TYPE_INTEGER, v};
}

addr make_double_address(const double i) {
  addr_value v;
  v.dbl_val = i;
  return addr{ADDRESS_TYPE_DOUBLE, v};
}

chunk make_word_int(const char c, const int i) {
  addr int_address = make_int_address(i);
  return chunk(c, int_address);
}

chunk make_word_double(const char c, const double i) {
  addr double_addr = make_double_address(i);
  return chunk(c, double_addr);
}

bool operator==(const chunk &l, const chunk &r) { return l.equals(r); }

bool operator!=(const chunk &l, const chunk &r) { return !(l == r); }

chunk make_comment(const char start_delim, const char end_delim,
                   const std::string &comment_text) {
  return chunk(start_delim, end_delim, comment_text);
}

chunk make_percent_chunk() { return chunk(); }

chunk make_isolated_word(const char c) { return chunk(c); }

} // namespace gpr

static double hypot_f(double x, double y) { return (sqrt(x * x + y * y)); }

const double motor_steps_per_rev = 3200.0;
const double encoder_steps_per_rev = 2880.0;

const double lead_screw_tpi_x = 10.0;
const double lead_screw_tpi_y = 12.0;
const double lead_screw_tpi_z = 12.0;

inline double mm_to_step_x(double mm) {
	mm *= motor_steps_per_rev / (25.4 / lead_screw_tpi_x); return mm;
}

inline double mm_to_step_z(double mm) {
	mm *= motor_steps_per_rev / (25.4 / lead_screw_tpi_z); return mm;
}

inline double mm_to_step_y(double mm) {
	mm *= motor_steps_per_rev / (25.4 / lead_screw_tpi_y); return mm;
}

static int32_t current_block = 0;
static int32_t current_inter = 0;
static int32_t current_bytes = 0;

static int32_t prev_z_mul = 0;
static int32_t prev_z_div = 0;
static int32_t prev_x_mul = 0;
static int32_t prev_x_div = 0;
static int32_t prev_d_mul = 0;
static int32_t prev_d_div = 0;

static uint32_t gcd_impl(uint32_t u, uint32_t v)
{
    int shift;
    if (u == 0) return v;
    if (v == 0) return u;
    shift = __builtin_ctz(u | v);
    u >>= __builtin_ctz(u);
    do {
        v >>= __builtin_ctz(v);
        if (u > v) {
            uint32_t t = v;
            v = u;
            u = t;
        }
        v = v - u;
    } while (v != 0);
    return u << shift;
}

static int32_t gcd(int32_t u, int32_t v) {
    return int32_t(gcd_impl(abs(u),abs(v)));
}

static void output_intermediate(
			double target_x_pos,
			double target_y_pos,
			double target_z_pos,
			double feed_rate,
			bool wait_for_index) {

  static double prev_target_x_pos = 0;
  static double prev_target_y_pos = 0;
  static double prev_target_z_pos = 0;
  
  double target_feed_x = 0;
  double target_feed_y = 0;
  double target_feed_z = 0;

  int axis_mode = 0;

  double sign_x = -1.0;
  if (target_x_pos >= prev_target_x_pos) {
	sign_x = 1.0;
  }

  double sign_z = -1.0;
  if (target_z_pos >= prev_target_z_pos) {
	sign_z = 1.0;
  }

  double dx = fabs(prev_target_x_pos - target_x_pos);
  double dz = fabs(prev_target_z_pos - target_z_pos);
  double dy = fabs(prev_target_x_pos - target_x_pos);

  if (prev_target_x_pos != target_x_pos &&
	  prev_target_z_pos != target_z_pos) {
	  if (dx > dz) {
		  axis_mode = 0;
		  target_feed_y = 0;
		  target_feed_x = feed_rate * (dx / hypot_f(dx, dz)) * sign_x;
		  target_feed_z = fabs(target_feed_x) * (dz / dx) * sign_z;
	  } else {
		  axis_mode = 2;
		  target_feed_y = 0;
		  target_feed_z = feed_rate * (dz / hypot_f(dx, dz)) * sign_z;
		  target_feed_x = fabs(target_feed_z) * (dx / dz) * sign_x; 
	  }
  } else if (prev_target_x_pos != target_x_pos) {
	  axis_mode = 0;
	  target_feed_x = feed_rate * sign_x;
	  target_feed_y = 0;
	  target_feed_z = 0;
  } else if (prev_target_z_pos != target_z_pos) {
	  axis_mode = 2;
	  target_feed_x = 0;
	  target_feed_y = 0;
	  target_feed_z = feed_rate * sign_z;
  }
  int32_t target_pos = 0;
  switch(axis_mode) {
  	case 0: {
	  target_pos = int32_t(mm_to_step_x(target_x_pos));
  	} break;
  	case 1: {
	  target_pos = int32_t(mm_to_step_y(target_y_pos));
  	} break;
  	case 2: {
	  target_pos = int32_t(mm_to_step_z(target_z_pos));
  	} break;
  }
  
  int32_t z_mul = int32_t(target_feed_z * lead_screw_tpi_z * motor_steps_per_rev * 10.0);
  int32_t z_div = int32_t(25.4 * encoder_steps_per_rev * 10.0);
  int32_t x_mul = int32_t(target_feed_x * lead_screw_tpi_x * motor_steps_per_rev * 10.0);
  int32_t x_div = int32_t(25.4 * encoder_steps_per_rev * 10.0);
  int32_t d_mul = int32_t(target_feed_y * lead_screw_tpi_y * motor_steps_per_rev * 10.0);
  int32_t d_div = int32_t(25.4 * encoder_steps_per_rev * 10.0);

  int32_t gcd_z = gcd(z_mul, z_div);
  z_mul /= gcd_z;
  z_div /= gcd_z;

  int32_t gcd_x = gcd(x_mul, x_div);
  x_mul /= gcd_x;
  x_div /= gcd_x;

  int32_t gcd_d = gcd(d_mul, d_div);
  d_mul /= gcd_d;
  d_div /= gcd_d;
  
  current_bytes += 2;
 
  if (abs(target_pos) >> 25) {
	  current_bytes += 4;
  } else if (abs(target_pos)>> 17) {
	  current_bytes += 3;
  } else if (abs(target_pos)) {
	  current_bytes += 2;
  }
 
  if (prev_z_mul != z_mul || prev_z_div != z_div ) {
	  if ((abs(z_mul) | abs(z_div)) >> 25) {
		  current_bytes += 8;
	  } else if ((abs(z_mul) | abs(z_div)) >> 17) {
		  current_bytes += 6;
	  } else if ((abs(z_mul) | abs(z_div))) {
		  current_bytes += 4;
	  }
  }
  if (prev_x_mul != x_mul || prev_x_div != x_div ) {
	  if ((abs(x_mul) | abs(x_div)) >> 25) {
		  current_bytes += 8;
	  } else if ((abs(x_mul) | abs(x_div)) >> 17) {
		  current_bytes += 6;
	  } else if ((abs(x_mul) | abs(x_div))) {
		  current_bytes += 4;
	  }
  }
  if (prev_d_mul != d_mul || prev_d_div != d_div ) {
	  if ((abs(d_mul) | abs(d_div)) >> 25) {
		  current_bytes += 8;
	  } else if ((abs(d_mul) | abs(d_div)) >> 17) {
		  current_bytes += 6;
	  } else if ((abs(d_mul) | abs(d_div))) {
		  current_bytes += 4;
	  }
  }
  
  prev_z_mul = z_mul;
  prev_z_div = z_div;
  prev_x_mul = x_mul;
  prev_x_div = x_div;
  prev_d_mul = d_mul;
  prev_d_div = d_div;
			
  char str[256] = { 0 };
  sprintf(str,"%04d %04d %05d %c"
  			  "%08x"
			  "%08x" "%08x"
			  "%08x" "%08x"
			  "%08x" "%08x"
			  "%08x",
			  current_block,
			  current_inter++,
			  current_bytes,
			  (axis_mode == 0) ? 'X' : ((axis_mode == 1) ? 'D' : 'Z'),
			  target_pos,
			  (z_mul && dz) ? z_mul : 0,
			  (z_mul && dz) ? z_div : 0,
			  (x_mul && dx) ? x_mul : 0,
			  (x_mul && dx) ? x_div : 0,
			  (d_mul && dy) ? d_mul : 0,
			  (d_mul && dy) ? d_div : 0,
			  wait_for_index?1:0);
  
  cout << str << endl;

  prev_target_x_pos = target_x_pos;
  prev_target_y_pos = target_y_pos;
  prev_target_z_pos = target_z_pos;

}

static void arc_xz(double position_x, double position_y, double position_z,
                   double target_x, double target_y, double target_z,
                   double offset_x, double offset_y, double offset_z,
                   double feed_rate, 
                   bool is_clockwise_arc) {

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7
#define ARC_TOLERANCE 0.002
#define N_ARC_CORRECTION 12

  double radius = hypot_f(offset_x, offset_z);

  double center_x = position_x + offset_x;
  double center_z = position_z + offset_z;

  double r_x = -offset_x; // Radius vector from center to current location
  double r_z = -offset_z;

  double rt_x = target_x - center_x;
  double rt_z = target_z - center_z;

  double angular_travel =
      atan2(r_x * rt_z - r_z * rt_x, r_x * rt_x + r_z * rt_z);

  if (is_clockwise_arc) {
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel -= 2 * M_PI;
    }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel += 2 * M_PI;
    }
  }

  uint32_t segments =
      uint32_t(floor(fabs(0.5 * angular_travel * radius) /
                     sqrt(ARC_TOLERANCE * (2 * radius - ARC_TOLERANCE))));

  if (segments) {
    double theta_per_segment = angular_travel / segments;
    double linear_per_segment = (target_y - position_y) / segments;
    double cos_T = 2.0 - theta_per_segment * theta_per_segment;
    double sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
    cos_T *= 0.5;
    double sin_Ti;
    double cos_Ti;
    double r_axisi;
    uint8_t count = 0;
    for (int i = 1; i < segments; i++) {
      if (count < N_ARC_CORRECTION) {
        r_axisi = r_x * sin_T + r_z * cos_T;
        r_x = r_x * cos_T - r_z * sin_T;
        r_z = r_axisi;
        count++;
      } else {
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_x = -offset_x * cos_Ti + offset_z * sin_Ti;
        r_z = -offset_x * sin_Ti - offset_z * cos_Ti;
        count = 0;
      }
      // Update arc_target location
      position_x = center_x + r_x;
      position_z = center_z + r_z;

      position_y += linear_per_segment;

      output_intermediate(position_x, position_y, position_z, feed_rate, false);
      //printf("cutti x:%+9.6f y:%+9.6f z:%+9.6f f:%+9.6f\n", position_x, position_y, position_z, feed_rate);
    }
  }
  
  output_intermediate(target_x, target_y, target_z, feed_rate, false);
  //printf("cutti x:%+9.6f y:%+9.6f z:%+9.6f f:%+9.6f\n", target_x, target_y, target_z, feed_rate);
}

int main() {
  std::ifstream t("1001.nc");
  std::string file_contents((std::istreambuf_iterator<char>(t)),
                            std::istreambuf_iterator<char>());

  gpr::gcode_program p = gpr::parse_gcode(file_contents);

  bool execute_move = false;

  double prev_target_x_pos = 0;
  double prev_target_y_pos = 0;
  double prev_target_z_pos = 0;

  int move_mode = 0;

  double target_x_pos = 0;
  double target_y_pos = 0;
  double target_z_pos = 0;

  double target_i_rad = 0;
  double target_k_rad = 0;

  double target_feed = 0;

  double coord_multiplier = 1.0;
  double feed_multiplier = 1.0;

  current_inter = 0;
  current_bytes = 0;
  for (int c = 0; c < p.num_blocks(); c++) {
  	current_block = c;
    const gpr::block &b = p.get_block(c);
    for (int d = 0; d < b.size(); d++) {
      const gpr::chunk &c = b.get_chunk(d);
      switch (c.tp()) {
      case gpr::CHUNK_TYPE_WORD_ADDRESS: {
        switch (c.get_word()) {
        case 'G': {
          switch (c.get_address().int_value()) {
          case 0: {
            move_mode = 0;
          } break;
          case 1: {
            move_mode = 1;
          } break;
          case 2: {
            move_mode = 2;
          } break;
          case 3: {
            move_mode = 3;
          } break;
          case 32: {
            move_mode = 4;
          } break;
          case 18: {
            // nop, assume this as the default
          } break;
          case 21: {
            coord_multiplier = 1.0;
          } break;
          case 22: {
            coord_multiplier = 25.4;
          } break;
          case 50: {
            // ignore
          } break;
          case 53: {
            // ignore
          } break;
          case 54: {
            // ignore
          } break;
          case 97: {
            // ignore
          } break;
          case 94:
          case 98: {
            feed_multiplier =
                1.0 / 64.0; // convert to feed per rev based on ~500rpm
          } break;
          case 95:
          case 99: {
            feed_multiplier = 1.0;
          } break;
          default: {
            cout << c << endl;
          } break;
          }
        } break;
        case 'M': {
          switch (c.get_address().int_value()) {
          case 0: {
          } break;
          case 1: {
          } break;
          case 3: {
          } break;
          case 8: {
          } break;
          case 9: {
          } break;
          case 22: {
          } break;
          case 30: {
          } break;
          case 31: {
          } break;
          default: {
            cout << c << endl;
          } break;
          }
        } break;
        case 'Y': {
          target_y_pos = c.get_address().double_value() * coord_multiplier;
          execute_move = true;
        } break;
        case 'X': {
          target_x_pos = c.get_address().double_value() * coord_multiplier;
          execute_move = true;
        } break;
        case 'Z': {
          target_z_pos = c.get_address().double_value() * coord_multiplier;
          execute_move = true;
        } break;
        case 'F': {
          target_feed = c.get_address().double_value() * feed_multiplier;
        } break;
        case 'I': {
          target_i_rad = c.get_address().double_value() * coord_multiplier;
        } break;
        case 'K': {
          target_k_rad = c.get_address().double_value() * coord_multiplier;
        } break;
        case 'T': {
          cout << "wait for tool!" << endl;
        } break;
        case 'S': {
          // nop
        } break;
        case 'O': {
          // nop
        } break;
        default: {
          cout << c << endl;
        } break;
        }
      } break;
      case gpr::CHUNK_TYPE_WORD: {
        cout << "fubar!" << endl;
      } break;
      default: {
      } break;
      }
    }
    if (execute_move) {
    
      execute_move = false;
      
      bool rapid = move_mode == 0;
      
      double target_feed_x = 0;
      double target_feed_y = 0;
      double target_feed_z = 0;

      switch (move_mode) {
      case 0: {
      	output_intermediate(target_x_pos, target_y_pos, target_z_pos, rapid ? 1.0 : target_feed, false);
        //printf("rapid x:%+9.6f y:%0+9.6f z:%+9.6f\n", target_x_pos, target_y_pos, target_z_pos);
      } break;
      case 1: {
      	output_intermediate(target_x_pos, target_y_pos, target_z_pos, rapid ? 1.0 : target_feed, false);
        //printf("cutti x:%+9.6f y:%+9.6f z:%+9.6f f:%+9.6f\n", target_x_pos, target_y_pos, target_z_pos, target_feed);
      } break;
      case 2: {
        arc_xz(prev_target_x_pos, prev_target_y_pos, prev_target_z_pos,
               target_x_pos, target_y_pos, target_z_pos,
               target_i_rad, prev_target_y_pos, target_k_rad,
               rapid ? 1.0 : target_feed, false);
         //printf("radcw x:%+9.6f y:%+9.6f z:%+9.6f i:%+9.6f k:%+9.6f f:%+9.6f\n", target_x_pos, target_y_pos, target_z_pos, target_i_rad, target_k_rad, target_feed);
      } break;
      case 3: {
        arc_xz(prev_target_x_pos, prev_target_y_pos, prev_target_z_pos,
               target_x_pos, target_y_pos, target_z_pos,
               target_i_rad, prev_target_y_pos, target_k_rad,
               rapid ? 1.0 : target_feed, true);
         //printf("radcc x:%+9.6f y:%+9.6f z:%+9.6f i:%+9.6f k:%+9.6f f:%+9.6f\n", target_x_pos, target_y_pos, target_z_pos, target_i_rad, target_k_rad, target_feed);
      } break;
      case 4: {
      	output_intermediate(target_x_pos, target_y_pos, target_z_pos, rapid ? 1.0 : target_feed, true);
        //printf("threa x:%+9.6f y:%+9.6f z:%+9.6f f:%+9.6f\n", target_x_pos, target_y_pos, target_z_pos, target_feed);
      } break;
      }

      prev_target_x_pos = target_x_pos;
      prev_target_y_pos = target_y_pos;
      prev_target_z_pos = target_z_pos;
    }
  }
}
