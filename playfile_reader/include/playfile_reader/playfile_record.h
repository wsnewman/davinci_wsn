#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

typedef std::vector <double> record_t;
typedef std::vector <record_t> data_t;

std::istream& operator >> (std::istream & ins, record_t & record);

std::istream& operator >> (std::istream & ins, data_t & data);
