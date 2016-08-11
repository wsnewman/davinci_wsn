#include <playfile_reader/playfile_record.h>

// Read a single line.
std::istream& operator >> (std::istream & ins, record_t & record){
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	std::string line;
	getline(ins, line);

	// now we'll use a stringstream to separate the fields out of the line
	std::stringstream ss(line);
	std::string field;
	while (getline(ss, field, ',')){
		// for each field we wish to convert it to a double
		// (since we require that the CSV contains nothing but floating-point values)
		std::stringstream fs(field);
		double f = 0.0;// (default value is 0.0)
		fs >> f;

		// add the newly-converted field to the end of the record
		record.push_back(f);
	}
	
	return ins;
}

// Read a list of CSV records.
std::istream& operator >> (std::istream & ins, data_t & data){
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record){
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}
