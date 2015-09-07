// example of how to read a joint-space trajectory stored line-by-line in CSV file format, convert to trajectory and play play it
//specialized for dual-PSM davinci; assumes fixed order for joints 1-7 of psm_one, joints 1-7 of psm_two, and arrival time, in seconds.
//each line must contain all 15 values (in fixed order), separated by commas
#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.
istream& operator >> ( istream& ins, record_t& record )
  {
  // make sure that the returned record contains only the stuff we read now
  record.clear();

  // read the entire line into a string (a CSV record is terminated by a newline)
  string line;
  getline( ins, line );

  // now we'll use a stringstream to separate the fields out of the line
  stringstream ss( line );
  string field;
  while (getline( ss, field, ',' ))
    {
    // for each field we wish to convert it to a double
    // (since we require that the CSV contains nothing but floating-point values)
    stringstream fs( field );
    double f = 0.0;  // (default value is 0.0)
    fs >> f;

    // add the newly-converted field to the end of the record
    record.push_back( f );
    }

  // Now we have read a single line, converted into a list of fields, converted the fields
  // from strings to doubles, and stored the results in the argument record, so
  // we just return the argument stream as required for this kind of input overload function.
  return ins;
  }

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.
istream& operator >> ( istream& ins, data_t& data )
  {
  // make sure that the returned data only contains the CSV data we read here
  data.clear();

  // For every record we can read from the file, append it to our resulting data
  record_t record;
  while (ins >> record)
    {
    data.push_back( record );
    }

  // Again, return the argument stream as required for this kind of input stream overload.
  return ins;  
  }

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
	//instantiate a DavinciJointPublisher object and pass in pointer to nodehandle for constructor to use
    DavinciJointPublisher davinciJointPublisher(nh);  
 
    //ROS_INFO("instantiating  forward solver and an ik_solver");
    //Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    //Davinci_IK_solver ik_solver;
    
    if (argc!=2) {
      ROS_INFO("argc= %d; missing file command-line argument; halting",argc);
    return 0;
    }
     //open the trajectory file:
     ifstream infile(argv[1]);
	if(!infile)			// file couldn't be opened
	{
		cerr << "Error: file could not be opened; halting" << endl;
		exit(1);
	}    

    // define a vector of desired joint displacements...w/o linkage redundancies
    //7'th angle is related to jaw opening--but not well handled yet
    //Vectorq7x1 q_vec,q_vec2;
    //vector <Vectorq7x1> q1_vecs,q2_vecs;
    //vector <double> arrival_times;

    // Here is the data we want.
    data_t data;

  // Here is the file containing the data. Read it into data.
  infile >> data;

  // Complain if something went wrong.
  if (!infile.eof())
    {
    cout << "error reading file!\n";
    return 1;
    }

  infile.close();

  // Otherwise, list some basic information about the file.
  cout << "CSV file contains " << data.size() << " records.\n";

  unsigned min_record_size = data[0].size();
  unsigned max_record_size = 0;
  for (unsigned n = 0; n < data.size(); n++) {
    if (max_record_size < data[ n ].size())
      max_record_size = data[ n ].size();
    if (min_record_size > data[ n ].size())
      min_record_size = data[ n ].size();    
  }
  if (max_record_size>15) {
      ROS_WARN("bad file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

  }
  if (min_record_size<15) {
            ROS_WARN("bad file");
    cout << "The smallest record has " << min_record_size << " fields.\n";
    return 1;

  }
  
  //data is valid; pack it up as a trajectory and ship it
  
  //cout << "The second field in the fourth record contains the value " << data[ 3 ][ 1 ] << ".\n";

  cout << "Good bye!\n";
  return 0;
  }


