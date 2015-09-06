// example of how to read trajectories stored line-by-line in CSV file format, convert the to trajectories and play them

ifstream file ( "file.csv" ); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
string value;
while ( file.good() )
{
     getline ( file, value, ',' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
     cout << string( value, 1, value.length()-2 ); // display value removing the first and the last character from it
}
