#include <cstdlib>
#include <getoptions.h>
#include <iostream>
#include <vector>

using namespace std;

char * ahrs = "initial ahrs string";
float dt = 0.0;
int verbose = 0;

void 
help()
{
	cerr << "Usage: \n"
		 << "-h | --help \n"
		 << "-v | --verbose \n"
		 << "-a | --ahrs string \n"
		 << "-d | --dt time \n" 
		 << endl;
		 
	
	exit(-1);
}

void
version()
{
	cout << "Version unknown " << endl;
	exit(-1);
}
int 
main(int argc, char **argv)
{
	cout << "This is a demo of getoptions library" << endl;
	cout << "do: check_getoptions --help to see example "<< endl;
	int rc = getoptions( &argc, &argv,
        "h|?|help&",           help,
        "V|version&",          version,
        "v|verbose+",          &verbose,
        "a|ahrs=s",            &ahrs,
        "d|dt=f",              &dt,
        0
   );
   if (rc) {	cout << "getoptions returned " << "\n"; }
   cout << "Settings: \n"
   		<< " verbose: " << verbose << "\n"
   		<< " ahrs: " << ahrs << "\n"
   		<< " dt: " << dt << "\n" 
   		<< endl;

	vector <int> v;

}