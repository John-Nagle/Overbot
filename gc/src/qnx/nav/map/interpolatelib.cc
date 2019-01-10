//
//	playbacktest.cc  --  map updating based on canned file data
//
//	Test only,
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
#include <stdio.h>
#include <vector>
#include "logprint.h"
#include "interpolatelib.h"


//
//	dumptimestamp  --  dump a 64-bit timestamp
//
//	Input is nanoseconds since epoch.
//
static void dumptimestamp(const char* msg, const uint64_t timestamp)
{
	timespec ts;														// as timespec
	nsec2timespec(&ts,timestamp);						// convert to seconds
	time_t tstime = ts.tv_sec;									// into time type
	tm localtm;
	localtime_r(&tstime,&localtm);							// convert time
	char s[100];
	const char* format = "%F %T";							// yyyy-mm-dd hh:mm:ss 
	strftime(s,sizeof(s),format,&localtm);					// edit time
	logprintf("%s %s (%lld ns.)\n",msg,s,timestamp);	// print
}





// Get the 3x3 matrix from the 4x4 matrix -- this function should exist in the algebra3.h file. 
void getmat3frommat4 ( const mat4& origmat4, mat3& newmat3)
{
	newmat3[0][0] = origmat4[0][0];
	newmat3[0][1] = origmat4[0][1];
	newmat3[0][2] = origmat4[0][2];
	
	newmat3[1][0] = origmat4[1][0];
	newmat3[1][1] = origmat4[1][1];
	newmat3[1][2] = origmat4[1][2];	
	
	newmat3[2][0] = origmat4[2][0];
	newmat3[2][1] = origmat4[2][1];
	newmat3[2][2] = origmat4[2][2];
}

//Normalize the 3x3 matrix. -- this function should exist in the algebra3.h file. 
void normalizemat3 (mat3& origmat)
{
	origmat[0].normalize();
	origmat[1].normalize();
	origmat[2].normalize();
}

// Copy the contents on the tempmat into the new matrix
void copymat3tomat4 ( const mat3& tempmat, mat4&newmat )
{
	newmat[0][0] = tempmat[0][0];
	newmat[0][1] = tempmat[0][1];
	newmat[0][2] = tempmat[0][2];
	
	newmat[1][0] = tempmat[1][0];
	newmat[1][1] = tempmat[1][1];
	newmat[1][2] = tempmat[1][2];
	
	newmat[2][0] = tempmat[2][0];
	newmat[2][1] = tempmat[2][1];
	newmat[2][2] = tempmat[2][2];
}

void printmat ( const mat3& inputmat)
{
	int i, j;	
	for( i = 0 ; i < 3 ; i++ )
	{
		for ( j = 0 ; j < 3 ; j++ )
			printf ( "%f	         ", inputmat[i][j] );
	
		printf("\n");		
	}
}

void printmat ( const mat4& inputmat)
{
	int i, j;
	
	for( i = 0 ; i < 4 ; i++ )
	{
		for ( j = 0 ; j < 4 ; j++ )
			printf ( "%f	          ", inputmat[i][j] );
	
		printf("\n");		
	}
}


//
//	 Interpolates a 4x4 matrix using LERP given time at the 2 boundary points 
bool interpolateLERP( const mat4& prevmat, uint64_t prevtime, const mat4& nextmat, uint64_t nexttime,
	uint64_t wantedtime, mat4& wantedmat)
{
	const uint64_t	ACCEPTABLE_TIME_DIFF 				= 100;		// time is specified in ns, 100 is a very small time diff.
	const float		DOTRESULT_OPP_VECTORS 			= -1.0;  		// result of dot productof 2 opposite vectors.
	const float		ACCEPTABLE_DOTERROR				= 0.002;		// acceptable error for dot product between 2 vectors.
	
	mat3 orientprevmat, orientnextmat;
	mat3 orientnewmat;		
	
	uint64_t diffmatprev, diffmatnext;
	
	diffmatprev = wantedtime - prevtime;
	diffmatnext = nexttime - wantedtime;	
	
	//Handle special cases

	// Case 1: Out of bounds case: When wantedtime is either greater than nexttime or less then prevtime
	// 				Cannot interpolate because LERPfract would result is a negative number.
	// 				Find out closest in time and return that value instead.
	if ( ( prevtime >= wantedtime )  || ( nexttime <= wantedtime  ) )
	{
	//	cout <<  "Out of bounds case.\n ";
	//	cout << " Prevtime =" << prevtime <<  ", Wantedtime =" << wantedtime << ", Nextime=" << nexttime ;
		if ( prevtime == 0 || diffmatnext == 0 )
		{
	//		cout << "\nReturning nextmat.\n";
			wantedmat = nextmat;
		}
		else
		{
	//		cout << "\nReturning prevmat.\n";
			wantedmat = prevmat; 
		}
		return (true);	
	}
	
	// Next let's calculate the orientation matrix for each of the 2 matrices.	
	getmat3frommat4( prevmat, orientprevmat );
	getmat3frommat4( nextmat, orientnextmat );
	
//	orientprevmat.transpose();
//	orientnextmat.transpose();
	
	diffmatprev = wantedtime - prevtime;
	diffmatnext = nexttime - wantedtime;	

	//Case 2: Handle the case where the 2 matrices have the same time. Avoid divide by zero case in LERP function.
	if  (nexttime == prevtime )
	{
		wantedmat = prevmat;			
		printf ("\nHere nexttime is equal to prevtime \n ");
	}
	else
	{
		uint64_t	diff		= 	wantedtime - prevtime; 
		uint64_t	delta		=	nexttime - prevtime;
	
		float	LERPfract 	= 		(float)diff / (float)delta;
			
		wantedmat = identity3D(); 					// Make the wantedmat an identity matrix - start with a clean matrix.
	
		// Case 3 : When the wantedtime is very close or equal to prevtime -- in this case, prevmat can be used for orientation.
		if ( diffmatprev <= ACCEPTABLE_TIME_DIFF )
		{
			printf ("\nHere  time diff is ALMOST equal - giving prev mat\n\n ");
			orientnewmat = orientprevmat;
		}	
		else 
		{
			// Case 4 : When the wantedtime is very close or equal to nexttime -- in this case, nextmat can be used for orientation
			if ( diffmatnext <= ACCEPTABLE_TIME_DIFF )
			{
				printf ("\n\nHere  time diff is ALMOST equal - giving next mat\n\n ");
				orientnewmat = orientnextmat;
			}
			else 
			{
				// Case 5 : When two vectors are opposite of each other(Theta =180 deg.) , 
				//				finding a interpolated matrix is not impossible because 
				// 				they cancel each other out. Dot product is defined as a.b = |a|*|b|*cos(theta).  
				//				Since |a| and |b| will be equal to 1, the dot product will be equal to cos(theta). 
				//				Therefore, we have to eliminate cases where the dot product is cos(180) = -1
				//				In this case, return the vector that is closest in time to the wantedtime.
				//				for all other cases perform normal interpolation.
	
				for (int i=0;i < 3 ; i++ )
				{
						float dotproduct;
						dotproduct = orientprevmat[i] * orientnextmat[i];
						if ( (dotproduct >=  (DOTRESULT_OPP_VECTORS - ACCEPTABLE_DOTERROR ) ) && (dotproduct <= ( DOTRESULT_OPP_VECTORS + ACCEPTABLE_DOTERROR) ) )
						{
								// these two vectors cannot be interpolated because they are opp. to each other.
								// simply return one that is closest in time.	
								//Check which of the 2 vectors is closest in time to the wanted vector.
						//		printf ("\n\nOpposite vectors - Forget LERP. \n\n");

								if (diffmatprev <= diffmatnext )
								{
										orientnewmat[i] = orientprevmat[i];
								}	
								else
								{
									orientnewmat[i] = orientnextmat[i];
								}						
						}
						else
						{ 
		//						printf ("\nHurray -- actually performing ORIENTATION LERP now \n");
								// Perform Normal interpolation of the 2 vectors							
								orientnewmat[i] = (1-LERPfract)*orientprevmat[i] + LERPfract*orientnextmat[i];   // apply Linear Interpolation formula.
						}
						
						//normalize the vector right here 
						orientnewmat[i].normalize();
				} // end of for loop 
			}	// end of case 5 and else of case 4.
		} // else of case 3

	//	normalizemat3 ( orientnewmat );							// normalize the orientation matrix -  orientnewmat.
	
		// Copy normalized orientation matrix into wantedmat
		copymat3tomat4 ( orientnewmat, wantedmat ); 
			 
		// Now we have the correct interpolated and normalized orientation matrix.
		// Next we must inteprolate position using the LERP formula. 
		for (int i=0;i < 3 ; i++ )
		{
			wantedmat[i][3] = (1-LERPfract)*prevmat[i][3] + LERPfract*nextmat[i][3];   // apply Linear Interpolation formula.
		}
		
	} // end of else if (prevtime==nexttime)
	
//	cout <<"Prev=" << prevtime <<  ", Want=" << wantedtime<< ", Next="<< nexttime ;
//	printf ("\nPrevious Mat :\n");
//	printmat (prevmat);
//	printf ("\nNext Mat :\n");
//	printmat (nextmat);
//	printf ("\n The result mat is : \n");
//	printmat (wantedmat);
//	printf ("\n=======================================");

	return (true);
}