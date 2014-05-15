#pragma once

#include <sys/time.h>

#define SKIPJPEGS 0


// CAREFULL! ----------------------------------------
#define FEATURE_TRANSFORM SURF_SURF  	//SIFT_SIFT //SURF_SURF // ORB_ORB
#define IS_COMPENSATING_TRANSPARENCY 0 	//compensation of transparency
#define CODING_SKIP_HACK 0				//hack encoder to skip any encoding (= frame pass-through)
// --------------------------------------------------

//SPIRAL
#define ADAPTIVE_SPIRAL 1
#define MAXNUM_ORBFEATURESINBLOCK 60
#define B_DIMX (DIMX / BSIZE)
#define B_DIMY (DIMY / BSIZE)
#define MAXNUMBLOCKS (B_DIMX*B_DIMY/4) //never specify more than B_DIMX*B_DIMY!
#define MINNUMBLOCKS 6

#define VERSION_2  //specific version of Spiral Estimation

#ifdef VERSION_1 //no empty rows
#define SPANS {1,1,2,2}
#define SPANSTEPSIZE 2
#define FIX 0
#endif

#ifdef VERSION_2 //empty rows in x and y
#define SPANS {1,2,3,4}
#define SPANSTEPSIZE 4
#define FIX 0
#endif

#ifdef VERSION_3 //empty rows only in y
#define SPANS {1,2,2,4}
#define SPANSTEPSIZE 2
#define FIX 1
#endif


//#define VERBOSE
//#define VERBOSE2
//#define REPROERR_AKT

#define IS_USING_HALF 0

//Inverting images?
#define IS_INVERTING false

// VISUALIZATION, PRINTF
//#define SPIRAL_ARM_STEPPING //hack to create JPG for each spiral arm
#define ACTIVATE_CSHOW_CLASSES
#define VIS_MATCHINGRESULTS
#define WITH_DIFF_IMG
#define WRITE_CURRGB
#define COUT(instruction) //cout instruction
#define COUT_MATRIX(instruction) //cout instruction   //print all sorts of H matrices

//JPG quality
#define JPG_QUALITY 90

//setting "nice level" (priority scheduling, lower=higher prio)
#define PROCESS_PRIORITY 20

//Encoder continually places current CTB index here
extern int CurrentCTU;

//Encoder continually places current POC index here
extern int CurrentPOC;

//only these POCs are visualized and exported to XML
//const int NumPOCS = 4; //number of observed POCs
//const int ObsPOCS[4] = {0,1,2,3}; //must be equal to NumPOCS!
const int NumPOCS = 6; //number of observed POCs
const int ObsPOCS[6] = {0,1,2,3,4,5}; //must be equal to NumPOCS!

//only these CTUs are visualized and exported to XML
const int NumObsCTUs = 1; //number of observed CTBs
const int ObsCTU[3] = {26}; //must be equal to NumObsCTBs!  /{17, 43, 69}

inline bool isInterestingPOC()
{
  //HACK!!!
  return true;

  for(int i = 0; i < NumPOCS; i++)
  {
     int POC = ObsPOCS[i];
     if(CurrentPOC == POC)
       return true;
  }
  return false;
}

inline bool isInterestingCTU()
{
  //lets leave if this POC isn't interesting in the first place
  if(not isInterestingPOC())
    return false;

  //look if current CTU index is interesting
  for(int i = 0; i < NumObsCTUs; i++)
  {
     int Index = ObsCTU[i];
     if(CurrentCTU == Index)
       return true;
  }

  return false;
}

#define THROW_ERROR {std::cout << "Hey, that's crap, in " << __FILE__ << ", " << __LINE__ << endl << flush; exit(-1);}

// ----- COMMENT SECOND PART TO DEACTIVATE ------------------------------------
//define DOXML(instruction) if(isInterestingCTU()) {instruction;}
//define DOVIZ(instruction) {instruction;}
// ----------------------------------------------------------------------------

#define INIT_TIMER struct timeval tp; \
        double sec, usec, start, end, Seconds;

#define START_TIMER \
		cout << "START.. "; \
		gettimeofday( &tp, NULL ); \
        sec = static_cast<double>( tp.tv_sec ); \
        usec = static_cast<double>( tp.tv_usec )/1E6; \
        start = sec + usec;

#define STOP_TIMER \
		gettimeofday( &tp, NULL ); \
        sec = static_cast<double>( tp.tv_sec ); \
        usec = static_cast<double>( tp.tv_usec )/1E6; \
        end = sec + usec; \
        Seconds = end - start; \
        cout << " ..STOP: "; \
        cout << Seconds << " secs" << endl << flush;

//So-You-Want-A-Basic-Color-Code-Huh
#define WHITE Scalar(255,255,255)
#define GRAY  Scalar(128,128,128)
#define BLACK Scalar(0,0,0)
#define YELLOW Scalar(0,255,255)
#define GREEN Scalar(0,255,0)
#define RED Scalar(0,0,255)
#define BLUE Scalar(255,0,0)
#define GREEN_2 Scalar(9,241,9)
#define DEEPSKYBLUE Scalar(255,191,0)
#define DARKORANGE Scalar(0,140,255)
#define OLIVE Scalar(0x00,0x80,0x80)
#define MEDIUMPURPLE Scalar(0xD8,0x70,0x93)
#define MAGENTA Scalar(255,0,255);

//DEPRECATED, LEAVE ON, CONTROLLED VIA CMAKELISTS.TXT!
#define IS_WARPING 0

//DELETE LATER
//NOW SET IN CMAKELISTS
//#define BSIZE (64) //block size
//#define DIMX 800
//#define DIMY 480
//#define DIMX 1920
//#define DIMY 1080
//Center 2D+3D, MZ
//#define CENTERX 493
//#define CENTERY 338
//Center 3D, no MZ
//#define CENTERX 400
//#define CENTERY 338

//Safety measures
#if RDCURVES == 1 && CODING_SKIP_HACK == 1
#error "CODING_SKIP_HACK is active in an RD measurement environment!"
#endif
