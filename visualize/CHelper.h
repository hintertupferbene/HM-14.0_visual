#ifndef CHELPER_H
#define CHELPER_H

#include <vector>
#include <opencv2/opencv.hpp>
//#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include "OpenEXR/half.h"
#include "Defines.h"

using namespace std;
using namespace cv;
using namespace boost;

#define HEVC_CODE

#define RESCALING_INTERPOLATION INTER_CUBIC // choose: INTER_AREA //INTER_LANCZOS4 //INTER_LINEAR //INTER_CUBIC

#define FOR(i,length) for(int i=0; i<(int)(length); i++)

namespace helpme
{
Mat invertImage(Mat Image);
void convertToCV_8UC3(Mat &Image);

//makes sure we have access to these functions in HM and in CManip* and CShow*,
//but do not need to have CWarper.h know anything about HM
//complicated fella, handle with care
#ifdef HEVC_CODE

//»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»
// ALT-GR + y / x
//«««««««««««««««««««««««««««««««««««««««««««

inline void reduceToHalfPrecision(Mat D)
{
  //only prepared for double Mats!
  if(D.depth() != CV_64F || D.dims != 2)
    THROW_ERROR;

  //half *halfvals = new halfvals[]
  int rows = D.size().height;
  int cols = D.size().width;
  for(int y = 0; y < rows ; y++)
    for(int x = 0; x < cols ; x++)
    {
      half tmp = D.at<double>(y, x);
      cout << "Half: " << double(tmp) << endl;
      D.at<double>(y, x) = double(tmp);
    }
}

//------------------------------------------------------------------
// copy_PicYuv2Mat
//------------------------------------------------------------------
inline void copy_PicYuv2Mat(TComPicYuv* pcPicYuv, Mat& Image)
{
	int iWidth = pcPicYuv->getWidth();
	int iHeight = pcPicYuv->getHeight();

	//Luminance
	Pel* pSrc = pcPicYuv->getBufY();
	UInt iSrcStride = pcPicYuv->getStride(); //stride = width + 2*lumamargin
	pSrc += pcPicYuv->getLumaMargin(); //getting rid of first margin in the upper left
	pSrc += pcPicYuv->getLumaMargin() * iSrcStride;
	Mat Y = Mat(iHeight, iWidth, CV_16SC1, Scalar(0));
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pDst = Y.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pSrc = pSrc + iSrcStride;
	}

	//Chrominance Cb
	iWidth = iWidth / 2;
	iHeight = iHeight / 2;
	iSrcStride = iWidth + (pcPicYuv->getChromaMargin() * 2);
	pSrc = pcPicYuv->getBufU();
	pSrc += pcPicYuv->getChromaMargin(); //getting rid of first margin in the upper left
	pSrc += pcPicYuv->getChromaMargin() * iSrcStride;
	Mat tmp = Mat(iHeight, iWidth, CV_16SC1, Scalar(0));
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pDst = tmp.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pSrc = pSrc + iSrcStride;
	}
	Mat Cb;
	cv::resize(tmp, Cb, Size(iWidth * 2, iHeight * 2), 0, 0, RESCALING_INTERPOLATION);

	//Chrominance Cr
	pSrc = pcPicYuv->getBufV();
	pSrc += pcPicYuv->getChromaMargin(); //getting rid of first margin in the upper left
	pSrc += pcPicYuv->getChromaMargin() * iSrcStride;
	tmp = Mat(iHeight, iWidth, CV_16SC1, Scalar(0));
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pDst = tmp.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pSrc = pSrc + iSrcStride;
	}
	Mat Cr;
	cv::resize(tmp, Cr, Size(iWidth * 2, iHeight * 2), 0, 0, RESCALING_INTERPOLATION);

	//Merging to color image
	Mat tmp2;
	Mat pointers[] =
	{ Y, Cb, Cr };
	merge(pointers, 3, tmp2); //now its RGB!
	Image = tmp2.clone();

	//note: image has 16 bit bitdepth!
	return;
}

//------------------------------------------------------------------
// copy_Mat2PicYuv
//------------------------------------------------------------------
inline void copy_Mat2PicYuv(Mat& Image, TComPicYuv* pcPicYuv)
{
	int iWidth = Image.size().width;
	int iHeight = Image.size().height;

	vector<Mat> splitted;
	split(Image, splitted);
	Mat Y, Cb, Cr;
	Y = splitted[0];
	resize(splitted[1], Cb, cv::Size(iWidth / 2, iHeight / 2), 0, 0, RESCALING_INTERPOLATION);
	resize(splitted[2], Cr, cv::Size(iWidth / 2, iHeight / 2), 0, 0, RESCALING_INTERPOLATION);

	//Luminance
	Pel* pDst = pcPicYuv->getBufY();
	UInt iSrcStride = pcPicYuv->getStride(); //stride = width + 2*lumamargin
	pDst += pcPicYuv->getLumaMargin(); //getting rid of first margin in the upper left
	pDst += pcPicYuv->getLumaMargin() * iSrcStride;
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pSrc = Y.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pDst = pDst + iSrcStride;
	}

	//Chrominance Cb
	iWidth = iWidth / 2;
	iHeight = iHeight / 2;
	iSrcStride = iWidth + (pcPicYuv->getChromaMargin() * 2);
	pDst = pcPicYuv->getBufU();
	pDst += pcPicYuv->getChromaMargin(); //getting rid of first margin in the upper left
	pDst += pcPicYuv->getChromaMargin() * iSrcStride;
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pSrc = Cb.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pDst = pDst + iSrcStride;
	}

	//Chrominance Cr
	iSrcStride = iWidth + (pcPicYuv->getChromaMargin() * 2);
	pDst = pcPicYuv->getBufV();
	pDst += pcPicYuv->getChromaMargin(); //getting rid of first margin in the upper left
	pDst += pcPicYuv->getChromaMargin() * iSrcStride;
	for (int y = 0; y < iHeight; y++)
	{
		unsigned char* pSrc = Cr.ptr<unsigned char>(y);
		::memcpy(pDst, pSrc, sizeof(Pel) * iWidth);
		pDst = pDst + iSrcStride;
	}

	return;
}

#endif

//----------------------------------------------
// INITIALIZING TMP DIRECTORY FOR EACH DEBUG RUN
//----------------------------------------------
inline void initializeTmpDir()
{
	static bool FirstCall = true;
	if (FirstCall)
	{
		system("rm -rf ./tmp");
		system("mkdir ./tmp");
		cout << "rm -rf ./tmp: Tmp dir cleaned\n";
		FirstCall = false;
	}
}

//----------------------------------------------
// INITIALIZING JPG DIRECTORY FOR EACH DEBUG RUN
//----------------------------------------------
inline void initializeJPGDir()
{
	static bool FirstCall = true;
	if (FirstCall)
	{
		system("rm -rf ./jpg");
		system("mkdir ./jpg");
		cout << "rm -rf ./jpg: JPG dir cleaned\n";
		FirstCall = false;
	}
}

//------------------------------------------------------------------
// writeJPG
//------------------------------------------------------------------
enum ENumerationType
{
	NO_ENUM, AUTO_ENUM, MANUAL_ENUM
};
inline void writeJPG(Mat image, string Identifier, ENumerationType NumerationType = AUTO_ENUM, int ManualNumber = 1000)
{
//	static bool FirstCall = true;
//	if (FirstCall)
//	{
//		system("rm -rf ./jpg");
//		cout << "rm -rf ./jpg: JPG dir cleaned\n";
//		FirstCall = false;
//	}

	if(SKIPJPEGS)
	  return;

	const char *Format = "jpg";
	//const char *Format = "png";

	if(IS_INVERTING)
		image = invertImage(image);

	system("mkdir -p ./jpg");
	int Quality = JPG_QUALITY; //100 = best
	vector<int> params;
	params.push_back(CV_IMWRITE_JPEG_QUALITY);
	params.push_back(Quality);
	char outjpg[100];
	static int AutoNumber = 0;
	if (NumerationType == NO_ENUM)
	{
		sprintf(outjpg, "jpg/%s.%s", Identifier.c_str(), Format);
	}
	else if (NumerationType == AUTO_ENUM)
	{
		sprintf(outjpg, "jpg/%s_%.4d.%s", Identifier.c_str(), AutoNumber, Format);
		AutoNumber++;
	}
	else // MANUAL NUMBER
	{
		sprintf(outjpg, "jpg/%s_%.4d.%s", Identifier.c_str(), ManualNumber, Format);
	}

	cout << "Wrote: " << outjpg << endl;
	imwrite(outjpg, image, params);
}

inline void writeJPG_Direkt(Mat image, string FileName)
{
  if(SKIPJPEGS)
    return;

  //format given by provided filename!
  int Quality = JPG_QUALITY; //100 = best
  vector<int> params;
  params.push_back(CV_IMWRITE_JPEG_QUALITY);
  params.push_back(Quality);

  //INVERTING
  if (IS_INVERTING)
    image = invertImage(image);


  imwrite(FileName, image, params);
  cout << "Wrote: " << FileName << endl;

}

//------------------------------------------------------------------
// writeText
//------------------------------------------------------------------
inline void writeText(Mat Matrix, string text, Scalar color = Scalar(0, 0, 200), Point textOrg = Point(3, 3.0), double fontScale = 0.5, int thickness = 2)
{
	//int fontFace = FONT_HERSHEY_SIMPLEX;
	int fontFace = FONT_HERSHEY_PLAIN;

	int baseline = 0;
	Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;

	//new line for each new line, using boost
	char_separator<char> sep("\n");
	tokenizer<char_separator<char> > tokens(text, sep);
	BOOST_FOREACH (const string& entry, tokens){
	textOrg.y += 20*fontScale;
	putText(Matrix, entry, textOrg, fontFace, fontScale,
			color, thickness, 3);
}

}

//------------------------------------------------------------------
// convert YCrCb to RGB, return copy
//------------------------------------------------------------------
inline Mat getRGB(Mat &YCrCb)
{
	Mat RGB;
	cv::cvtColor(YCrCb, RGB, CV_YCrCb2RGB);
	return RGB;
}

inline Mat getRGB(Mat &Y, Mat &U, Mat &V)
{
	Mat YUV;
	Mat pointers[] =
	{ Y, U, V };
	merge(pointers, 3, YUV); //now its RGB!

	Mat RGB;
	cv::cvtColor(YUV, RGB, CV_YCrCb2RGB);
	return RGB;
}

inline void convertToCV_8UC3(Mat &Image)
{
	if (Image.channels() != 3) //grayscale to color
	{
		Mat tmp;
		Mat pointers[] =
		{ Image, Image, Image };
		merge(pointers, 3, tmp); //now its RGB!
		Image = tmp.clone();
	}

	if (Image.depth() != CV_8U)
		Image.convertTo(Image, CV_8U);
}

//------------------------------------------------------------------
// invertImage
//------------------------------------------------------------------
inline Mat invertImage(Mat Image)
{
	Mat cloned = Image.clone();
	cloned.convertTo(Image, CV_8U); //making sure elvis is in da house

	Mat inverted = cloned.clone(); //reserving space
	inverted = Scalar(255, 255, 255);
	inverted -= cloned;
	return inverted;
}

//------------------------------------------------------------------
// getMatrixType
//------------------------------------------------------------------
inline void getMatrixType(cv::Mat M, const char* string)
{
	if (string != NULL)
		std::cout << "----Type of matrix " << string << "----" << endl;
	else
		std::cout << "----Type of matrix ----" << endl;

	cv::Size sizeM = M.size();
	int rows = sizeM.height;
	int cols = sizeM.width;
	int depth = M.depth();
	char depthstr[50];
	switch (depth)
	{
	case CV_8U:
		sprintf(depthstr, "CV_8U");
		break;
	case CV_8S:
		sprintf(depthstr, "CV_8S");
		break;
	case CV_16U:
		sprintf(depthstr, "CV_16U");
		break;
	case CV_16S:
		sprintf(depthstr, "CV_16S");
		break;
	case CV_32S:
		sprintf(depthstr, "CV_32S");
		break;
	case CV_32F:
		sprintf(depthstr, "CV_32F");
		break;
	case CV_64F:
		sprintf(depthstr, "CV_64F");
		break;
	default:
		sprintf(depthstr, "USERTYPE");
		break;
	}

	std::cout << "Cols x Rows = " << cols << "x" << rows << ", ";
	std::cout << "Type = " << depthstr << ", Dims = " << M.dims << ", NumChannels = " << M.channels() << "\n";
}

//------------------------------------------------------------------
// printMatrix
//------------------------------------------------------------------
inline string printMatrix(cv::Mat M, const char* string)
{
	stringstream sout(stringstream::in | stringstream::out);
	cv::Size sizeM = M.size();
	int rows = sizeM.height;
	int cols = sizeM.width;

	if (string != NULL)
		sout << "----Matrix " << string << "----" << endl;
	else
		sout << "----Matrix ----" << endl;

	//getMatrixType(M, string);
	sout << setw(5) << fixed; // << setprecision( 3 );// << right << fixed;

	if (M.channels() == 1) //2D matrix, like H or F
	{
		for (int row = 0; row < rows; ++row)
		{
			for (int col = 0; col < cols; ++col)
			{
				if (M.depth() == 6)
					sout << (double) M.at<double>(row, col) << " ";
				else if (M.depth() == 5)
					sout << (double) M.at<float>(row, col) << " ";
				else
					assert(0);
			}
			sout << endl;
		}
		sout << flush;
	}
	else //uh oh, some strange 3D points matrix, originating from vector<Point2f> (OpenCV likes those!)
	{
		//indexing separate channels is tough!
		//also see: /HOMES/springer/Forschungsarbeit/z_Sonstiges/OpenCV/Channel-Indexing unter OpenCV.pdf
		vector<Mat> splitted;
		split(M, splitted);
		for (int ch = 0; ch < M.channels(); ch++)
		{
			sout << "--------Channel " << ch << ": ----------" << endl;
			for (int row = 0; row < rows; ++row)
			{
				for (int col = 0; col < cols; ++col)
				{
					if (col > 100)
						continue;

					if (M.depth() == 6)
						sout << (double) splitted[ch].at<double>(row, col) << " ";
					else if (M.depth() == 5)
						sout << (double) splitted[ch].at<float>(row, col) << " ";
					else
						assert(0);
					sout << "\t";
				}
				//std::sout << endl;
			}
			sout << endl;
		}
		sout << flush;
	}

	//cout << sout.str();
	return sout.str();
}

//------------------------------------------------------------------
// Image Matrix
//------------------------------------------------------------------
class CImageMatrix
{
public:
	CImageMatrix(int NumRows, int NumCols, int DimX, int DimY) //;, int Type = CV_8UC3)
	{
		this->DimX = DimX; //Dimension of one image
		this->DimY = DimY;
		this->NumRows = NumRows;
		this->NumCols = NumCols;
		this->Final = Mat(NumRows * DimY, NumCols * DimX, CV_8UC3, Scalar(0, 0, 0));
	}


	//use DimY and DimX to set multiple tiles, leave as is to set single tile
	void fillPosition(Mat Image, int Row, int Col, char *text = NULL, int CustomDimY = -1, int CustomDimX = -1)
	{
		Mat Cloned = Image.clone();
		if (performCheck(Cloned, Row, Col) == false)
			return;

		if (CustomDimY == -1 || CustomDimX == -1) //not given
		{
			CustomDimY = this->DimY;
			CustomDimX = this->DimX;
		}

		if(text != NULL)
		{
			helpme::writeText(Cloned, text, Scalar(0, 255, 0), Point(20, 0), 2, 4);
		}

		Mat Roi;
		Roi = Final(Rect(Col * DimX, Row * DimY, CustomDimX, CustomDimY));

		int d = 2;
		Mat ClonedRoi = Cloned(Rect(d, d, CustomDimX-d, CustomDimY-d));
		Mat Interior = ClonedRoi.clone();
		Cloned = Scalar(0,0,0); //all black
		Interior.copyTo(ClonedRoi); //now we have a border!

		Cloned.copyTo(Roi);
	}


	bool performCheck(Mat &Image, int Row, int Col)
	{
		if (Row >= NumRows || Col >= NumCols)
		{
			//cout << "WARNING CImageMatrix: Row >= NumRows || Col >= NumCols)" << endl;
			return false;
		}

		convertToCV_8UC3(Image);

		return true;
	}

	void drawBox(int Row, int Col)
	{
		Mat roi = Final(Rect(Col * DimX, Row * DimY, DimX, DimY));

		int LineSize = 6;
		cv::Scalar BoarderColor = cv::Scalar(0, 0, 255);
		line(roi, Point(0, 0), Point(DimX - 1, 0), BoarderColor, LineSize); //top
		line(roi, Point(0, 0), Point(0, DimY - 1), BoarderColor, LineSize); //left
		line(roi, Point(DimX - 1, DimY - 1), Point(DimX - 1, 0), BoarderColor, LineSize); //right
		line(roi, Point(DimX - 1, DimY - 1), Point(0, DimY - 1), BoarderColor, LineSize); //right
	}

	Mat Final;
	int NumRows, NumCols;
	int DimY, DimX;
};

}


class CYuvReader
{
public:

	CYuvReader(const char *file, unsigned short w, unsigned short h)
	{
		width = w;
		height = h;
		pSourcefile = fopen(file, "rb");

		if(pSourcefile == NULL)
		{
			cout << "Couldn't open file " << file << endl;
			endOfFile = 0;
		}
		else
		{
			fseek(pSourcefile, 0, SEEK_END);
			endOfFile = ftell(pSourcefile);
			fseek(pSourcefile, 0, SEEK_SET);
		}
	}

	Mat getMatY()
	{
		if(pSourcefile == NULL)
			return Mat();

		int pos = ftell(pSourcefile);
		assert(pos < endOfFile);
		unsigned char *samples = new unsigned char[width*height];
		int length = fread(samples, sizeof(unsigned char), width*height, pSourcefile);
		assert(length==width*height);
		Mat tmp = Mat(height, width, CV_8UC1,samples);
		Mat M = tmp.clone();
		fseek(pSourcefile, width*height/2, SEEK_CUR);
		delete[] samples;
		return M;
	}

	void getMatYUV444(Mat &Y, Mat &U, Mat &V, int DIMX, int DIMY)
	{
		if(pSourcefile == NULL)
			assert(0);
		int pos = ftell(pSourcefile);
		assert(pos < endOfFile);

		unsigned char *y = (unsigned char*) malloc (DIMX*DIMY);
		unsigned char *u = (unsigned char*) malloc (DIMX*DIMY/4);
		unsigned char *v = (unsigned char*) malloc (DIMX*DIMY/4);

	    fread(y, sizeof(unsigned char), width*height, pSourcefile);
	    fread(u, sizeof(unsigned char), width*height/4, pSourcefile);
	    fread(v, sizeof(unsigned char), width*height/4, pSourcefile);

	    Mat tmp;
	    tmp = Mat(height, width, CV_8UC1,y);
	    Y = tmp.clone();
	    tmp = Mat(height/2, width/2, CV_8UC1,u);
	    cv::resize(tmp, U, Size(width, height));
	    tmp = Mat(height/2, width/2, CV_8UC1,v);
	    cv::resize(tmp, V, Size(width, height));

	    delete[] y;
	    delete[] u;
	    delete[] v;
	}

	void getMatYUV420(Mat &Y, Mat &U, Mat &V, int DIMX, int DIMY)
	{
		if(pSourcefile == NULL)
			assert(0);
		int pos = ftell(pSourcefile);
		assert(pos < endOfFile);

		unsigned char *y = (unsigned char*) malloc (DIMX*DIMY);
		unsigned char *u = (unsigned char*) malloc (DIMX*DIMY/4);
		unsigned char *v = (unsigned char*) malloc (DIMX*DIMY/4);

		int BytesRead[3];
		BytesRead[0] = fread(y, sizeof(unsigned char), width*height, pSourcefile);
		BytesRead[1] = fread(u, sizeof(unsigned char), width*height/4, pSourcefile);
		BytesRead[2] = fread(v, sizeof(unsigned char), width*height/4, pSourcefile);

		if(BytesRead[0] != width*height || BytesRead[1] != width*height/4 || BytesRead[2] != width*height/4) //ERROR!
		{
			cout << "ERROR: " << BytesRead[0] << " " << BytesRead[1] << " " << BytesRead[2] << endl;
			assert(0);
		}

	    Mat tmp;
	    tmp = Mat(height, width, CV_8UC1,y);
	    Y = tmp.clone();
	    tmp = Mat(height/2, width/2, CV_8UC1,u);
	    U = tmp.clone();
	    tmp = Mat(height/2, width/2, CV_8UC1,v);
	    V = tmp.clone();

	    delete[] y;
	    delete[] u;
	    delete[] v;
	}

	int getNumFrames()
	{
		return endOfFile/(width*height*1.5);
	}

	void skipFrames(int SkipFrames)
	{
		fseek(pSourcefile, SkipFrames*width*height*1.5, SEEK_SET);
	}



	unsigned int endOfFile;
	unsigned short height;
	unsigned short width;
	FILE *pSourcefile;
};

#endif
