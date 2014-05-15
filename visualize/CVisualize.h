#pragma once

#include <vector>

/*
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

//#include "opencv2/nonfree/features2d.hpp"
*/

#include "CHelper.h"

using namespace cv;
using namespace std;

class CVisualize
{

public:

	CVisualize()
	{

	}

	static void convertMatchesToPoint2f(vector<KeyPoint> &keypointsCur, vector<KeyPoint> &keypointsRef, vector<DMatch> &matches, vector<Point2f> &points_cur, vector<Point2f> &points_ref)
	{
		points_cur.clear();
		points_ref.clear();

		for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
		{
			float x = keypointsRef[it->trainIdx].pt.x; // Get the position of ref keypoints
			float y = keypointsRef[it->trainIdx].pt.y;
			points_ref.push_back(cv::Point2f(x, y));
			x = keypointsCur[it->queryIdx].pt.x; // Get the position of cur keypoints
			y = keypointsCur[it->queryIdx].pt.y;
			points_cur.push_back(cv::Point2f(x, y));
		}
	}

	static void convertPoint2fToKeypoints(vector<Point2f> PointsRef, vector<Point2f> PointsCur,
										  vector<KeyPoint> &KeypointsRef, vector<KeyPoint> &KeypointsCur)
	{
		KeypointsCur.clear();
		KeypointsRef.clear();

		for (int i = 0; i < PointsRef.size(); i++)
		{
			KeyPoint MyKeypointRef, MyKeypointCur;
			MyKeypointRef.pt.x = PointsRef[i].x;
			MyKeypointRef.pt.y = PointsRef[i].y;
			MyKeypointCur.pt.x = PointsCur[i].x;
			MyKeypointCur.pt.y = PointsCur[i].y;
			KeypointsRef.push_back(MyKeypointRef);
			KeypointsCur.push_back(MyKeypointCur);
		}
	}

	//Dummy matches vector, usefull for visualizeMatchingResults function
	static vector<DMatch> createDummyMatchVector(int NumKeypoints)
	{
		vector<DMatch> DummyMatchVector;
		for (int i = 0; i < NumKeypoints; i++)
		{
			//create a dummy match vector so we can reuse functions
			DMatch Match;
			Match.trainIdx = i; //each 2x1 column builds a matching pair
			Match.queryIdx = i;
			DummyMatchVector.push_back(Match);
		}
		return DummyMatchVector;
	}

	static void drawArrow(Mat Img, Point S, Point E, Scalar Color = GREEN, int LineSize = 2)
	{
		float spinSize  = 15;
		float headstyle = 7;

		//draw arrow line
		line(Img, S, E, Color, LineSize, CV_AA); //top

		//draw arrow head
		double angle;
		angle = atan2( (double) S.y - E.y, (double) S.x - E.x );

		S.x = (int) (E.x + spinSize * cos(angle + 3.1416 / headstyle));
		S.y = (int) (E.y + spinSize * sin(angle + 3.1416 / headstyle));
		line( Img, S, E, Color, LineSize, CV_AA, 0 );

		S.x = (int) (E.x + spinSize * cos(angle - 3.1416 / headstyle));
		S.y = (int) (E.y + spinSize * sin(angle - 3.1416 / headstyle));
		line( Img, S, E, Color, LineSize, CV_AA, 0 );
	}

	static void visualizeMatchingResults(Mat &CurY, Mat &RefY,
			vector<KeyPoint>& keypointsCur, vector<KeyPoint>& keypointsRef,
			vector<DMatch>& Matches_STEST, vector<DMatch>& Matches_GTEST,
			Mat &H,
			Mat &Result_FeatureMap, Mat &Result_DiffImage)
	{
		bool isWithText = false;
		vector<Point2f> points_ref, points_cur;

		//---------------------------------------------------------
		//DRAWING CUR FRAME WITH ALL KEYPOINTS
		//---------------------------------------------------------
		for (vector<KeyPoint>::const_iterator it = keypointsCur.begin(); it != keypointsCur.end(); ++it)
		{
			float x = it->pt.x; // Get the position of cur keypoints
			float y = it->pt.y;
			points_cur.push_back(cv::Point2f(x, y));
		}
		Mat cur_keypoints;
		Mat tmp11 = CurY.clone();
		Mat pointers11[] =
		{ tmp11, tmp11, tmp11 };
		merge(pointers11, 3, cur_keypoints); //now its RGB!
		for (int i = 0; i < points_cur.size(); i++)
			circle(cur_keypoints, points_cur[i], 2, cv::Scalar(0, 255, 0), 1);

		if (isWithText)
			helpme::writeText(cur_keypoints, "All detected keypoints");

		//---------------------------------------------------------
		//DRAWING CUR FRAME, WITH MATCHES AFTER STEST
		//---------------------------------------------------------
		points_ref.clear();
		points_cur.clear();
		convertMatchesToPoint2f(keypointsCur, keypointsRef, Matches_STEST, points_cur, points_ref);
		Mat cur_stest;
		Mat tmp1 = CurY.clone();
		Mat pointers1[] =
		{ tmp1, tmp1, tmp1 };
		merge(pointers1, 3, cur_stest); //now its RGB!
		for (int i = 0; i < points_cur.size(); i++)
		{
			circle(cur_stest, points_cur[i], 2, cv::Scalar(0, 255, 0), 1);
			line(cur_stest, points_cur[i], points_ref[i], cv::Scalar(0, 0, 255), 1);
		}
		if (isWithText)
			helpme::writeText(cur_stest, "After symmetry tests");

		//---------------------------------------------------------
		//DRAWING REF FRAME, WITH MATCHES AFTER S-TEST
		//H is used to esimate keypoints in cur frame
		//see perspectiveTransform() for details
		//---------------------------------------------------------
		Mat ref;
		Mat tmp2 = RefY.clone();
		Mat pointers2[] =
		{ tmp2, tmp2, tmp2 };
		merge(pointers2, 3, ref); //now its RGB!
		Mat xi(points_ref);
		//H.assignTo(H, CV_32F); //perspectiveTransform seems to check this internally
		Mat xi_tick;
		perspectiveTransform(xi, xi_tick, H);
		vector<Point2f> transformedpoints = xi_tick;
		for (int i = 0; i < points_ref.size(); i++)
		{
			circle(ref, points_ref[i], 2, cv::Scalar(0, 255, 0), 1);
			line(ref, points_ref[i], transformedpoints[i], cv::Scalar(0, 0, 255), 1);
		}

		//---------------------------------------------------------
		//DRAWING CUR FRAME, WITH MATCHES AFTER GTEST
		//---------------------------------------------------------
		convertMatchesToPoint2f(keypointsCur, keypointsRef, Matches_GTEST, points_cur, points_ref);
		Mat cur_gtest;
		Mat tmp3 = CurY.clone();
		Mat pointers3[] =
		{ tmp3, tmp3, tmp3 };
		merge(pointers3, 3, cur_gtest); //now its RGB!
		for (int i = 0; i < points_cur.size(); i++)
		{
			circle(cur_gtest, points_cur[i], 2, cv::Scalar(0, 255, 0), 1);
			line(cur_gtest, points_cur[i], points_ref[i], cv::Scalar(0, 0, 255), 1);
		}
		if (isWithText)
			helpme::writeText(cur_gtest, "After geometry tests");

		//---------------------------------------------------------
		//DRAWING DIFFERENCE BETWEEN WARPED FRAME AND REF FRAME
		//---------------------------------------------------------
		Mat Diff = getDifferenceImage(RefY, CurY, H);
		if (isWithText)
			helpme::writeText(Diff, "Difference after warping");

		//---------------------------------------------------------
		// MAKE RESULTS AVAILABLE TO THE CALLER
		//---------------------------------------------------------
		Result_FeatureMap = cur_stest.clone();
		Result_FeatureMap.push_back(cur_gtest.clone());
		Result_DiffImage = Diff.clone();
	}

	static Mat getDifferenceImage(Mat RefY, Mat CurY, Mat H)
	{
		Mat WarpedRef;
		Mat Rotated;

		warpPerspective(RefY.clone(), // input image
				WarpedRef,		 // output image
				H,				 // homography
				RefY.size(), INTER_CUBIC);
		Rotated = CurY.clone();

		Rotated = Rotated / 2;
		WarpedRef = WarpedRef / 2;
		Mat tmp5 = (128 + Rotated) - WarpedRef;
		Mat pointers[] =
		{ tmp5, tmp5, tmp5 };
		Mat diff;
		merge(pointers, 3, diff);
		//if (isWithText)
		//	helpme::writeText(diff, "Difference after warping");
		return diff;
	}

};

