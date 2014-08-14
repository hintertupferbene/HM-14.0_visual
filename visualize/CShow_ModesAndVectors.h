#pragma once

#include <iostream>
#include <iomanip>
#include <typeinfo>
#include <vector>

/*
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
*/

#define HEVC_CODE //activates certain code passages
#include "Defines.h"
#include "CHelper.h"

using namespace cv;
using namespace std;


#include "TLibCommon/TypeDef.h"

class CShow_ModesAndVectors
{

public:
  Mat Img;  //holds visualized image (no blending included)
  int Zoom; //Zoom of one CTB
  Point ImgDim;
  bool isInterestArea;
  bool isMarkingBlocks;
  bool isDrawingStoreUnits;
  bool isDrawingVectors;
  char text[500];
  int call; //keeping track of calls to this class
  vector<Point2f> KeypointsCur, KeypointsRef;
  vector<Point> MV_StartPoints;
  vector<Point> MV_EndPoints;

  //Interest area
  Point CTB_Span; //x and y span of observed LCUs (may be selection or all LCUs)
  Point CTB_UpperLeft; //from (inclusive)
  Point CTB_LowerRight; //to (exclusive)

  CShow_ModesAndVectors(TComPic* pcPic, int NumCTBs, char *VizFile, int CTBSize,
      bool isMarkingBlocks = true, bool isDrawingVectors = true )
  {
    //---------------------------------------------
    // CHANGE HERE
    //---------------------------------------------
    isInterestArea = false; // false means "full image"
    this->isDrawingVectors = isDrawingVectors;
    this->isMarkingBlocks = isMarkingBlocks;
    this->isDrawingStoreUnits = false;

    CTB_UpperLeft  = Point(0,2);
    CTB_LowerRight = Point(1,3);
    //---------------------------------------------

    call = 0;
    sprintf(text, "====%s(%d)====", "CShow_ModesAndVectors", call);
    int textlen = strlen(text);
    cout << text << endl;

    UInt WidthInLCUs  = pcPic->getPicSym()->getFrameWidthInCU();  //=13
    UInt HeightInLCUs = pcPic->getPicSym()->getFrameHeightInCU(); //=8

    if(isInterestArea) //only interest-area
    {
      Zoom = 64;
      CTB_Span = CTB_LowerRight - CTB_UpperLeft;
      ImgDim.x = CTB_Span.x*CTBSize*Zoom;
      ImgDim.y = CTB_Span.y*CTBSize*Zoom;
    }
    else //full image
    {
      Zoom = 3;
      CTB_UpperLeft  = Point(0,0);
      CTB_LowerRight = Point(WidthInLCUs, HeightInLCUs);
      CTB_Span = CTB_LowerRight - CTB_UpperLeft;
      ImgDim.x = (CTB_Span.x)*CTBSize*Zoom; //this is equal to 800x480 * Zoom
      ImgDim.y = (CTB_Span.y)*CTBSize*Zoom;
    }

    Img = Mat(ImgDim.y, ImgDim.x, CV_8UC3, BLACK);

    //---------------------------------------------
    //---------------------------------------------
    //---------------------------------------------
    // DRAWING EACH QUADTREE PARTITION
    //---------------------------------------------
    for ( UInt LCU_Index = 0; LCU_Index < NumCTBs; LCU_Index++ )
    {
      //---------------------------------------------
      // GETTING ROI ON ZOOMED CTU REGION
      //---------------------------------------------
      // note: we need to expand the roi by the zoom factor so we can draw text properly
      UInt uiWidthInLCUs  = pcPic->getPicSym()->getFrameWidthInCU();  //=13
      UInt uiHeightInLCUs = pcPic->getPicSym()->getFrameHeightInCU(); //=8
      Int iCUAddr     = pcPic->getCU( LCU_Index )->getAddr(); //probably the same than index!
      Point CTB_Pos  = Point(iCUAddr % uiWidthInLCUs, iCUAddr / uiWidthInLCUs);

      //don't dive into this CTB if it lies outside the interest area
      if( checkRange(CTB_Pos.x, true, 0, CTB_UpperLeft.x, CTB_LowerRight.x) == false ||
          checkRange(CTB_Pos.y, true, 0, CTB_UpperLeft.y, CTB_LowerRight.y) == false)
        continue;

      //CTB_Anc holds anchor (in pixels) relative to upper left corner of interest area!
      Point CTB_Anc = (CTB_Pos - CTB_UpperLeft) * CTBSize * Zoom;
      //CTB_Roi holds image ROI of the current CTB
      Mat CTB_Roi = Img(Rect(CTB_Anc.x, CTB_Anc.y, CTBSize*Zoom, CTBSize*Zoom));

      //---------------------------------------------
      // LETS DIVE INTO A RECURSION
      //---------------------------------------------
      TComDataCU* pcCU = pcPic->getCU( LCU_Index );
      xDrawCU( pcCU, 0, 0, CTB_Roi, CTB_Anc, CTBSize);

      //---------------------------------------------
      // FINALIZE: DRAW YELLOW CTB BORDER AND WRITE INDEX
      //---------------------------------------------
//      line(CTB_Roi, Point(0,0), Point(CTBSize*Zoom-1,0), YELLOW, 10); //top
//      line(CTB_Roi, Point(0,0), Point(0, CTBSize*Zoom-1), YELLOW, 10);//left
//      line(CTB_Roi, Point(CTBSize*Zoom-1,CTBSize*Zoom-1), Point(CTBSize*Zoom-1, 0), YELLOW, 10);//right
//      line(CTB_Roi, Point(CTBSize*Zoom-1,CTBSize*Zoom-1), Point(0, CTBSize*Zoom-1), YELLOW, 10);//right
      sprintf(text, "%d", LCU_Index);
      //helpme::writeText(CTB_Roi, text, YELLOW, Point(7,7), 1.0);
      helpme::writeText(CTB_Roi, text, BLACK, Point(7,7), 1.0);
    }
    //---------------------------------------------
    //---------------------------------------------
    //---------------------------------------------

    //---------------------------------------------
    // DRAWING THE MV'S WE COLLECTED
    //---------------------------------------------
    if(isDrawingVectors)
    {
      vector<Point>::const_iterator it1 = MV_StartPoints.begin();
      vector<Point>::const_iterator it2 = MV_EndPoints.begin();

      for( ; it1 != MV_StartPoints.end(); ++it1, ++it2)
      {
    	  Point S, E;
    	  S = *it1;
    	  E = *it2;
    	  int LineSize = 2;
    	  line(Img, S, E, GREEN_2, LineSize, CV_AA); //top

          double angle;                                                                           // Draws the spin of the arrow
          angle = atan2( (double) S.y - E.y, (double) S.x - E.x );

          double spinSize = 15;
          S.x = (int) (E.x + spinSize * cos(angle + 3.1416 / 7));
          S.y = (int) (E.y + spinSize * sin(angle + 3.1416 / 7));
          line( Img, S, E, GREEN_2, LineSize, CV_AA, 0 );

          S.x = (int) (E.x + spinSize * cos(angle - 3.1416 / 7));
          S.y = (int) (E.y + spinSize * sin(angle - 3.1416 / 7));
          line( Img, S, E, GREEN_2, LineSize, CV_AA, 0 );

      }

      //for( ; it1 != MV_StartPoints.end(); ++it1, ++it2)
        //line(Img, *it1, *it2, GREEN_2, 5, CV_AA); //top


    }

    //---------------------------------------------
    // MARKING ALL OBSERVED BLOCKS
    //---------------------------------------------
//    if(not isInterestArea)
//      markCTB (pcPic);

    //---------------------------------------------
    // BLENDING IN THE CUR FRAME
    //---------------------------------------------
    bool isBlending = true;
    Mat Blended;
    if(isBlending)
    {
      int DIMX = pcPic->getPicYuvRec()->getWidth();
      int DIMY = pcPic->getPicYuvRec()->getHeight();

      Mat Tmp;
      //helpme::copy_PicYuv2Mat(pcPic->getPicYuvOrg(), Tmp); // original image not available on decoder side!
      helpme::copy_PicYuv2Mat(pcPic->getPicYuvRec(), Tmp);
      helpme::convertToCV_8UC3(Tmp);
      Mat CurFrame = Mat(DIMY+100, DIMX+100, CV_8UC3, BLACK); //CTBs expand beyond image!
      Tmp.copyTo(CurFrame(Rect(0, 0, DIMX, DIMY)));
      cvtColor(CurFrame, CurFrame, CV_YCrCb2RGB);
      cvtColor(CurFrame, CurFrame, CV_RGB2GRAY);
      Mat pointers[] = { CurFrame, CurFrame, CurFrame };
      Mat Gray;
      merge(pointers, 3, Gray);
      Mat InterestArea = Gray(Rect(CTB_UpperLeft.x*CTBSize, CTB_UpperLeft.y*CTBSize, CTB_Span.x*CTBSize, CTB_Span.y*CTBSize));
      Mat Resized;
      cv::resize(InterestArea, Resized, Size(ImgDim.x, ImgDim.y),0,0, INTER_NEAREST);
      double alpha = 0.5;
      double beta =  1.0 - alpha;
      addWeighted( Img, alpha, Resized, beta, 0.0, Blended);
    }
    else //no blending
    {
      Blended = Img;
    }

    //---------------------------------------------
    // RENDERING TO JPG
    //---------------------------------------------
	sprintf(text, "./jpg/ModesAndVectors_Poc%04d.jpg", pcPic->getPOC());
	sprintf(VizFile, "ModesAndVectors_Poc%04d.jpg", pcPic->getPOC()); //so that caller knows name of file
    helpme::writeJPG_Direkt(Blended, text);

    //---------------------------------------------
    // OPENING CV-WINDOW
    //---------------------------------------------
    //Img = helpme::invertImage(Img);
    //namedWindow( "CShow_RDO_Results", CV_WINDOW_NORMAL );// Create a window for display.
//    if (pcPic->getPOC()==0){
//      imshow( "CShow_RDO_Results intra", Blended );                   // Show our image inside it.
//    }else if (pcPic->getPOC()==1){
//      imshow( "CShow_RDO_Results inter", Blended );                   // Show our image inside it.
//    }
//    waitKey(10);

    cout << string(textlen, '=') << endl; //==================
    call++;
  };



  void xDrawCU( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, Mat &CTB_Roi, Point &CTB_Anc, int CTBSize)
  {
//    if(pcCU->getPic()==0||pcCU->getPartitionSize(uiAbsZorderIdx)==SIZE_NONE)
//    {
//      return;
//    }
    TComPic* pcPic     = pcCU->getPic();
    UInt NumPartInCTB  = pcPic->getNumPartInCU();      //number of StorageUnits (4x4) in CTB
    UInt NumPartInCB = NumPartInCTB >> (uiDepth<<1); //number of StorageUnits (4x4) in current CB
    UInt NextCU_Increment   = NumPartInCB>>2; //increment (in StorageUnits) if CB is splitted further

    //if small block signals: way to go, not deep enough
    if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth ) //if upper left StorageUnit says "Final depth dude!"
    {
      for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=NextCU_Increment )
      {
      	//pcCU->getCUPelX() : get absolute pixel index if LCU(!) in frame
        UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
        UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
        if(    ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() )
            && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
        {
            xDrawCU( pcCU, uiAbsZorderIdx, uiDepth+1, CTB_Roi, CTB_Anc, CTBSize);
        }
      }
      return;
    }
    //we arrived at the final partition depth for this CU
    int offs = 1; //used for text line feed
    const char *PartitionSizeStrings[] =   {"SIZE_2Nx2N","SIZE_2NxN","SIZE_Nx2N", "SIZE_NxN",
                                            "SIZE_2NxnU","SIZE_2NxnD","SIZE_nLx2N","SIZE_nRx2N"};
    const char *FalseTrueStrings[] = {"false", "true"};

    //---------------------------------------------
    // GETTING SOME INITIAL INFOS
    //---------------------------------------------
    //getting the pcPic, POC and iCUAddr
    TComPic* rpcPic = pcCU->getPic();
    int POC = rpcPic->getPOC();
    Int iCUAddr     = pcCU->getAddr(); //CU raster index in this slice

    //getting picture height and width
    Int  y, iWidth, iHeight;
    TComPicYuv* pcPicYuv = rpcPic->getPicYuvRec();
    iWidth  = pcPicYuv->getWidth();
    iHeight = pcPicYuv->getHeight();

    //LCU width and height, position of LCU in picture
    UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();  //=13
    UInt uiHeightInLCUs = rpcPic->getPicSym()->getFrameHeightInCU(); //=8
    UInt uiCol=0, uiLin=0, uiSubStrm=0;

    //Asserting things (just making sure :)
    UInt uiTileCol;
    UInt uiTileStartLCU;
    UInt uiTileLCUX;
    UInt uiTileLCUY;
    UInt uiTileWidth;
    UInt uiTileHeight;
    uiTileCol = rpcPic->getPicSym()->getTileIdxMap(iCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
    uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(iCUAddr))->getFirstCUAddr();
    uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
    uiTileLCUY = uiTileStartLCU / uiWidthInLCUs;
    assert( uiTileCol == 0 and uiTileStartLCU == 0 and uiTileLCUX == 0 and uiTileLCUY == 0);
    uiTileWidth = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(iCUAddr))->getTileWidth();
    uiTileHeight = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(iCUAddr))->getTileHeight();
    //assert(uiWidthInLCUs  == 13 and uiTileWidth == 13); //making sure we have navigation sequences
    //assert(uiHeightInLCUs ==  8 and uiTileHeight == 8); //making sure we have navigation sequences

    //unused
    int uiCUPelX = pcCU->getCUPelX();
    int uiCUPelY = pcCU->getCUPelY();

    //from Zscan to Raster, then getting x and y indices
    int RasterPartIdx  = g_auiZscanToRaster[uiAbsZorderIdx];
    Point AbsPart = Point(RasterPartIdx % (CTBSize/4), RasterPartIdx / (CTBSize/4)); // 64/4 = partition ticks (=StorageUnits-Ticks) in a LCU

    //---------------------------------------------
    // ANCHOR AND DIM INFO OF CU INSIDE THE CTU
    //---------------------------------------------
    //final pixel positions, extraction of CB roi
    int CB_Width = CTBSize >> uiDepth; //width of CU in pixels
    Point CB_Anc  = Point(AbsPart.x, AbsPart.y ) * 4 * Zoom; //times 4 since one Smallest-Partition spans 4x4 pixels!
    Point CB_Dims = Point(CB_Width , CB_Width ) * Zoom;
    Mat CB_Roi = CTB_Roi(Rect(CB_Anc.x, CB_Anc.y, CB_Dims.x, CB_Dims.y));

    //---------------------------------------------
    // INTRA / INTER COLORING OF CU
    //---------------------------------------------

    if(isMarkingBlocks)
    {
      if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTRA)
        CB_Roi = RED;
      else if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTER && pcCU->isSkipped(uiAbsZorderIdx))
        CB_Roi = GREEN;
      else if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTER && pcCU->getMergeFlag(uiAbsZorderIdx))
              CB_Roi = YELLOW;
      else if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTER)
        CB_Roi = BLUE;
      else
        CB_Roi = GRAY; //used for outer bounds CB's
    }

    //---------------------------------------------
    // DRAWING STORAGE-UNITS GRID
    //---------------------------------------------
    Point StorageUnits = Point(CB_Width/4, CB_Width/4);
    if(StorageUnits.x * StorageUnits.y != NumPartInCB)
       exit(-1);
    if(isDrawingStoreUnits)
      for(int y = 0; y<StorageUnits.y; y++)
        for(int x = 0; x<StorageUnits.x; x++)
        {
          Point SU_Anc  = Point(x*4, y*4 ) * Zoom; //times 4 since one Smallest-Partition spans 4x4 pixels!
          Point SU_Dims = Point(4 , 4 ) * Zoom;
          Mat SU_Roi = CB_Roi(Rect(SU_Anc.x, SU_Anc.y, SU_Dims.x, SU_Dims.y));

          //drawing StorageUnit borders
          Scalar color2(0,0,0);
          line(SU_Roi, Point(0,0), Point(SU_Dims.x-1,0), color2,1); //top
          line(SU_Roi, Point(0,0), Point(0, SU_Dims.y-1), color2,1);//left
          line(SU_Roi, Point(SU_Dims.x-1,SU_Dims.y-1), Point(SU_Dims.x-1, 0), color2,1);//right
          line(SU_Roi, Point(SU_Dims.x-1,SU_Dims.y-1), Point(0, SU_Dims.y-1), color2,1);//right
        }


    PartSize ePartSize = pcCU->getPartitionSize( uiAbsZorderIdx );
    UInt NumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
    //NextPU_Increment: increment in terms of StorageUnits
    UInt NextPU_Increment = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() - uiDepth ) << 1 ) ) >> 4;

    //----------------------------------------------------
    //  PROCESSING PU'S
    //----------------------------------------------------
    for ( UInt uiPartIdx = 0, uiSubPartIdx = uiAbsZorderIdx; uiPartIdx < NumPU; uiPartIdx++, uiSubPartIdx += NextPU_Increment )
    {
      //----------------------------------------------------
      // GETTING MERGE INFO
      //----------------------------------------------------
      bool MergeFlag    = pcCU->getMergeFlag( uiSubPartIdx );
      UInt MergeIndex = pcCU->getMergeIndex(uiSubPartIdx);

      //----------------------------------------------------
      // GETTING PU SIZES AND COUNT TO NEXT PU (PU OFFSET?)
      //----------------------------------------------------
      UInt ruiPartAddr;
      Int PU_Width,  PU_Height;
      myGetPartIndexAndSize( pcCU, uiAbsZorderIdx, uiDepth, uiPartIdx, //uiPartIdx = PUIdx: PU index, maybe just one, maybe up to four
                             ruiPartAddr, PU_Width, PU_Height ); //ruiPartAddr is the same as the above uiSubPartIdx

      // ruiPartAddr is a relative index inside the current CU (no practical use here)
      // however, encoder code makes heavy use of this for addressing MV, MVField, etc.!
      //assert(ruiPartAddr != (uiSubPartIdx - uiAbsZorderIdx)); //just making sure we understood everything
      if(ruiPartAddr != (uiSubPartIdx - uiAbsZorderIdx))
        CB_Roi = Scalar(64,128,64);

      //----------------------------------------------------
      // WE ARE ABLE TO RECONSTRUCT PU ANCHOR IN PIXELS RELATIVE TO THE CTB
      //----------------------------------------------------
      RasterPartIdx  = g_auiZscanToRaster[uiAbsZorderIdx+ruiPartAddr];
      int SUnitsPerRow = CTBSize/4; // 64/4 = 16 StorageUnits in a CTB row
     // Point Anchor = Point(RasterPartIdx % (SUnitsPerRow), RasterPartIdx / (SUnitsPerRow))*4;

      //int CB_Width = CTBSize >> uiDepth; //width of CU in pixels
      Point PU_Anc  = Point(RasterPartIdx % (SUnitsPerRow), RasterPartIdx / (SUnitsPerRow)) * 4 *Zoom;
      Point PU_Dims = Point(PU_Width , PU_Height ) * Zoom;
      Mat PU_Roi = CTB_Roi(Rect(PU_Anc.x, PU_Anc.y, PU_Dims.x, PU_Dims.y));

      //----------------------------------------------------
      // DRAWING PU BORDERS
      //----------------------------------------------------
      if(isMarkingBlocks)
      {
        line(PU_Roi, Point(0,0), Point(PU_Dims.x-1,0), WHITE,2); //top
        line(PU_Roi, Point(0,0), Point(0, PU_Dims.y-1), WHITE,2);//left
        line(PU_Roi, Point(PU_Dims.x-1,PU_Dims.y-1), Point(PU_Dims.x-1, 0), WHITE,2);//right
        line(PU_Roi, Point(PU_Dims.x-1,PU_Dims.y-1), Point(0, PU_Dims.y-1), WHITE,2);//right
      }

      //rectangle(CTB_Roi, Anchor*Zoom, Anchor*Zoom+Point(riWidth*Zoom-1,riHeight*Zoom-1), Scalar(255,255,255), 3);
      //rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

      //----------------------------------------------------
      // WE ARE ABLE TO DETERMINE THE MV OF THIS PU
      //----------------------------------------------------
      TComMv cMv = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv( uiAbsZorderIdx+ruiPartAddr );
      Point Mv(cMv.getHor(), cMv.getVer());
      if(Mv.x != 0 || Mv.y != 0) //if this PU has motion (taking care not to register zero-MVs from intra and stuff)
      {
        //we need anchors relative to the whole image!
        Point MV_Anc = CTB_Anc + PU_Anc + Point(PU_Dims.x / 2, PU_Dims.y / 2); //PU anchor is relative to CTB
        Point MV_Dims = Point(cMv.getHor()*Zoom / 4, cMv.getVer()*Zoom / 4 ); //division by 4: since MV has quadpel precision
        //if(isDrawingVectors)
        //   line(Img, MV_Anc, MV_Anc+MV_Dims, GREEN_2, 2, CV_AA); //top

        MV_StartPoints.push_back(MV_Anc);
        MV_EndPoints.push_back(MV_Anc+MV_Dims);

        //----------------------------------------------------
        // SAVING MV INFO FOR LATER PROCESSING
        //----------------------------------------------------
        //TODO: Consider PUs refering to other ref frames than ref0!
        //TODO: Get rid of type conversion stalking
        //TODO: Get rid of getHor and getVer
        Point2f KeypCur(float(MV_Anc.x)/Zoom, float(MV_Anc.y)/Zoom); //need to get rid of zoom for export
        KeypointsCur.push_back(Point2f(KeypCur));
        Point2f KeypRef = KeypCur +  Point2f(float(cMv.getHor())/ 4, float(cMv.getVer())/ 4);
        KeypointsRef.push_back(Point2f(KeypRef));
      }

      //----------------------------------------------------
      // WRITING PU TEXT
      //----------------------------------------------------
      if(isInterestArea)
      {
        sprintf(text, "---PU %d/%d, %dx%d---", uiPartIdx+1, NumPU, PU_Width, PU_Height);
        helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
        sprintf(text, "Merge Flag/Idx: %s, %d", FalseTrueStrings[MergeFlag], MergeIndex);
        helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
        sprintf(text, "ruiPartAddr: %d", ruiPartAddr);
        helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
        //sprintf(text, "Anchor: %d,%d", PU_Anc.x, PU_Anc.y);
        //helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
        sprintf(text, "MV hor/ver: %d/%d", Mv.x, Mv.y);
        helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
        //sprintf(text, "MV Anc hor/ver: %d/%d", MV_Anc.x, MV_Anc.y);
        //helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      }
    }

    //----------------------------------------------------
    // DRAWING CB BORDERS
    //----------------------------------------------------
    if(isMarkingBlocks)
    {
      line(CB_Roi, Point(0,0), Point(CB_Dims.x-1,0), BLACK,3); //top
      line(CB_Roi, Point(0,0), Point(0, CB_Dims.y-1), BLACK,3);//left
      line(CB_Roi, Point(CB_Dims.x-1,CB_Dims.y-1), Point(CB_Dims.x-1, 0), BLACK,3);//right
      line(CB_Roi, Point(CB_Dims.x-1,CB_Dims.y-1), Point(0, CB_Dims.y-1), BLACK,3);//right
    }

    //----------------------------------------------------
    // WRITING CTB TEXT
    //----------------------------------------------------
    if(isInterestArea)
    {
      offs++;
      sprintf(text, "CTB Addr: %d", pcCU->getAddr()); //CTB Index
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "CU Depth: %d", pcCU->getDepth(uiAbsZorderIdx));
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "CU Dims: %dx%d", pcCU->getWidth(uiAbsZorderIdx), pcCU->getHeight(uiAbsZorderIdx));
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "AbsZorderIdx in CTB: %d", uiAbsZorderIdx);
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "Increment to next CU: %d", NextCU_Increment);
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "PU Partitions: %s", PartitionSizeStrings[pcCU->getPartitionSize(uiAbsZorderIdx)]);
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "Increment to next PU: %d", NextPU_Increment);
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "NumStorUnitsInCTB: %d", pcPic->getNumPartInCU());
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
      sprintf(text, "NumStorUnitsInCB: %d", NumPartInCB);
      helpme::writeText(CB_Roi, text, WHITE, Point(0,offs++)*20);
    }
    //off we go to next CU!
  } //end xDrawCU

  //NO LONGER USED
  //markCTB with yellow frame
//  void markCTB (TComPic* pcPic)
//  {
//    for(int i = 0; i < NumObsCTUs; i++)
//    {
//       int CTU_Index = ObsCTU[i];
//
//       TComDataCU* pcCU = pcPic->getCU( CTU_Index );
//       TComPic* rpcPic = pcCU->getPic();
//       UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();  //=13
//       UInt uiHeightInLCUs = rpcPic->getPicSym()->getFrameHeightInCU(); //=8
//       int LCU_x     = CTU_Index % uiWidthInLCUs; //from 0 to 12
//       int LCU_y     = CTU_Index / uiWidthInLCUs; //from 0 to 7
//       UInt uiAbsZorderIdx = 0;
//       Int iRastPartIdx  = g_auiZscanToRaster[uiAbsZorderIdx];
//       int AbsPart_x = iRastPartIdx % (64/4); //(64/4) : partition ticks in a LCU
//       int AbsPart_y = iRastPartIdx / (64/4);
//       int pixel_x = LCU_x*64+AbsPart_x*4;
//       int pixel_y = LCU_y*64+AbsPart_y*4;
//       Point P_anc = Point(pixel_x, pixel_y ) * Zoom; //anchor
//       Point P_dim = Point(64 , 64 ) * Zoom;
//       Mat roi = Img(Rect(P_anc.x, P_anc.y, P_dim.x, P_dim.y));
//       int LineWidth = 5 * Zoom;
//       line(roi, Point(0,0), Point(P_dim.x-1,0), YELLOW, LineWidth); //top
//       line(roi, Point(0,0), Point(0, P_dim.y-1), YELLOW, LineWidth);//left
//       line(roi, Point(P_dim.x-1,P_dim.y-1), Point(P_dim.x-1, 0), YELLOW, LineWidth);//right
//       line(roi, Point(P_dim.x-1,P_dim.y-1), Point(0, P_dim.y-1), YELLOW, LineWidth);//right
//    }
//  }

  void myGetPartIndexAndSize( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth,
                              UInt PUIdx, //PUIdx: PU index, maybe just one, maybe up to four
                              UInt& ruiPartAddr, Int& riWidth, Int& riHeight )
  {
    PartSize ePartSize = pcCU->getPartitionSize( uiAbsZorderIdx );

    unsigned char Width  = pcCU->getWidth(uiAbsZorderIdx);
    unsigned char Height = pcCU->getHeight(uiAbsZorderIdx);

    TComPic* pcPic     = pcCU->getPic();
    UInt NumPartInCTB  = pcPic->getNumPartInCU();      //number of StorageUnits (4x4) in CTB
    UInt NumPartInCB = NumPartInCTB >> (uiDepth<<1); //number of StorageUnits (4x4) in current CB

    switch ( ePartSize )
    {
      case SIZE_2NxN:
        riWidth = Width;
        riHeight = Height >> 1;
        ruiPartAddr = ( PUIdx == 0 )? 0 : NumPartInCB >> 1;
        break;
      case SIZE_Nx2N:
        riWidth = Width >> 1;
        riHeight = Height;
        ruiPartAddr = ( PUIdx == 0 )? 0 : NumPartInCB >> 2;
        break;
      case SIZE_NxN:
        riWidth = Width >> 1;
        riHeight = Height >> 1;
        ruiPartAddr = ( NumPartInCB >> 2 ) * PUIdx;
        break;
      case SIZE_2NxnU:
        riWidth     = Width;
        riHeight    = ( PUIdx == 0 ) ?  Height >> 2 : ( Height >> 2 ) + ( Height >> 1 );
        ruiPartAddr = ( PUIdx == 0 ) ? 0 : NumPartInCB >> 3;
        break;
      case SIZE_2NxnD:
        riWidth     = Width;
        riHeight    = ( PUIdx == 0 ) ?  ( Height >> 2 ) + ( Height >> 1 ) : Height >> 2;
        ruiPartAddr = ( PUIdx == 0 ) ? 0 : (NumPartInCB >> 1) + (NumPartInCB >> 3);
        break;
      case SIZE_nLx2N:
        riWidth     = ( PUIdx == 0 ) ? Width >> 2 : ( Width >> 2 ) + ( Width >> 1 );
        riHeight    = Height;
        ruiPartAddr = ( PUIdx == 0 ) ? 0 : NumPartInCB >> 4;
        break;
      case SIZE_nRx2N:
        riWidth     = ( PUIdx == 0 ) ? ( Width >> 2 ) + ( Width >> 1 ) : Width >> 2;
        riHeight    = Height;
        ruiPartAddr = ( PUIdx == 0 ) ? 0 : (NumPartInCB >> 2) + (NumPartInCB >> 4);
        break;
      default:
        assert ( ePartSize == SIZE_2Nx2N );
        riWidth = Width;
        riHeight = Height;
        ruiPartAddr = 0;
        break;
    }
  }
};

