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

using namespace cv;
using namespace std;

#define HEVC_CODE //activates certain code passages
#include "Defines.h"
#include "CHelper.h"

#include "TLibCommon/TypeDef.h"

class CShow_BitsCostDistortion
{

public:
  Mat Img;
  int Zoom; //Zoom of one CTU
  Point ImgDim;
  bool isInterestArea;
  bool isMarkingBlocks;
  bool isDrawingStoreUnits;
  bool isDrawingVectors;
  char text[500];
  vector<Point2f> KeypointsCur, KeypointsRef;
  vector<Point> MV_StartPoints;
  vector<Point> MV_EndPoints;

  CShow_BitsCostDistortion(TComPic* pcPic, int NumCTBs, char *VizFile,
      bool isMarkingBlocks = true, bool isDrawingVectors = true )
  {
    //---------------------------------------------
    // CHANGE HERE
    //---------------------------------------------
    isInterestArea = false; // false means "full image"
    this->isDrawingVectors = isDrawingVectors;
    this->isMarkingBlocks = isMarkingBlocks;
    this->isDrawingStoreUnits = false;
    //Point CTB_UpperLeft  = Point(0,0); //define interest-area
    //Point CTB_LowerRight = Point(3,3); //exclusive!
    Point CTB_UpperLeft  = Point(0,2); //define interest-area
    Point CTB_LowerRight = Point(1,3); //exclusive!
    //---------------------------------------------

    static int call = 0;
    sprintf(text, "====%s(%d)====", "CShow_BitsCostDistortion", call);
    int textlen = strlen(text);
    cout << text << endl;

    UInt WidthInLCUs  = pcPic->getPicSym()->getFrameWidthInCU();  //=13
    UInt HeightInLCUs = pcPic->getPicSym()->getFrameHeightInCU(); //=8
    Point CTB_Span;
    if(isInterestArea) //only interest-area
    {
      Zoom = 64;
      CTB_Span = CTB_LowerRight - CTB_UpperLeft;
      ImgDim.x = CTB_Span.x*64*Zoom;
      ImgDim.y = CTB_Span.y*64*Zoom;
    }
    else //full image
    {
      Zoom = 3;
      CTB_UpperLeft  = Point(0,0);
      CTB_LowerRight = Point(WidthInLCUs, HeightInLCUs);
      CTB_Span = CTB_LowerRight - CTB_UpperLeft;
      ImgDim.x = (CTB_Span.x)*64*Zoom; //this is equal to 800x480 * Zoom
      ImgDim.y = (CTB_Span.y)*64*Zoom;
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
      // GETTING ROI ON ZOOMED CTB REGION
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
      Point CTB_Anc = (CTB_Pos - CTB_UpperLeft) * 64 * Zoom;
      //CTB_Roi holds image ROI of the current CTB
      Mat CTB_Roi = Img(Rect(CTB_Anc.x, CTB_Anc.y, 64*Zoom, 64*Zoom));

      //---------------------------------------------
      // LETS DIVE INTO A RECURSION
      //---------------------------------------------
      TComDataCU* pcCU = pcPic->getCU( LCU_Index );
      xDrawCU( pcCU, 0, 0, CTB_Roi, CTB_Anc);

      //---------------------------------------------
      // FINALIZE: DRAW YELLOW CTB BORDER AND WRITE INDEX
      //---------------------------------------------
//      line(CTB_Roi, Point(0,0), Point(64*Zoom-1,0), YELLOW, 10); //top
//      line(CTB_Roi, Point(0,0), Point(0, 64*Zoom-1), YELLOW, 10);//left
//      line(CTB_Roi, Point(64*Zoom-1,64*Zoom-1), Point(64*Zoom-1, 0), YELLOW, 10);//right
//      line(CTB_Roi, Point(64*Zoom-1,64*Zoom-1), Point(0, 64*Zoom-1), YELLOW, 10);//right
      sprintf(text, "%d", LCU_Index);
      helpme::writeText(CTB_Roi, text, YELLOW, Point(7,7), 1.0);
    }
    //---------------------------------------------
    //---------------------------------------------
    //---------------------------------------------

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
      Mat Tmp;
      //helpme::copy_PicYuv2Mat(pcPic->getPicYuvOrg(), Tmp); // original not available on decoder side!
      helpme::copy_PicYuv2Mat(pcPic->getPicYuvRec(), Tmp);
      helpme::convertToCV_8UC3(Tmp);
      Mat CurFrame = Mat(DIMY+100, DIMX+100, CV_8UC3, BLACK); //CTBs expand beyond image!
      Tmp.copyTo(CurFrame(Rect(0, 0, DIMX, DIMY)));
      cvtColor(CurFrame, CurFrame, CV_YCrCb2RGB);
      cvtColor(CurFrame, CurFrame, CV_RGB2GRAY);
      Mat pointers[] = { CurFrame, CurFrame, CurFrame };
      Mat Gray; //gray == Cur in gray
      merge(pointers, 3, Gray);
      Mat InterestArea = Gray(Rect(CTB_UpperLeft.x*64, CTB_UpperLeft.y*64, CTB_Span.x*64, CTB_Span.y*64));
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
	  sprintf(text, "./jpg/BitsCostDistortion_Poc%04d.jpg", pcPic->getPOC());
	  sprintf(VizFile, "BitsCostDistortion_Poc%04d.jpg", pcPic->getPOC()); //so that caller knows name of file
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



  void xDrawCU( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, Mat &CTB_Roi, Point &CTB_Anc)
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
        UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
        UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
        if(    ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() )
            && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
        {
            xDrawCU( pcCU, uiAbsZorderIdx, uiDepth+1, CTB_Roi, CTB_Anc);
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
    Point AbsPart = Point(RasterPartIdx % (64/4), RasterPartIdx / (64/4)); // 64/4 = partition ticks (=StorageUnits-Ticks) in a LCU

    //---------------------------------------------
    // ANCHOR AND DIM INFO OF CU INSIDE THE CTU
    //---------------------------------------------
    //final pixel positions, extraction of CB roi
    int CB_Width = 64 >> uiDepth; //width of CU in pixels
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
    //UInt NumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
    //NextPU_Increment: increment in terms of StorageUnits
    UInt NextPU_Increment = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() - uiDepth ) << 1 ) ) >> 4;

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

