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

class CShow_TransformUnits
{

public:
  Mat Img;  //holds visualized image (no blending included)
  int Zoom; //Zoom of one CTB
  Point ImgDim;
  bool isDrawingStoreUnits;
  char text[500];
  int call; //keeping track of calls to this class
  vector<Point2f> KeypointsCur, KeypointsRef;
  vector<Point> MV_StartPoints;
  vector<Point> MV_EndPoints;

  //Interest area
  Point CTB_Span; //x and y span of observed LCUs (may be selection or all LCUs)
  Point CTB_UpperLeft; //from (inclusive)
  Point CTB_LowerRight; //to (exclusive)

  CShow_TransformUnits(TComPic* pcPic, int NumCTBs, char *VizFile, int CTBSize )
  {
    //---------------------------------------------
    // CHANGE HERE
    //---------------------------------------------
    this->isDrawingStoreUnits = false;

    CTB_UpperLeft  = Point(0,2);
    CTB_LowerRight = Point(1,3);
    //---------------------------------------------

    call = 0;
    sprintf(text, "====%s(%d)====", "CShow_TransformUnits", call);
    int textlen = strlen(text);
    cout << text << endl;

    UInt WidthInLCUs  = pcPic->getPicSym()->getFrameWidthInCU();  //=13
    UInt HeightInLCUs = pcPic->getPicSym()->getFrameHeightInCU(); //=8

    Zoom = 3;
    CTB_UpperLeft  = Point(0,0);
    CTB_LowerRight = Point(WidthInLCUs, HeightInLCUs);
    CTB_Span = CTB_LowerRight - CTB_UpperLeft;
    ImgDim.x = (CTB_Span.x)*CTBSize*Zoom; //this is equal to 800x480 * Zoom
    ImgDim.y = (CTB_Span.y)*CTBSize*Zoom;


    Img = Mat(ImgDim.y, ImgDim.x, CV_8UC3, BLACK);

    //---------------------------------------------
    //---------------------------------------------
    //---------------------------------------------
    // DRAWING EACH QUADTREE PARTITION
    //---------------------------------------------
    for ( UInt LCU_Index = 0; LCU_Index < NumCTBs; LCU_Index++ )
    //for ( UInt LCU_Index = 0; LCU_Index < 1; LCU_Index++ )
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
    // BLENDING IN THE CUR FRAME
    //---------------------------------------------
    bool isBlending = true;
    Mat Blended;
    if(isBlending)
    {
      int DIMX = pcPic->getPicYuvRec()->getWidth();
      int DIMY = pcPic->getPicYuvRec()->getHeight();

      Mat Tmp;
      //helpme::copy_PicYuv2Mat(pcPic->getPicYuvOrg(), Tmp); // original not available on decoder side!
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
	sprintf(text, "./jpg/TransformUnits_Poc%04d.jpg", pcPic->getPOC());
	sprintf(VizFile, "TransformUnits_Poc%04d.jpg", pcPic->getPOC()); //so that caller knows name of file
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

    //if(isMarkingBlocks)
    if(true)
    {
      if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTRA)
      {
        CB_Roi = RED;
      }
      else if( (pcCU->getPredictionMode(uiAbsZorderIdx) != MODE_INTRA) && !(pcCU->getMergeFlag( uiAbsZorderIdx ) && pcCU->getPartitionSize(uiAbsZorderIdx) == SIZE_2Nx2N ) )
      {
        // this is the case where rqt_root_cbf (aka no_residual_syntax_flag) is present
        if(pcCU->getQtRootCbf( uiAbsZorderIdx ))
        {
          // residual is coded
          CB_Roi = BLUE;
        }
        else
        {
          // residual is NOT coded --> kind of SKIP
          //CB_Roi = YELLOW;
          // BLUE with GREEN stripes (it is a kind of both, inter and skip)
          CB_Roi = BLUE;
          for(int c=0; c<CB_Width*Zoom; c+=Zoom)
          {
            CB_Roi.colRange(c, c+Zoom-1) = GREEN;
          }
        }
      }
      else if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTER && pcCU->isSkipped(uiAbsZorderIdx))
      {
        CB_Roi = GREEN;
      }
      else if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTER)
      {
        CB_Roi = BLUE;
      }
      else
      {
        CB_Roi = GRAY; //used for outer bounds CB's
      }
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
          //Scalar color2(0,0,0);
          Scalar color2(255,0,255);
          line(SU_Roi, Point(0,0), Point(SU_Dims.x-1,0), color2,1); //top
          line(SU_Roi, Point(0,0), Point(0, SU_Dims.y-1), color2,1);//left
          line(SU_Roi, Point(SU_Dims.x-1,SU_Dims.y-1), Point(SU_Dims.x-1, 0), color2,1);//right
          line(SU_Roi, Point(SU_Dims.x-1,SU_Dims.y-1), Point(0, SU_Dims.y-1), color2,1);//right
        }

    // Draw additional TU grid if the current CU is further split
    // (see TEncEntropy::xEncodeTransform)
    // Only draw TUs if rqt_root_cbf == 1
    if(pcCU->getQtRootCbf( uiAbsZorderIdx ))
      xDrawTU( pcCU, uiDepth, CB_Width, 0, CTB_Roi, uiAbsZorderIdx, CTBSize );
    

    //----------------------------------------------------
    // DRAWING CB BORDERS
    //----------------------------------------------------
    Int thickness = 3;
    Scalar col = BLACK;
    
    line(CB_Roi, Point(0,0), Point(CB_Dims.x-1,0), col, thickness); //top
    line(CB_Roi, Point(0,0), Point(0, CB_Dims.y-1), col, thickness);//left
    line(CB_Roi, Point(CB_Dims.x-1,CB_Dims.y-1), Point(CB_Dims.x-1, 0), col, thickness);//right
    line(CB_Roi, Point(CB_Dims.x-1,CB_Dims.y-1), Point(0, CB_Dims.y-1), col, thickness);//right


    //off we go to next CU!
  } //end xDrawCU


    void xDrawTU( TComDataCU* pcCU,
                  UInt        uiDepth,
                  UInt        TB_Width,
                  UInt        uiAbsPartIdx,     // index counted for current CB
                  Mat&        CTB_Roi,
                  UInt        uiAbsZorderIdx,   // index counted for complete CTB to BEGINNING of current CB
                  int         CTBSize
                )
    {
        const UInt uiSubdiv = pcCU->getTransformIdx( uiAbsZorderIdx + uiAbsPartIdx ) + pcCU->getDepth( uiAbsZorderIdx + uiAbsPartIdx ) > uiDepth;
        const UInt uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()]+2 - uiDepth;
        //printf("uiAbsZorderIdx + uiAbsPartIdx = %u; uiDepth = %u; uiSubdiv = %u\n", uiAbsZorderIdx + uiAbsPartIdx, uiDepth, uiSubdiv);
        
        if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiSubdiv == 0 )
        {
            printf("TU is too big!!\n");
        }
        
        if( uiSubdiv )
        {
            ++uiDepth;
            const UInt partNum = pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1); // = numQPart
            //printf("partNum = %u\n", partNum);
            
            xDrawTU( pcCU, uiDepth, TB_Width >> 1, uiAbsPartIdx, CTB_Roi, uiAbsZorderIdx, CTBSize );
            
            uiAbsPartIdx += partNum;
            xDrawTU( pcCU, uiDepth, TB_Width >> 1, uiAbsPartIdx, CTB_Roi, uiAbsZorderIdx, CTBSize );
            
            uiAbsPartIdx += partNum;
            xDrawTU( pcCU, uiDepth, TB_Width >> 1, uiAbsPartIdx, CTB_Roi, uiAbsZorderIdx, CTBSize );
            
            uiAbsPartIdx += partNum;
            xDrawTU( pcCU, uiDepth, TB_Width >> 1, uiAbsPartIdx, CTB_Roi, uiAbsZorderIdx, CTBSize );
        }
        else
        {
            // draw the TBs
            
            //from Zscan to Raster, then getting x and y indices
            int RasterPartIdx  = g_auiZscanToRaster[uiAbsZorderIdx + uiAbsPartIdx]; // from CTB beginning to CU beginning to TB beginning
            Point AbsPart = Point(RasterPartIdx % (CTBSize/4), RasterPartIdx / (CTBSize/4)); // 64/4 = partition ticks (=StorageUnits-Ticks) in a LCU
            
            Point TB_Anc  = Point(AbsPart.x, AbsPart.y ) * 4 * Zoom; //times 4 since one Smallest-Partition spans 4x4 pixels!
            Point TB_Dims = Point(TB_Width , TB_Width ) * Zoom;
            Mat TB_Roi = CTB_Roi(Rect(TB_Anc.x, TB_Anc.y, TB_Dims.x, TB_Dims.y));
            
            
            // check if DCT or DST is used for this luma TB
            if(pcCU->getPredictionMode(uiAbsZorderIdx) == MODE_INTRA && TB_Width == 4)
            {
                TB_Roi = CV_RGB(250,150,0);
            }
            
            
            Int thickness = 2;
            Scalar col = WHITE;
            UInt offset = 0;
            
            line(TB_Roi, Point(offset,offset), Point(TB_Dims.x-1-offset,offset), col, thickness); //top
            line(TB_Roi, Point(offset,offset), Point(offset, TB_Dims.y-1-offset), col, thickness);//left
            line(TB_Roi, Point(TB_Dims.x-1-offset,TB_Dims.y-1-offset), Point(TB_Dims.x-1-offset, offset), col, thickness);//right
            line(TB_Roi, Point(TB_Dims.x-1-offset,TB_Dims.y-1-offset), Point(offset, TB_Dims.y-1-offset), col, thickness);//bottom
            
            
            
        }
    } //end xDrawTU

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

