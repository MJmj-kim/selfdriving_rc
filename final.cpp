//reversion 20_05_15

#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

#define Cen1 9
#define Cen2 10

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate, frameFinalDuplicate1;
Mat ROILane, ROILaneEnd;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd;

RaspiCam_Cv Camera;

stringstream ss;


vector<int> histrogramLane;
vector<int> histrogramLaneEnd;

Point2f Source[] = {Point2f(55,160),Point2f(345,160),Point2f(5,240), Point2f(395,240)};
Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};

CascadeClassifier Stop_Cascade, Traffic_Cascade;
Mat frame_Stop, RoI_Stop, gray_Stop, frame_Traffic, RoI_Traffic, gray_Traffic;
vector<Rect> Stop, Traffic;
int dist_Stop, dist_Traffic;

 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

void Capture()
{
    Camera.grab();
    Camera.retrieve( frame);
    cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
    cvtColor(frame, frame_Traffic, COLOR_BGR2RGB);
    cvtColor(frame, frame, COLOR_BGR2RGB);
    
}

void Perspective()
{
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
	
	Matrix = getPerspectiveTransform(Source, Destination);
	warpPerspective(frame, framePers, Matrix, Size(400,240));
}

void Threshold()
{
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
	inRange(frameGray, 200, 255, frameThresh);
	Canny(frameGray,frameEdge, 900, 900, 3, false);
	add(frameThresh, frameEdge, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
	cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);   //used in histrogram function only
	
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();
    
    for(int i=0; i<400; i++)       //frame.size().width = 400
    {
	ROILane = frameFinalDuplicate(Rect(i,140,1,100));
	divide(255, ROILane, ROILane);
	histrogramLane.push_back((int)(sum(ROILane)[0])); 
    }
	
	histrogramLaneEnd.resize(400);
        histrogramLaneEnd.clear();
	for (int i = 0; i < 400; i++)       
	{
		ROILaneEnd = frameFinalDuplicate1(Rect(i, 0, 1, 240));   
		divide(255, ROILaneEnd, ROILaneEnd);       
		histrogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));  
		
	
	}
	   laneEnd = sum(histrogramLaneEnd)[0];
	   cout<<"Lane END = "<<laneEnd<<endl;
}

void LaneFinder()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr); 
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); 
}

void LaneCenter()
{
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos+5;  
    frameCenter = 191;
    
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    Result = laneCenter-frameCenter;
}

void Stop_detection()
{
    if(!Stop_Cascade.load("//home//pi//Desktop//MACHINE LEARNING//Stop_cascade.xml"))
    {
	printf("Unable to open stop cascade file");
    }
    
    RoI_Stop = frame_Stop(Rect(200,0,200,200));
    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop, Stop);
    
    for(int i=0; i<Stop.size(); i++)
    {
	Point P1(Stop[i].x, Stop[i].y);
	Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);
	
	rectangle(RoI_Stop, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
	dist_Stop = (-1.07)*(P2.x-P1.x) +102.597;
       ss.str(" ");
       ss.clear();
       ss<<"D = "<<dist_Stop<<"cm";
       putText(RoI_Stop, ss.str(), Point2f(1,190), 0,1, Scalar(0,0,255), 2);
	
    }
    
}

void Traffic_detection()
{
    if(!Traffic_Cascade.load("//home//pi//Desktop//MACHINE LEARNING//Traffic_cascade.xml"))
    {
	printf("Unable to open Traffic cascade file");
    }
    
    RoI_Traffic = frame_Traffic(Rect(200,0,200,200));
    cvtColor(RoI_Traffic, gray_Traffic, COLOR_RGB2GRAY);
    equalizeHist(gray_Traffic, gray_Traffic);
    Traffic_Cascade.detectMultiScale(gray_Traffic, Traffic);
    
    for(int i=0; i<Traffic.size(); i++)
    {
	Point P1(Traffic[i].x, Traffic[i].y);
	Point P2(Traffic[i].x + Traffic[i].width, Traffic[i].y + Traffic[i].height);
	
	rectangle(RoI_Traffic, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Traffic, "Traffic Light", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
	dist_Traffic = (-2.5)*(P2.x-P1.x) +115;
       ss.str(" ");
       ss.clear();
       ss<<"D = "<<dist_Traffic<<"cm";
       putText(RoI_Traffic, ss.str(), Point2f(1,190), 0,1, Scalar(0,0,255), 2);
	
    }
    
}

int main(int argc,char **argv)
{
	
    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    
    Setup(argc, argv, Camera);
    cout<<"Connecting to camera"<<endl;
    if (!Camera.open())
    {
		
	cout<<"Failed to Connect"<<endl;
    }
     
	cout<<"Camera Id = "<<Camera.getId()<<endl;
     
 
    while(1)
    {
	
    auto start = std::chrono::system_clock::now();

    Capture();
    Perspective();
    Threshold();
    Histrogram();
    LaneFinder();
    LaneCenter();
    Stop_detection();
    Traffic_detection();
    
    if (dist_Stop > 30 && dist_Stop < 50)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 8
	digitalWrite(23, 0);
	digitalWrite(24, 1);
	cout<<"Stop sign"<<endl;
	dist_Stop = 0;
    
	goto Stop_sign;
    }
    if (dist_Traffic > 1 && dist_Traffic < 40)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 9
	digitalWrite(23, 0);
	digitalWrite(24, 1);
	cout<<"Traffic"<<endl;
	dist_Traffic = 0;
	
	goto Traffic;
    }
    if (laneEnd > 10000)
    {
       	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 7
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Lane End"<<endl;
    }
    
    else if (Result >-Cen1 && Result <Cen1)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 0
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Forward"<<endl;
    }
    
        
    else if (Result >=Cen1 && Result <Cen2)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 1
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right1"<<endl;
    }
    
        else if (Result >=Cen2 && Result <20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 2
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right2"<<endl;
    }
    
        else if (Result >= 20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 3
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right3"<<endl;
    }
    
        else if (Result <= -Cen1 && Result >-Cen2)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 4
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left1"<<endl;
    }
    
        else if (Result <=-Cen2 && Result >-20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 5
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left2"<<endl;
    }
    
        else if (Result <= -20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 6
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left3"<<endl;
    }
    
    Stop_sign:
    Traffic:
  
   if (laneEnd > 10000)
    {
       ss.str(" ");
       ss.clear();
       ss<<" Lane End";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,0), 2);
    
     }
    
    else if (Result >-Cen1 && Result <Cen1)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" Move Forward";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
    
    else if (Result > Cen1)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<"Move Right";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
     
     else if (Result < -Cen1)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" Move Left";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
    
    
    namedWindow("orignal", WINDOW_KEEPRATIO);
    moveWindow("orignal", 0, 100);
    resizeWindow("orignal", 640, 480);
    imshow("orignal", frame);
    
    namedWindow("Perspective", WINDOW_KEEPRATIO);
    moveWindow("Perspective", 640, 100);
    resizeWindow("Perspective", 640, 480);
    imshow("Perspective", framePers);
      
    namedWindow("Final", WINDOW_KEEPRATIO);
    moveWindow("Final", 1280, 100);
    resizeWindow("Final", 640, 480);
    imshow("Final", frameFinal);
    
    namedWindow("Stop Sign", WINDOW_KEEPRATIO);
    moveWindow("Stop Sign", 1280, 580);
    resizeWindow("Stop Sign", 640, 480);
    imshow("Stop Sign", RoI_Stop);  
    
    namedWindow("Traffic", WINDOW_KEEPRATIO);
    moveWindow("Traffic", 0, 580);
    resizeWindow("Traffic", 640, 480);
    imshow("Traffic", RoI_Traffic);
    
    waitKey(1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    //cout<<"FPS = "<<FPS<<endl;
    
    }

    
    return 0;
     
}
