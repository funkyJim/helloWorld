#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Mat src; Mat hsv; Mat hue;
int bins = 25;
void Hist_and_Backproj(int, void* );

int main( int, char** argv )
{
  src = imread( argv[1] );
  if( src.empty() )
    { cout<<"Usage: ./calcBackProject_Demo1 <path_to_image>"<<endl;
      return -1;
    }
  cvtColor( src, hsv, COLOR_BGR2HSV );
  hue.create( hsv.size(), hsv.depth() );
  int ch[] = { 0, 0 };
  mixChannels( &hsv, 1, &hue, 1, ch, 1 );
  const char* window_image = "Source image";
  namedWindow( window_image, WINDOW_NORMAL );
  createTrackbar("* Hue  bins: ", window_image, &bins, 180, Hist_and_Backproj );
  Hist_and_Backproj(0, 0);
  imshow( window_image, src );
  waitKey(0);
  return 0;
}

void Hist_and_Backproj(int, void* )
{
  Mat hist;
  int histSize = MAX( bins, 2 );
  float hue_range[] = { 0, 180 };
  const float* ranges = { hue_range };

  calcHist( &hue, 1, 0, Mat(), hist, 1, &histSize, &ranges );
  normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

  Mat backproj;
  calcBackProject( &hue, 1, 0, hist, backproj, &ranges );

  namedWindow( "BackProj", WINDOW_NORMAL );
  imshow( "BackProj", backproj );

  int w = 400; int h = 400; // histImg dimensions w=cols, h=rows
  int bin_w = cvRound( (double) w / histSize ); // histImg bin width
  Mat histImg = Mat::zeros( w, h, CV_8UC3 ); // Mat used to draw histogram

  for( int i = 0; i < bins; i ++ ) {
    rectangle( histImg, Point( i*bin_w, h ), // bottom-left corner
      Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ), // top-right corner
      Scalar( 0, 0, 255 ), // rectangle color=red
      -1 ); // filled rectangle
  }

  namedWindow( "Histogram", WINDOW_NORMAL );
  imshow( "Histogram", histImg );
}
