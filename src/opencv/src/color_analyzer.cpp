/*
 * Analyzes an image and transforms it into a binary image
 * given some sort of colour specifiction
 * 
 * Current method: Conversion to HSV (Hue, Saturation, Value) and filtering
 * the numbers appropriately. Currently trying out methods to find 
 * which HSV thresholds are the best and see if it can't be
 * automated properly.
 * Ref: https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *          - Just the HSV value finding rectangle part, not the object tracking part.
 * 			- Sketchy, works better if you make a rectangle on the ground
 * Usage: Point to a video file, press C to start calibrating. Then click and drag a rectangle on the calibration window
 * 			You should see H S V values being set afterwards.
 */   

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

//Disgusting global variables for testing purposes
int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

bool calibrationMode = false;
bool calibrationImgSet = false;
bool mouseIsDragging;
bool mouseMove;
bool rectangleSelected;

cv::Point initialClickPoint, currentMousePoint;
cv::Rect rectangleROI;
vector<int> H_ROI, S_ROI, V_ROI;

//Window Namings
const string inputWindow = "input";
const string outputWindow = "output";
const string calibrationWindow = "Calibration Frame";
const string hsvWindow = "HSV";

void convertToBinary(const Mat &input, Mat &output, Mat &hsvImage);
void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param);
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame);

int main(int argc, char** argv){
    
    //Takes ownership of camera
    VideoCapture cap(0);//"/home/valerian/Documents/src/opencv/course.mov");
    if (!cap.isOpened()){
        cout << "Error opening camera" << endl;
        return -1;
    }

    Mat inputImage;
    Mat outputImage; 
    Mat hsvImage;
    Mat calibrationImage;
    Mat hsvCalibration;
   
    //Camera information for user
    
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size: " << width << " x " << height << endl;
    
    
    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(outputWindow, CV_WINDOW_AUTOSIZE);
    //Testing Window
    //namedWindow("Control", CV_WINDOW_AUTOSIZE);    
    //Good Values: lowh = 98, highh = 130, lows = 30, highs = 165, lowv = 129, highv = 255
    /*
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
    */


    mouseIsDragging = false;
    mouseMove = false;
    rectangleSelected = false;
    while(1){

        //Reads image from camera
        
        bool isRead = cap.read(inputImage);
        if (!isRead){
            cout << "Failed to read image from camera" << endl;
            cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            continue; 
            //break;
        }
        
        if (calibrationMode && !calibrationImgSet){
        	namedWindow(calibrationWindow, CV_WINDOW_AUTOSIZE);
        	setMouseCallback(calibrationWindow, clickAndDrag_Rectangle, &hsvCalibration);
        	calibrationImage = inputImage.clone();
        	cvtColor(calibrationImage, hsvCalibration, COLOR_BGR2HSV);
            calibrationImgSet = true;
        }
        else if (calibrationMode){
            imshow(calibrationWindow, calibrationImage);

        } 
        else {
            destroyWindow(calibrationWindow);
        }
        
        recordHSV_Values(inputImage, hsvCalibration);
        convertToBinary(inputImage, outputImage, hsvImage);
        imshow(inputWindow, inputImage);
        imshow(outputWindow, outputImage);

        //Escape key to finish program
        int a = waitKey(30); 
        if (a == 27){
            cout << "Escaped by user" << endl;
            break;
        } else if (a == 99){
        	calibrationMode = true;
        }
    }
    return 0;
}

void convertToBinary(const Mat &input, Mat &output, Mat &hsvImage){
    
    cvtColor(input, hsvImage, COLOR_BGR2HSV);
    
    inRange(hsvImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), output);
    //Morphological Opening (removes small objects from foreground)
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    //Morphological Closing (fill small holes in the foreground)
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );   
    
} 

void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param){
    //only if calibration mode is true will we use the mouse to change HSV values
    if (calibrationMode == true){
        //get handle to video feed passed in as "param" and cast as Mat pointer
        if (event == EVENT_LBUTTONDOWN && mouseIsDragging == false)
        {
            //keep track of initial point clicked
            initialClickPoint = cv::Point(x, y);
            //user has begun dragging the mouse
            mouseIsDragging = true;
        }
        /* user is dragging the mouse */
        if (event == EVENT_MOUSEMOVE && mouseIsDragging == true)
        {
            //keep track of current mouse point
            currentMousePoint = cv::Point(x, y);
            //user has moved the mouse while clicking and dragging
            mouseMove = true;
        }
        /* user has released left button */
        if (event == EVENT_LBUTTONUP && mouseIsDragging == true)
        {
            //set rectangle ROI to the rectangle that the user has selected
            rectangleROI = Rect(initialClickPoint, currentMousePoint);
            calibrationMode = false;
            calibrationImgSet = false;
            //reset boolean variables
            mouseIsDragging = false;
            mouseMove = false;
            rectangleSelected = true;
        }

        if (event == EVENT_RBUTTONDOWN){
            //user has clicked right mouse button
            //Reset HSV Values
            iLowH = 0;
            iLowS = 0;
            iLowV = 0;
            iHighH = 255;
            iHighS = 255;
            iHighV = 255;

        }
        if (event == EVENT_MBUTTONDOWN){

            //user has clicked middle mouse button
            //enter code here if needed.
        }

    }

}
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame){
    //save HSV values for ROI that user selected to a vector
    if (mouseMove == false && rectangleSelected == true){
        
        //clear previous vector values
        if (H_ROI.size()>0) H_ROI.clear();
        if (S_ROI.size()>0) S_ROI.clear();
        if (V_ROI.size()>0 )V_ROI.clear();
        //if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
        if (rectangleROI.width<1 || rectangleROI.height<1) cout << "Please drag a rectangle, not a line" << endl;
        else{
            for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
                //iterate through both x and y direction and save HSV values at each and every point
                for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++){
                    //save HSV value at this point
                    H_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[0]);
                    S_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[1]);
                    V_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[2]);
                }
            }
        }
        //reset rectangleSelected so user can select another region if necessary
        rectangleSelected = false;
        //set min and max HSV values from min and max elements of each array
                    
        if (H_ROI.size()>0){
            //NOTE: min_element and max_element return iterators so we must dereference them with "*"
            iLowH = *std::min_element(H_ROI.begin(), H_ROI.end());
            iHighH = *std::max_element(H_ROI.begin(), H_ROI.end());
            cout << "MIN 'H' VALUE: " << iLowH << endl;
            cout << "MAX 'H' VALUE: " << iHighH << endl;
        }
        if (S_ROI.size()>0){
            iLowS = *std::min_element(S_ROI.begin(), S_ROI.end());
            iHighS = *std::max_element(S_ROI.begin(), S_ROI.end());
            cout << "MIN 'S' VALUE: " << iLowS << endl;
            cout << "MAX 'S' VALUE: " << iHighS << endl;
        }
        if (V_ROI.size()>0){
            iLowV = *std::min_element(V_ROI.begin(), V_ROI.end());
            iHighV = *std::max_element(V_ROI.begin(), V_ROI.end());
            cout << "MIN 'V' VALUE: " << iLowV << endl;
            cout << "MAX 'V' VALUE: " << iHighV << endl;
        }

    }

    if (mouseMove == true){
        //if the mouse is held down, we will draw the click and dragged rectangle to the screen
        rectangle(hsv_frame, initialClickPoint, cv::Point(currentMousePoint.x, currentMousePoint.y), cv::Scalar(255, 255, 255), 1, 8, 0);
    }
}
