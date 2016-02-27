/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     Issue tracker: http://code.opencv.org
     GitHub:        https://github.com/Itseez/opencv/
   ************************************************** */



/* **************************************************

	stereo calibration using pair images

	edited by seo.kg
	Dec.30.2015~

	TODO
	need to correct the rms error/ reprojection error
	-> subpixel opencv (X)
	-> change the size of the box: 10cm for chessboard box size
	100mm (X)

   ************************************************** */
#include "use_opencv.h"
#include "stereo_calib.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

	// ************************* //
	// stereo camera calibration //
	// ************************* //
    Size boardSize;
    string imagelistfn;
    bool showRectified = true;

    for( int i = 1; i < argc; i++ )
    {
        if( string(argv[i]) == "-w" )
        {
            if( sscanf(argv[++i], "%d", &boardSize.width) != 1 || boardSize.width <= 0 )
            {
                cout << "invalid board width" << endl;
                return print_help();
            }
        }
        else if( string(argv[i]) == "-h" )
        {
            if( sscanf(argv[++i], "%d", &boardSize.height) != 1 || boardSize.height <= 0 )
            {
                cout << "invalid board height" << endl;
                return print_help();
            }
        }
        else if( string(argv[i]) == "-nr" )
            showRectified = false;
        else if( string(argv[i]) == "--help" )
            return print_help();
        else if( argv[i][0] == '-' )
        {
            cout << "invalid option " << argv[i] << endl;
            return 0;
        }
        else
            imagelistfn = argv[i];
    }

    if( imagelistfn == "" )
    {
		imagelistfn = "../head_tracking/images/data.xml";
		boardSize = Size(9, 11);
		//imagelistfn = "../head_tracking/images_sample/sample.xml"; //sample data from opencv
		//boardSize = Size(9, 6); //sample data from opencv
    }
    else if( boardSize.width <= 0 || boardSize.height <= 0 )
    {
        cout << "if you specified XML file with chessboards, you should also specify the board width and height (-w and -h options)" << endl;
        return 0;
    }

	vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }

   StereoCalib(imagelist, boardSize, true, showRectified);
	return 0;
}


