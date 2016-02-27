#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cvaux.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/video/tracking.hpp"


//#include "opencv2/core/core.hpp"
//#include "opencv2/core/core_c.h"
//#include "opencv2/highgui/highgui_c.h"
//#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/nonfree/nonfree.hpp"
//#include "opencv2/videostab/videostab.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/flann/miniflann.hpp"
//#include "opencv2/photo/photo.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/ml/ml.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/ts/ts.hpp"
//#include "opencv2/stitching/stitcher.hpp"
//#include "opencv2/legacy/legacy.hpp"


#ifdef _DEBUG

#pragma comment(lib,"opencv_ts300d.lib")
#pragma comment(lib,"opencv_world300d.lib")

#else

#pragma comment(lib,"opencv_ts300.lib")
#pragma comment(lib,"opencv_world300.lib")
#endif
