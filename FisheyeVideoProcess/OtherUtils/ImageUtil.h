#pragma once
#include "..\Config.h"
class ImageUtil {
	#define BLACK_TOLERANCE 3
private:
	static bool checkInterior(const Mat& mask, const Rect& interiorBB, bool& top, bool& bottom, bool& left, bool& right) {
		bool ret = true;
		Mat sub = mask(interiorBB);
		int x=0,y=0;
		int _top = 0, _bottom = 0, _left = 0, _right = 0;

		for (; x<sub.cols; ++x) {
			if (sub.at<unsigned char>(y,x) <= BLACK_TOLERANCE) {
				ret = false;
				++_top;
			}
		}
		for (y=sub.rows-1, x=0; x<sub.cols; ++x) {
			if (sub.at<unsigned char>(y,x) <= BLACK_TOLERANCE) {
				ret = false;
				++_bottom;
			}
		}
		for (x=0,y=0; y<sub.rows; ++y) {
			if (sub.at<unsigned char>(y,x) <= BLACK_TOLERANCE) {
				ret = false;
				++_left;
			}
		}
		for (x=sub.cols-1,y=0; y<sub.rows; ++y) {
			if (sub.at<unsigned char>(y,x) <= BLACK_TOLERANCE) {
				ret = false;
				++_right;
			}
		}

		if (_top > _bottom) {
			top = (_top > _left && _top > _right);
		} else {
			bottom = (_bottom > _left && _bottom > _right);
		}

		if (_left >= _right) {
			left = (_left >= _bottom && _left >= _top);
		} else {
			right = (_right >= _top && _right >= _bottom);
		}
		return ret;
	}

public:
	static bool almostBlack(const Vec3b &v) {
		const int tolerance = square(3*BLACK_TOLERANCE);
		return v[0]*v[0] + v[1]*v[1] + v[2]*v[2] <= tolerance;
	}
	// REF: http://stackoverflow.com/questions/21410449/how-do-i-crop-to-largest-interior-bounding-box-in-opencv
	static bool removeBlackPixelByContourBound(Mat &src, Mat &dst, Rect &interiorBoundingBox) {
		Mat grayscale;
		cvtColor(src, grayscale, CV_BGR2GRAY);
		Mat mask = grayscale > BLACK_TOLERANCE;

		std::vector<std::vector<Point>> contours;
		std::vector<Vec4i> hierarchy;

		findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
		Mat contourImg = Mat::zeros(src.size(), CV_8UC3);

		int maxSz = 0, id = 0;
		for (int i=0; i<contours.size(); ++i) {
			if (contours.at(i).size() > maxSz) {
				maxSz = contours.at(i).size();
				id = i;
			}
		}

		Mat contourMask = cv::Mat::zeros(src.size(), CV_8UC1);
		drawContours(contourMask, contours, id, Scalar(255), -1, 8, hierarchy,0,Point());

		std::vector<Point> cSortedX = contours.at(id);
		std::sort(cSortedX.begin(), cSortedX.end(), [](const Point &a, const Point &b){return a.x<b.x;});
		std::vector<Point> cSortedY = contours.at(id);
		std::sort(cSortedY.begin(), cSortedY.end(), [](const Point &a, const Point &b){return a.y<b.y;});
		int minX = 0, maxX = cSortedX.size()-1;
		int minY = 0, maxY = cSortedY.size()-1;


		while(minX < maxX && minY < maxY) {
			Point minP(cSortedX[minX].x, cSortedY[minY].y);
			Point maxP(cSortedX[maxX].x, cSortedY[maxY].y);
			interiorBoundingBox = Rect(minP.x, minP.y, maxP.x-minP.x, maxP.y-minP.y);
			// out-codes
			bool ocTop = false, ocBottom = false, ocLeft = false, ocRight = false;

			if (checkInterior(contourMask, interiorBoundingBox, ocTop, ocBottom, ocLeft, ocRight))
				break;
			if (ocLeft) ++minX;
			if (ocRight)--maxX;
			if (ocTop) ++minY;
			if (ocBottom) --maxY;
			if (!(ocLeft || ocRight || ocTop || ocBottom)) {
				LOG_WARN("removeBlackPixelByContourBound() failed.");
				return false;
			}
		}
		double restRatioPercent = interiorBoundingBox.size().area()*1.0/src.size().area();
		dst = src(interiorBoundingBox).clone();
		return true;
	}


	/* USM sharpening process */
	static void USM(Mat &src, Mat &dst) {
		Mat blur, tmp2; 
		double amount=1.0;
		int thres=0;
		double sigma=3;

		GaussianBlur(src,blur,Size(),sigma, sigma);
		Mat lowContratsMask = abs(src-blur)<thres;
		tmp2 = src*(1+amount)+blur*(-amount);
		src.copyTo(tmp2, lowContratsMask);
		dst = tmp2;
	}

	/* Laplace enhancement process */
	static void LaplaceEnhannce(Mat &src, Mat &dst) {
		Mat k = (Mat_<int>(3,3) <<0,-1,0,-1,5,-1,0,-1,0);
		filter2D(src,dst,src.depth(),k);
	}

	static void brightnessAndContrastAuto(Mat &src, Mat &dst, float clipHistPercent=5) {

		CV_Assert(clipHistPercent >= 0);
		CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

		int histSize = 256;
		float alpha, beta;
		double minGray = 0, maxGray = 0;

		//to calculate grayscale histogram
		cv::Mat gray;
		if (src.type() == CV_8UC1) gray = src;
		else if (src.type() == CV_8UC3) cvtColor(src, gray, CV_BGR2GRAY);
		else if (src.type() == CV_8UC4) cvtColor(src, gray, CV_BGRA2GRAY);
		if (clipHistPercent == 0) {
			// keep full available range
			cv::minMaxLoc(gray, &minGray, &maxGray);
		} else {
			cv::Mat hist; //the grayscale histogram

			float range[] = { 0, 256 };
			const float* histRange = { range };
			bool uniform = true;
			bool accumulate = false;
			calcHist(&gray, 1, 0, cv::Mat (), hist, 1, &histSize, &histRange, uniform, accumulate);

			// calculate cumulative distribution from the histogram
			std::vector<float> accumulator(histSize);
			accumulator[0] = hist.at<float>(0);
			for (int i = 1; i < histSize; i++)
				accumulator[i] = accumulator[i - 1] + hist.at<float>(i);

			// locate points that cuts at required value
			float max = accumulator.back();
			clipHistPercent *= (max / 100.0); //make percent as absolute
			clipHistPercent /= 2.0; // left and right wings
			// locate left cut
			minGray = 0;
			while (accumulator[minGray] < clipHistPercent)
				minGray++;

			// locate right cut
			maxGray = histSize - 1;
			while (accumulator[maxGray] >= (max - clipHistPercent))
				maxGray--;
		}

		// current range
		float inputRange = maxGray - minGray;

		alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
		beta = -minGray * alpha;             // beta shifts current range so that minGray will go to 0

		// Apply brightness and contrast normalization
		// convertTo operates with saurate_cast
		src.convertTo(dst, -1, alpha, beta);

		// restore alpha channel from source 
		if (dst.type() == CV_8UC4) {
			int from_to[] = { 3, 3};
			cv::mixChannels(&src, 4, &dst,1, from_to, 1);
		}
		return;
	}

	/* Create an all-black 3-channels Mat of given size and type */
	static Mat createDummyMatRGB(Size sz, int src_type) {
		Mat m(sz, src_type,Scalar(255,255,255));
		return m;
	}

	/* Auto-adjust resize op */
	static void ImageUtil::resize(InputArray src, OutputArray dst, Size dsize, double fx = 0, double fy = 0) {
		if (dsize.area() == 0 && fx == 0 && fy == 0) {
			if (dsize.height == 0 && dsize.width == 0)
				dsize = src.size();
			else if (dsize.height != 0) {
				double ratio = dsize.height*1.0/src.size().height;
				dsize.width = src.size().width*ratio;
			}
		}
		src.size().area() > dsize.area()
			? cv::resize(src, dst, dsize, fx, fy, CV_INTER_AREA)
			: cv::resize(src, dst, dsize, fx, fy, CV_INTER_CUBIC);
	}

	inline static void ImageUtil::resizeHeightBased(Mat &src, Mat &dst, int height) {
		double ratio = height*1.0/src.size().height;
		int width = src.size().width*ratio;
		ImageUtil::resize(src,dst,Size(width,height));
	}

	static void batchResize(std::vector<Mat> &srcs, std::vector<Mat> &dsts, Size dsize) {
		dsts.resize(srcs.size());
		for (int i=0; i<srcs.size(); ++i) {
			ImageUtil::resize(srcs[i],dsts[i],dsize);
		}
	}

	static void batchResizeHeightBased(std::vector<Mat> &srcs, std::vector<Mat> &dsts, int height) {
		dsts.resize(srcs.size());
		for (int i=0; i<srcs.size(); ++i) {
			ImageUtil::resizeHeightBased(srcs[i],dsts[i],height);
		}
	}


	/* More conveneient way to use cv::imshow */
	static void imshow(char * winName, Mat img, double ratio = 1.0, bool holdon = false) {
		Mat tmp;
		ImageUtil::resize(img,tmp,Size(img.cols*ratio, img.rows*ratio));
		cv::imshow(winName, tmp);
		if (holdon) cvWaitKey();
	}
	static void imshow(char * winName, Mat img, Size dsize, double ratio = 1.0, bool holdon = false) {
		Mat tmp;
		ImageUtil::resize(img,tmp,dsize);
		imshow(winName,tmp,ratio,holdon);
	}


	/* BGR-Equalization process */
	static void equalizeHistBGR(Mat &src, Mat &dst) {
		if (src.channels()<3) {
			LOG_WARN("ImageUtil: failed in equalizeHistBGR (less than 3 channels)");
			dst = src.clone();
		} else {
			cvtColor(src, dst, COLOR_BGR2YCrCb);
			std::vector<Mat> channels;
			cv::split(dst, channels);
			cv::equalizeHist(channels[0], channels[0]);
			cv::merge(channels,dst);
			cvtColor(dst,dst,COLOR_YCrCb2BGR);
		}
	}

	/* Batch ops supported */
	static void batchOperation(std::vector<Mat> &srcs, std::vector<Mat> &dsts, void (*op)(Mat &src, Mat &dst)) {
		if (dsts.size() != srcs.size()) dsts = std::vector<Mat>(srcs.size());
		for (int i=0; i<srcs.size(); ++i) op(srcs[i],dsts[i]);
	}


};
