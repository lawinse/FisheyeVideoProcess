#include "..\Config.h"
#include <opencv2\stitching\detail\warpers.hpp>
#include <opencv2\stitching.hpp>

#pragma once



namespace supp{
	struct ResultRoi {
			Size srcSz;
			Rect roi;
			ResultRoi(Size _sz, Rect _roi):srcSz(_sz),roi(_roi){};
		};
	struct PlaneLinearTransformHelper {
#define PLT_ADJUST_TYPE 0	//0->BOTH, 1->X_AXIS, 2->Y_AXIS
		float ax,bx,ay,by;
		PlaneLinearTransformHelper(float _ax=1.0, float _bx=0.0, float _ay=1.0, float _by=0.0)
			:ax(_ax), ay(_ay), bx(_bx), by(_by) {};
		inline void transformForward(float x, float y, float &u, float &v) {
			u = ax*x+bx, v = ay*y+by;
		}
		inline void transformBackward(float x, float y, float &u, float &v) {
			u = (x-bx)/ax, v = (y-by)/ay;
		}
		static PlaneLinearTransformHelper calcPLT(Rect &rectBase, Rect &rectIn) {
			float ax,bx,ay,by;
			ax = rectBase.width/float(rectIn.width);
			ay = rectBase.height/float(rectIn.height);
			bx = rectBase.x - rectIn.x*ax;
			by = rectBase.y - rectIn.y*ay;
	#if PLT_ADJUST_TYPE == 0
			return PlaneLinearTransformHelper(1,100,1,-200);
	#elif PLT_ADJUST_TYPE == 1
			return PlaneLinearTransformHelper(ax,bx,1,0);
	#elif PLT_ADJUST_TYPE == 2
			return PlaneLinearTransformHelper(1,0,ay,by);
	#else
			return PPlaneLinearTransformHelper();
	#endif
		}
		bool operator == (const PlaneLinearTransformHelper &obj) const {
			return ax == obj.ax && ay == obj.ay && bx == obj.bx && by == obj.by;
		}
		bool operator != (const PlaneLinearTransformHelper &obj) const {return !((*this)==obj);}

	};


// Spherical, Cylinderical, Mercator
	struct _ProjectorBase : public cv::detail::ProjectorBase {
	#define PROJ_DATA_TTL_LEN 40

		bool b_setAllMatsData;
		int ttlProjTime;
		int curProjIdx;
		std::vector<std::vector<float>> projData;

		PlaneLinearTransformHelper pltHelper;

		void autoSaveProjData();
		void autoSaveMapData();
		bool isSetAllMatsData() const {return b_setAllMatsData;}
		void setAllMatsData(bool b) {b_setAllMatsData = b;}

		_ProjectorBase():curProjIdx(0){setAllMatsData(false);}

		_ProjectorBase(std::vector<std::vector<float>> &_data):curProjIdx(0){
			setAllMatsMultiple(_data);
		}
		void setAllMatsMultiple(std::vector<std::vector<float>> &_data);
		std::vector<std::vector<float>>getAllMatsMultiple();

		void setAllMats(std::vector<float>&);
		std::vector<float> getAllMats();

		void setCameraParams(InputArray K = Mat::eye(3, 3, CV_32F),
							 InputArray R = Mat::eye(3, 3, CV_32F),
							 InputArray T = Mat::zeros(3, 1, CV_32F));
		inline void mapForward(float x, float y, float &u, float &v) {
			pltHelper.transformForward(x,y,u,v);
		}
		inline void mapBackward(float x, float y, float &u, float &v) {
			pltHelper.transformBackward(x,y,u,v);
		}

	
	};

	struct _SphericalProjector : _ProjectorBase {
		inline void _SphericalProjector::mapForward(float x, float y, float &u, float &v) {
			float x_ = r_kinv[0] * x + r_kinv[1] * y + r_kinv[2];
			float y_ = r_kinv[3] * x + r_kinv[4] * y + r_kinv[5];
			float z_ = r_kinv[6] * x + r_kinv[7] * y + r_kinv[8];

			u = scale * atan2f(x_, z_);
			float w = y_ / sqrtf(x_ * x_ + y_ * y_ + z_ * z_);
			v = scale * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));
			_ProjectorBase::mapForward(u,v,u,v);
		}

		inline void _SphericalProjector::mapBackward(float u, float v, float &x, float &y) {
			_ProjectorBase::mapBackward(u,v,u,v);
			u /= scale;
			v /= scale;

			float sinv = sinf(static_cast<float>(CV_PI) - v);
			float x_ = sinv * sinf(u);
			float y_ = cosf(static_cast<float>(CV_PI) - v);
			float z_ = sinv * cosf(u);

			float z;
			x = k_rinv[0] * x_ + k_rinv[1] * y_ + k_rinv[2] * z_;
			y = k_rinv[3] * x_ + k_rinv[4] * y_ + k_rinv[5] * z_;
			z = k_rinv[6] * x_ + k_rinv[7] * y_ + k_rinv[8] * z_;

			if (z > 0) { x /= z; y /= z;}
			else x = y = -1;
		}
	};

	template <class P>
	class RewarpableRotationWarperBase : public cv::detail::RotationWarperBase<P> {
	public:
		std::vector<ResultRoi> resultRoiData;

		std::vector<PlaneLinearTransformHelper> plts;
		int curBuildMapsTime;
		RewarpableRotationWarperBase():curBuildMapsTime(INT_MAX){}

		void setPLTs(std::vector<PlaneLinearTransformHelper>& _plts) {plts.assign(_plts.begin(), _plts.end());curBuildMapsTime = 0;}

		void setProjectorData(Mat &dmat) {
			std::vector<std::vector<float>> d = std::vector<std::vector<float>>(dmat.rows, std::vector<float>(dmat.cols));
			for (int i=0; i<dmat.rows; ++i)
				for (int j=0; j<dmat.cols; ++j)
					 d[i][j] = dmat.at<float>(i,j);
			projector_.setAllMatsMultiple(d);
		}
		Mat getProjectorAllData() {
			auto d = projector_.getAllMatsMultiple();
			Mat dmat = Mat::zeros(d.size(), d[0].size(), CV_32F);
			for (int i=0; i<d.size(); ++i)
				for (int j=0; j<d[0].size(); ++j)
					dmat.at<float>(i,j) = d[i][j];
			return dmat;
		}

		std::vector<ResultRoi> getResultRoiData() {return resultRoiData;}	
	    void detectResultRoi(Size src_size, Point &dst_tl, Point &dst_br) {
			if (curBuildMapsTime < plts.size()) {
				//LOG_MESS("RewarpableRotationWarperBase.detectResultRoi: Manually set pltHelper.")
				projector_.pltHelper = plts[curBuildMapsTime]; 
				curBuildMapsTime++;
			}
			RotationWarperBase<P>::detectResultRoi(src_size, dst_tl, dst_br);
			resultRoiData.push_back(ResultRoi(src_size,Rect(dst_tl, dst_br)));
		}
		void detectResultRoiByBorder(Size src_size, Point &dst_tl, Point &dst_br) {
			if (curBuildMapsTime < plts.size()) {
				//LOG_MESS("RewarpableRotationWarperBase.detectResultRoiByBorder: Manually set pltHelper.")
				projector_.pltHelper = plts[curBuildMapsTime]; 
				curBuildMapsTime++;
			}
			RotationWarperBase<P>::detectResultRoiByBorder(src_size, dst_tl, dst_br);
			resultRoiData.push_back(ResultRoi(src_size,Rect(dst_tl, dst_br)));
		}
	};


	class RewarpableSphericalWarper : public RewarpableRotationWarperBase<_SphericalProjector> {
	public:
		RewarpableSphericalWarper(float scale) {projector_.scale = scale;}
		//Rect buildMaps(Size src_size, InputArray K, InputArray R, OutputArray xmap, OutputArray ymap) {
		//	return RewarpableRotationWarperBase<_SphericalProjector>::buildMaps(src_size,K,R,xmap,ymap);
		//}

		Point warp(InputArray src, InputArray K, InputArray R, int interp_mode, int border_mode, OutputArray dst);
	protected:
		void detectResultRoi(Size src_size, Point &dst_tl, Point &dst_br);
	};

	struct _CylindricalProjector : _ProjectorBase {
		inline void _CylindricalProjector::mapForward(float x, float y, float &u, float &v) {
			float x_ = r_kinv[0] * x + r_kinv[1] * y + r_kinv[2];
			float y_ = r_kinv[3] * x + r_kinv[4] * y + r_kinv[5];
			float z_ = r_kinv[6] * x + r_kinv[7] * y + r_kinv[8];

			u = scale * atan2f(x_, z_);
			v = scale * y_ / sqrtf(x_ * x_ + z_ * z_);

			_ProjectorBase::mapForward(u,v,u,v);

		}


		 void _CylindricalProjector::mapBackward(float u, float v, float &x, float &y) {
			_ProjectorBase::mapBackward(u,v,u,v);
			u /= scale;
			v /= scale;

			float x_ = sinf(u);
			float y_ = v;
			float z_ = cosf(u);

			float z;
			x = k_rinv[0] * x_ + k_rinv[1] * y_ + k_rinv[2] * z_;
			y = k_rinv[3] * x_ + k_rinv[4] * y_ + k_rinv[5] * z_;
			z = k_rinv[6] * x_ + k_rinv[7] * y_ + k_rinv[8] * z_;

			if (z > 0) { x /= z; y /= z; }
			else x = y = -1;
			
		}
	};


	class RewarpableCylindricalWarper : public RewarpableRotationWarperBase<_CylindricalProjector> {
	public:
		RewarpableCylindricalWarper(float scale) { projector_.scale = scale; }

		//Rect buildMaps(Size src_size, InputArray K, InputArray R, OutputArray xmap, OutputArray ymap) {
		//	return RewarpableRotationWarperBase<_CylindricalProjector>::buildMaps(src_size,K,R,xmap,ymap);
		//}

		Point warp(InputArray src, InputArray K, InputArray R, int interp_mode, int border_mode, OutputArray dst);
	protected:
		void detectResultRoi(Size src_size, Point &dst_tl, Point &dst_br) {
			RewarpableRotationWarperBase<_CylindricalProjector>::detectResultRoiByBorder(src_size, dst_tl, dst_br);
		}
	};

	struct _MercatorProjector : _ProjectorBase {
		inline void _MercatorProjector::mapForward(float x, float y, float &u, float &v) {
			float x_ = r_kinv[0] * x + r_kinv[1] * y + r_kinv[2];
			float y_ = r_kinv[3] * x + r_kinv[4] * y + r_kinv[5];
			float z_ = r_kinv[6] * x + r_kinv[7] * y + r_kinv[8];

			float u_ = atan2f(x_, z_);
			float v_ = asinf(y_ / sqrtf(x_ * x_ + y_ * y_ + z_ * z_));

			u = scale * u_;
			v = scale * logf( tanf( (float)(CV_PI/4) + v_/2 ) );
			_ProjectorBase::mapForward(u,v,u,v);
		}

		inline void _MercatorProjector::mapBackward(float u, float v, float &x, float &y) {
			_ProjectorBase::mapBackward(u,v,u,v);
			u /= scale;
			v /= scale;

			float v_ = atanf( sinhf(v) );
			float u_ = u;

			float cosv = cosf(v_);
			float x_ = cosv * sinf(u_);
			float y_ = sinf(v_);
			float z_ = cosv * cosf(u_);

			float z;
			x = k_rinv[0] * x_ + k_rinv[1] * y_ + k_rinv[2] * z_;
			y = k_rinv[3] * x_ + k_rinv[4] * y_ + k_rinv[5] * z_;
			z = k_rinv[6] * x_ + k_rinv[7] * y_ + k_rinv[8] * z_;

			if (z > 0) { x /= z; y /= z; }
			else x = y = -1;
		}
	};


	class RewarpableMercatorWarper : public RewarpableRotationWarperBase<_MercatorProjector>{
	public:
		RewarpableMercatorWarper(float scale) {projector_.scale = scale;}
	};

		class CylindricalWarper: public cv::WarperCreator {
		public:
			Ptr<detail::RotationWarper> create(float scale) const { return makePtr<detail::CylindricalWarper>(scale); }
			Ptr<RewarpableCylindricalWarper> create_(float scale) const { return makePtr<RewarpableCylindricalWarper>(scale); }
		};

		class SphericalWarper: public cv::WarperCreator {
		public:
			Ptr<detail::RotationWarper> create(float scale) const { return makePtr<detail::SphericalWarper>(scale); }
			Ptr<RewarpableSphericalWarper> create_(float scale) const { return makePtr<RewarpableSphericalWarper>(scale); }
		};

		class MercatorWarper: public cv::WarperCreator {
		public:
			Ptr<detail::RotationWarper> create(float scale) const { return makePtr<detail::MercatorWarper>(scale); }
			Ptr<RewarpableMercatorWarper> create_(float scale) const { return makePtr<RewarpableMercatorWarper>(scale); }
		};

}