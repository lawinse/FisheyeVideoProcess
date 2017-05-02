#include "CorrectingUtil.h"

void CorrectingUtil::basicCorrecting(Mat &srcImage, Mat &dstImage, CorrectingType ctype) {
	assert(ctype <= BASIC_REVERSED);
	int col, row, u0, v0, R, i, j, u, v, i_dst, j_dst;//�С��� 
	double f, r, alpha, theta, x, y, z, r_xoz, phi, lambda;
	//Vec3b pix;

	//imshow("ԭͼ", srcImage);
	col = srcImage.cols; //������x��
	row = srcImage.rows; //������y��

	u0 = round(col / 2);
	v0 = round(row / 2);	//����
	R = col - u0;	//���ΰ뾶
	f = 2 * R / (180 * M_PI / 180);	//�ӳ���235��???

	//��ֵ��������
	switch (ctype)
	{
	case BASIC_FORWARD:
		for (i = 0; i < row; i++)	//�б���
		{
			for (j = 0; j < col; j++)	//�б���
			{
				u = j - u0;		//������,ԭ����Բ��
				v = v0 - i;		//������,ԭ����Բ��
				r = sqrt(pow(u, 2) + pow(v, 2));	//��Բ�ĵľ���
				if (r > R)		//�����߽�
				{
					continue;
				}
				//��alpha��
				if (r == 0)
				{
					alpha = 0;
				}
				else if (u > 0)
				{
					alpha = asin(v / r);	//��atan2(v,u) or asin(v/r)���ԣ���atan(v/u)����
				}
				else if (u < 0)
				{
					alpha = M_PI - asin(v / r);
				}
				else if (u == 0 && v>0)
				{
					alpha = M_PI / 2;
				}
				else
					alpha = 3 * M_PI / 2;

				//��theta��
				theta = r / f;
				//��ӳ�䵽�����������
				x = f*sin(theta)*cos(alpha);
				y = f*sin(theta)*sin(alpha);
				z = f*cos(theta);
				//��γ������
				r_xoz = sqrt(pow(x, 2) + pow(z, 2));
				phi = M_PI / 2 - atan(y / r_xoz);	//γ��
				if (z >= 0)
				{
					lambda = M_PI - acos(x / r_xoz);	//����
				}
				i_dst = round(f*phi);	//�����
				j_dst = round(f*lambda);	//�����
				dstImage.at<Vec3b>(i_dst, j_dst)[0] = srcImage.at<Vec3b>(i, j)[0];
				dstImage.at<Vec3b>(i_dst, j_dst)[1] = srcImage.at<Vec3b>(i, j)[1];
				dstImage.at<Vec3b>(i_dst, j_dst)[2] = srcImage.at<Vec3b>(i, j)[2];
				pixelReMapping.set(std::make_pair(i,j), std::make_pair(i_dst,j_dst));

			}
		}
		break;
	case BASIC_REVERSED:
		for (i_dst = 0; i_dst < row; i_dst++)	//�б���
		{
			for (j_dst = 0; j_dst < col; j_dst++)	//�б���
			{
				//��γ��
				phi = i_dst / f;
				lambda = j_dst / f;
				//���ݾ�γ����γֱ������
				x = f*(-1)*sin(phi)*cos(lambda);
				y = f*cos(phi);
				z = f*sin(phi)*sin(lambda);
				//��theta��r��alpha(��㡢Բ��������z��н�theta��Բ�뾶r����x��н�alpha)
				theta = acos(z / f);
				r = f*theta;
				if (theta == 0)
				{
					alpha = 0;
				}
				else if (x > 0)
				{
					alpha = atan(y / x);
				}
				else if (x < 0)
				{
					alpha = atan(y / x) + M_PI;
				}
				else if (x == 0 && y>0)
				{
					alpha = M_PI / 2;
				}
				else
				{
					alpha = 3 * M_PI / 2;
				}
				//��ӳ�䵽Բ�ϵĵ�
				u = round(r*cos(alpha));
				v = r*sin((alpha));
				//����ת������������
				i = v0 - v;
				j = u + u0;

				//��ֵ
				dstImage.at<Vec3b>(i_dst, j_dst)[0] = srcImage.at<Vec3b>(i, j)[0];
				dstImage.at<Vec3b>(i_dst, j_dst)[1] = srcImage.at<Vec3b>(i, j)[1];
				dstImage.at<Vec3b>(i_dst, j_dst)[2] = srcImage.at<Vec3b>(i, j)[2];
				pixelReMapping.set(std::make_pair(i,j), std::make_pair(i_dst,j_dst));

			}
		}
		break;
	default:
		assert(false);
	}
}

void CorrectingUtil::doCorrect(Mat &srcImage, Mat &dstImage, CorrectingParams cParams) {
	assert(srcImage.cols == srcImage.rows);		// Ensure to be a square
	//assert(srcImage.size() == dstImage.size());

	bool needPersistReMap = false;
	if (cParams.use_reMap && !pixelReMapping.isMapped()) {
		if (!pixelReMapping.load(cParams.hashcode()))
			needPersistReMap = true;
		else
			_cParams = cParams;
	}

	if (cParams.use_reMap && pixelReMapping.isMapped() && cParams == _cParams) {
		if (pixelReMapping.reMap(srcImage, dstImage)) return;
	}

	switch (cParams.ctype) {
	case BASIC_FORWARD:
	case BASIC_REVERSED:
		basicCorrecting(srcImage, dstImage, cParams.ctype);
		break;
	case LONG_LAT_MAPPING_FORWARD:
	case LONG_LAT_MAPPING_REVERSED:
		LLMCorrecting(srcImage, dstImage, cParams.centerOfCircle, cParams.radiusOfCircle, cParams.ctype);
		break;
	case PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_FORWARD:
		PLLMCLMCorrentingForward(
			srcImage, dstImage, cParams.centerOfCircle, cParams.radiusOfCircle, cParams.dmType);
		break;
	case PERSPECTIVE_LONG_LAT_MAPPING_CAM_LENS_MOD_REVERSED:
		PLLMCLMCorrentingReversed(
			srcImage, dstImage, cParams.centerOfCircle, cParams.radiusOfCircle, cParams.dmType);
		break;
	case LONG_LAT_MAPPING_CAM_LENS_MOD_UNFIXED_FORWARD:
	case LONG_LAT_MAPPING_CAM_LENS_MOD_UNFIXED_REVERSED:
		LLMCLMUFCorrecting(srcImage, dstImage, cParams.centerOfCircle, cParams.radiusOfCircle, cParams.ctype, cParams.w);
		break;
	default:
		assert(false);
	}
	_cParams = cParams;
	if (needPersistReMap) pixelReMapping.persist(cParams.hashcode());
}

// LONG_LON_MAPPING
void CorrectingUtil::LLMCorrecting(
	Mat &srcImage, Mat &dstImage, Point2i center, int radius, CorrectingType ctype) {
	assert(ctype == LONG_LAT_MAPPING_FORWARD || ctype == LONG_LAT_MAPPING_REVERSED);

	double dx = camFieldAngle / srcImage.cols; // srcImage.cols should be the same as srcImage.size().width
	double dy = dx;
	double f = radius/(camFieldAngle/2);	// equal-distance projection 

	double lat, lon;
	double x,y,z,r;
	double theta_sphere, phi_sphere;
	double p_pol, theta_pol;
	double x_cart, y_cart;
	

	int u_src, v_src, u_dst, v_dst;

	double lon_offset = (PI - camFieldAngle) / 2, lat_offset = (PI - camFieldAngle) / 2;
	int left,top;

	switch (ctype) {
	// @Deprecated
	case LONG_LAT_MAPPING_FORWARD:
		left = center.x - radius; assert(left == 0);
		top = center.y - radius; assert(top == 0);

		for (int j=top; j<top+2*radius; ++j)
			for (int i=left; i<left+2*radius; ++i) {
				if (square(i-center.x) + square(j-center.y) > square(radius)) continue;

				u_src = i, v_src = j;

				/* Coord Tranform */
				x_cart = u_src - center.x;
				y_cart = center.y - v_src;

				theta_pol = cvFastArctan(y_cart, x_cart)*PI/180;
				p_pol = sqrt(square(x_cart) + square(y_cart));

				phi_sphere = theta_pol;
				theta_sphere = p_pol/f;		// equal-distance projection 

				x = sin(theta_sphere)*cos(phi_sphere);
				y = sin(theta_sphere)*sin(phi_sphere);
				z = cos(theta_sphere);

				lat = acos(y);
				lon = cvFastArctan(z,-x)*PI/180;

				u_dst = (lon-lon_offset)/dx;
				v_dst = (lat-lat_offset)/dy;

				if (u_dst < 0 || u_dst >= dstImage.rows || v_dst < 0 || v_dst >= dstImage.cols)
					continue;

				dstImage.at<Vec3b>(v_dst,u_dst)[0] = srcImage.at<Vec3b>(j,i)[0];
				dstImage.at<Vec3b>(v_dst,u_dst)[1] = srcImage.at<Vec3b>(j,i)[1];
				dstImage.at<Vec3b>(v_dst,u_dst)[2] = srcImage.at<Vec3b>(j,i)[2];
				pixelReMapping.set(std::make_pair(j,i), std::make_pair(v_dst,u_dst));
			}
		break;
	case LONG_LAT_MAPPING_REVERSED:
		for (int j=0; j<dstImage.rows; ++j) {
			lat = lat_offset + j*dy;
			for (int i=0; i<dstImage.cols; ++i) {
				lon = lon_offset + i*dx;

				/* Corrd Tranform */
				// lat-lon -->> sphere
				x = -sin(lat)*cos(lon);
				y = cos(lat);
				z = sin(lat)*sin(lon);

				//sphere -->> theta-phi
				theta_sphere = acos(z);
				phi_sphere = cvFastArctan(y,x)*PI/180;

				//theta-phi -->> fisheye polar
				p_pol = theta_sphere*f;		// equal-distance projection 
				theta_pol = phi_sphere;

				//fisheye polar -->> x,y in cart plane
				x_cart = p_pol*cos(theta_pol);
				y_cart = p_pol*sin(theta_pol);

				u_src = center.x + x_cart;
				v_src = center.y - y_cart;

				if (u_src < 0 || u_src >= srcImage.rows || v_src < 0 || v_src >= srcImage.cols)
					continue;

				dstImage.at<Vec3b>(j,i)[0] = srcImage.at<Vec3b>(v_src,u_src)[0];
				dstImage.at<Vec3b>(j,i)[1] = srcImage.at<Vec3b>(v_src,u_src)[1];
				dstImage.at<Vec3b>(j,i)[2] = srcImage.at<Vec3b>(v_src,u_src)[2];
				pixelReMapping.set(std::make_pair(v_src,u_src), std::make_pair(j,i));
			}
		}
		break;
	default:
		assert(false);
	}
}

// Pespective LLM Forward
void CorrectingUtil::PLLMCLMCorrentingForward(Mat &srcImage, Mat &dstImage, Point2i center, int radius, DistanceMappingType dmType) {
	double dx = camFieldAngle / srcImage.cols; 
	double dy = dx;
	double f = radius/(camFieldAngle/2);	// equal-distance projection 

	double lat, lon;
	double x,y,z,r;
	double theta_sphere, phi_sphere;
	double p_pol, theta_pol;
	double x_cart, y_cart;
	
	int u_src, v_src, u_dst, v_dst;
	double lon_offset = (PI - camFieldAngle) / 2, lat_offset = (PI - camFieldAngle) / 2;
	double mo;

	for (int j=0; j<srcImage.rows; ++j)
		for (int i=0; i<srcImage.cols; ++i) {
			switch (dmType) {
			case LONG_LAT:
				lat = lat_offset+j*dy;

				lon = lon_offset+i*dx;

				x = -sin(lat)*cos(lon);
				y = cos(lat);
				z = sin(lat)*sin(lon);
				break;

			case PERSPECTIVE:
				x = i-center.x;
				y = center.y-j;
				z = focusLen;
				mo = sqrt(square(x) + square(y) + square(z));
				x /= mo;
				y /= mo;
				z /= mo;


				break;
			default:
				assert(false);
			}


			theta_sphere = acos(z);
			phi_sphere = cvFastArctan(y,x)*PI/180;

			p_pol = f*theta_sphere;
			theta_pol = phi_sphere;

			x_cart = p_pol*cos(theta_pol);
			y_cart = p_pol*sin(theta_pol);

			u_src = x_cart + center.x;
			v_src = -y_cart + center.y;

			if (u_src < 0 || u_src >= srcImage.rows || v_src < 0 || v_src >= srcImage.cols)
					continue;

			dstImage.at<Vec3b>(j,i)[0] = srcImage.at<Vec3b>(v_src,u_src)[0];
			dstImage.at<Vec3b>(j,i)[1] = srcImage.at<Vec3b>(v_src,u_src)[1];
			dstImage.at<Vec3b>(j,i)[2] = srcImage.at<Vec3b>(v_src,u_src)[2];
			pixelReMapping.set(std::make_pair(v_src,u_src), std::make_pair(j,i));
		}
}

double CorrectingUtil::getPhiFromV(double v) {
	double l = abs(2-v);
	return (v>2) ? PI-asin(8/(square(l)+4)-1) : asin(8/(square(l)+4)-1);   // derived by simplification
}

void CorrectingUtil::rotateEarth(double &x, double &y, double &z) {
	Mat curP(Point3f(x,y,z));
	std::vector<Point3f> pts;

	//TOSOLVE: Why rotate the earth east-wards and south-wards
	const double theta_left = 0;
	const double phi_up = 0;
	pts.push_back(Point3f(cos(theta_left),0,sin(theta_left)));
	pts.push_back(Point3f(sin(phi_up)*sin(theta_left), cos(phi_up), -sin(phi_up)*cos(theta_left)));
	pts.push_back(Point3f(-cos(phi_up)*sin(theta_left), sin(phi_up), cos(phi_up)*cos(theta_left)));

	Mat tmp(Mat(pts).reshape(1).t()*curP);
	Mat_<double> tmpIte;
	tmp.convertTo(tmpIte, CV_64F);

	x = tmpIte.at<double>(0,0);
	y = tmpIte.at<double>(1,0);
	z = tmpIte.at<double>(2,0);
}

void CorrectingUtil::PLLMCLMCorrentingReversed(
	Mat &srcImage, Mat &dstImage, Point2i center, int radius, DistanceMappingType dmtype) {
	double dx = camFieldAngle / srcImage.cols; 
	double dy = dx;
	double f = radius/(camFieldAngle/2);	// equal-distance projection  

	double lat, lon;
	double x,y,z,r;
	double theta_sphere, phi_sphere;
	double p_pol, theta_pol;
	double x_cart, y_cart;
	
	int u_src, v_src, u_dst, v_dst;
	double lon_offset = (PI - camFieldAngle) / 2, lat_offset = (PI - camFieldAngle) / 2;
	double mo;


	for (int j=0; j<dstImage.rows; ++j)
		for (int i=0; i<dstImage.cols; ++i) {
			//std::cout << i << " " << j << std::endl;
			switch (dmtype) {
			case LONG_LAT:
				lat = getPhiFromV((double)j*4.0/dstImage.rows);
				//lon = getPhiFromV((double)i*4.0/dstImage.cols);
				//lat = lat_offset+j*dy;
				lon = lon_offset+i*dx;

				x = -sin(lat)*cos(lon);
				y = cos(lat);
				z = sin(lat)*sin(lon);
				break;

			case PERSPECTIVE:
				x = i-center.x;
				y = center.y-j;
				z = focusLen;
				mo = sqrt(square(x) + square(y) + square(z));
				x /= mo;
				y /= mo;
				z /= mo;
				rotateEarth(x,y,z);

				break;
			default:
				assert(false);
			}

			theta_sphere = acos(z);
			phi_sphere = cvFastArctan(y,x)*PI/180;

			p_pol = f*theta_sphere;
			theta_pol = phi_sphere;

			x_cart = p_pol*cos(theta_pol);
			y_cart = p_pol*sin(theta_pol);

			u_src = x_cart + center.x;
			v_src = -y_cart + center.y;

			if (u_src < 0 || u_src >= srcImage.rows || v_src < 0 || v_src >= srcImage.cols)
					continue;

			dstImage.at<Vec3b>(j,i) = srcImage.at<Vec3b>(v_src,u_src);
			pixelReMapping.set(std::make_pair(v_src,u_src), std::make_pair(j,i));
		}
}

void CorrectingUtil::LLMCLMUFCorrecting(Mat &src, Mat &dst, Point2i center, int radius, CorrectingType ctype,  Point2d w) {
	double w_lon = w.x, w_lat = w.y;
	double dx = camFieldAngle / src.cols; 
	double dy = dx;
	double f = radius/(camFieldAngle/2);

	double lat, lon;
	double p_pol, theta_pol;
	double theta_sphere, phi_sphere;

	double x,y,z,r;

	double x_cart, y_cart;
	double u_src,v_src,u_dst,v_dst;
	double lon_offset = (PI - camFieldAngle) / 2, lat_offset = (PI - camFieldAngle) / 2;
	double lat_max = 2*getLFromPhi_ufixed(0,w_lat), lon_max = 2*getLFromPhi_ufixed(0,w_lon);

	int left, top;

	switch (ctype) {
	case LONG_LAT_MAPPING_CAM_LENS_MOD_UNFIXED_REVERSED:
		for (int j=0; j<dst.rows; ++j)
			for (int i=0; i<dst.cols; ++i) {
				lat = getPhiFromV_ufixed(j*lat_max / dst.rows, w_lat);
				//lon = getPhiFromV_ufixed(i*lon_max / dst.cols, w_lon);
				//lat = lat_offset+j*dy;
				lon = lon_offset+i*dx;

				x = -sin(lat)*cos(lon);
				y = cos(lat);
				z = sin(lat)*sin(lon);

				theta_sphere = acos(z);
				phi_sphere = cvFastArctan(y,x)*PI/180;

				p_pol = f*theta_sphere;
				theta_pol = phi_sphere;

				x_cart = p_pol*cos(theta_pol);
				y_cart = p_pol*sin(theta_pol);

				u_src = x_cart + center.x;
				v_src = -y_cart + center.y;

				if (u_src < 0 || u_src >= src.rows || v_src < 0 || v_src >= src.cols)
						continue;
				dst.at<Vec3b>(j,i) = src.at<Vec3b>(v_src,u_src);
				pixelReMapping.set(std::make_pair(v_src,u_src), std::make_pair(j,i));
			}
		break;
	case LONG_LAT_MAPPING_CAM_LENS_MOD_UNFIXED_FORWARD:
		left = center.x - radius;
		top = center.y - radius;

		for (int j = top; j < top+2*radius; ++j)
			for (int i = left; i < left+2*radius; ++i) {
				if (square(i-center.x) + square(j-center.y) > square(radius))
					continue;
				u_src = i, v_src = j;
				x_cart = (u_src-center.x);
				y_cart = -(v_src-center.y);

				theta_pol = cvFastArctan(y_cart, x_cart)*PI/180;
				p_pol = sqrt(square(x_cart) + square(y_cart));

				theta_sphere = p_pol*(camFieldAngle/2)/radius;
				phi_sphere = theta_pol;

				x = sin(theta_sphere)*cos(phi_sphere);
				y = sin(theta_sphere)*sin(phi_sphere);
				z = cos(theta_sphere);

				lat = acos(y);
				lon = cvFastArctan(z,-x)*PI/180;

				v_dst = src.rows*(lat_max/2-getLFromPhi_ufixed(lat, w_lat)) / lat_max;
				//u_dst = src.cols*(lon_max/2-getLFromPhi_ufixed(lon, w_lon)) / lon_max;
				u_dst = (lon-lon_offset)/dx;
				//v_dst = (lat-lat_offset)/dy;

				if (u_dst < 0 || u_dst >= dst.rows || v_dst < 0 || v_dst >= dst.cols)
						continue;
				dst.at<Vec3b>(v_dst,u_dst) = src.at<Vec3b>(j,i);
				pixelReMapping.set(std::make_pair(j,i), std::make_pair(v_dst,u_dst));
			}
		break;
	default:
		assert(false);
	}
}

double CorrectingUtil::getPhiFromV_ufixed(double v, double w) const {
	static const int maxIter = 200;
	int cnt = 0;
	static double L_0 = getLFromPhi_ufixed(0,w);
	double left, right, mid, tmp;
	if (v >=0 && v < L_0) {
		left = 0, right = PI/2.0, mid = left;
	} else {
		left = PI/2.0, right = PI, mid = right;
	}
	tmp = _equation_ufixed(v, mid, w);
	while (abs(tmp) > ERR && cnt++ < maxIter) {
		mid = (left + right) / 2;
		tmp = _equation_ufixed(v, mid, w);
		if (tmp > 0) left = mid;
		else right = mid;
	}
	return mid;
}

double CorrectingUtil::getLFromPhi_ufixed(double phi, double w) const {
	double l = sin(w)*sqrt(square(cos(phi)) + square(1-sin(phi))) / sin(PI - w-atan((1-sin(phi)) / abs(cos(phi))));
	return (phi > PI/2.0) ? -l:l;
}

double CorrectingUtil::_equation_ufixed(double l, double phi, double w) const {
	static double L_0 = getLFromPhi_ufixed(0,w);
	return getLFromPhi_ufixed(phi,w)-L_0+l;
}

