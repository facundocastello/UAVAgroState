#ifndef UAVAGROSTATEINDEXCALCS_H
#define UAVAGROSTATEINDEXCALCS_h
#include "CommonFunctions.h"

class UAVAgroStateIndexCalcs
{
public:
	void static info(const cv::Mat &image, std::ostream &out = std::cout) {
		out << "Characteristics\n";
		out << "\tSize " << image.rows << 'x' << image.cols << '\n';
		out << "\tChannels " << image.channels() << '\n';
		out << "\tDepth ";
		out << '\t';
		switch (image.depth()) {
		case CV_8U: out << "8-bit unsigned integers ( 0..255 )\n"; break;
		case CV_8S: out << "8-bit signed integers ( -128..127 )\n"; break;
		case CV_16U: out << "16-bit unsigned integers ( 0..65535 )\n"; break;
		case CV_16S: out << "16-bit signed integers ( -32768..32767 )\n"; break;
		case CV_32S: out << "32-bit signed integers ( -2147483648..2147483647 )\n"; break;
		case CV_32F: out << "32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )\n"; break;
		case CV_64F: out << "64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )\n"; break;
		}
	}

	Mat static ndviCalcu(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position);

		imwrite("Imagenes/resultados/NDVI/"+ strImg +"original.png", imgaux);
		
		vector<Mat> BGRA;
		split(imgaux, BGRA);	
		Mat numerador,denominador,division;
		
		subtract(BGRA[0],BGRA[2],numerador,cv::noArray(),CV_8S);
		add(BGRA[0],BGRA[2],denominador,cv::noArray(),CV_8S);
		divide(numerador,denominador,division,1.,CV_32F);
		
		division = (division * 127) + 127;
		division.convertTo(division,CV_8U);
		Mat resultadogris = division;
		// cout<<BGRA[3].empty()<<endl;
		if(!BGRA[3].empty()){
			Mat auxAlpha[4]={division,division,division,BGRA[3]};
			merge(auxAlpha,4,resultadogris);
		};
		imwrite("Imagenes/resultados/NDVI/"+ strImg +"resultadogris.png", resultadogris);

		applyColorMap(division, division, COLORMAP_JET);
		vector<Mat> BGR;
		split(division,BGR);
		Mat resultadocolor = division;
		if(!BGRA[3].empty()){
			Mat auxAlpha2[4]={BGR[0],BGR[1],BGR[2],BGRA[3]};
			merge(auxAlpha2,4,resultadocolor);
		}
		imwrite("Imagenes/resultados/NDVI/"+ strImg +"resultadocolor.png", resultadocolor);
	
		return division;
	}

private:
    
};


#endif