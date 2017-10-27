#ifndef UAVAGROSTATESTICHER_H
#define UAVAGROSTATESTICHER_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "funcionesutiles.h"
#include "CommonFunctions.h"

using namespace cv;
using namespace std;

bool sortByDist(DMatch a, DMatch b){
	return a.distance < b.distance;
}



class UAVAgroStateStitcher{
	public:
		int tamano;
		float kPoints;

		UAVAgroStateStitcher(int tamano = 4,
					float kPoints = 3
					)
		{
			this->tamano = tamano;
			this->kPoints = kPoints;
			//position: -abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3
		}

		vector<Mat> stitchWarp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));

			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			objWarped.copyTo(scene, imgMaskWarped);

			return{ scene, imgMaskWarped };
		}

		vector<Mat> stitchWarpTransp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));

			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			objWarped  = CommonFunctions::makeBackGroundTransparent(objWarped);


			objWarped.copyTo(scene, imgMaskWarped);

			return{ scene, imgMaskWarped };
		}

		void saveDetectAndCompute(Mat img, Mat mask, Mat &descriptors, vector<KeyPoint> &keypoints){
		   	// clock_t begin = clock();
		   	///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
		  	//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
			Ptr<cv::BRISK> orb = cv::BRISK::create(this->kPoints);
			cout << "--------------------------------------------------------" << endl;
			cout << "Calculando KeyPoints y descriptores de la imagen sola: ... ";

			orb->detectAndCompute(img , mask , keypoints , descriptors);
			cout << "\n KeyPoint imagen: " << keypoints.size()<< endl;
			// begin = CommonFunctions::tiempo(begin, "Tiempo para kp y descriptores: ");
		}

		void saveDetectAndCompute(vector<Mat> imgs,
			 vector<Mat> &vecDesc,
			 vector< vector<KeyPoint> > &vecKp){
			// clock_t begin = clock();
			///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
			//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
			Ptr<cv::BRISK> orb = cv::BRISK::create(this->kPoints);
			FileStorage fsDesc("Data/DetectCompute/descriptores.yml", FileStorage::WRITE);
			for(int i = 0;i < imgs.size() ; i++){
				cout << "--------------------------------------------------------" << endl;
				cout << "Calculando KeyPoints y descriptores de la imagen" + to_string(i) +": ... ";
				std::vector<KeyPoint> keypoints;
				Mat descriptors;
				orb->detectAndCompute(imgs[i] , Mat() , keypoints , descriptors);
				fsDesc << "descriptor"+ to_string(i) << descriptors;
				fsDesc << "features" + to_string(i)  << "[";
				for( int i = 0; i < keypoints.size(); i++ )
				{
					fsDesc << "{:" << "x" << keypoints[i].pt.x << "y" << keypoints[i].pt.y << "}";
				}
				fsDesc << "]";
				vecDesc.push_back(descriptors);
				vecKp.push_back(keypoints);
				cout << "\n KeyPoint imagen" + to_string(i) + ": " << keypoints.size()<< endl;
				// begin = CommonFunctions::tiempo(begin, "Tiempo para kp y descriptores" + to_string(i) + ": ");
			}
			fsDesc.release();
		}
		Mat readDetectAndComputeDesc(int i){
			clock_t begin = clock();
			FileStorage fs("Data/DetectCompute/descriptores.yml", FileStorage::READ);
			Mat descriptor;
			fs["descriptor"+ to_string(i)] >> descriptor;
			begin = CommonFunctions::tiempo(begin, "Tiempo para cargar desc: ");

			return descriptor;
		}
		vector<Point2f> readDetectAndComputeKp(int i){
			clock_t begin = clock();
			FileStorage fs("Data/DetectCompute/descriptores.yml", FileStorage::READ);
			FileNode features = fs["features"+ to_string(i)];
			// iterate through a sequence using FileNodeIterator
			vector<Point2f> Kp;
			for(FileNodeIterator it = features.begin() ; it != features.end(); ++it )
			{
				Point2f pt((float)(*it)["x"], (float)(*it)["y"]);
				Kp.push_back(pt);
			}
			begin = CommonFunctions::tiempo(begin, "Tiempo para cargar kp: ");
			return Kp;
		}

		vector< DMatch > matchKp(Mat descriptors_1, Mat descriptors_2){
			BFMatcher matcher(NORM_HAMMING,true);
			vector< DMatch > matches;
			matcher.match(descriptors_1, descriptors_2, matches);
			cout << "Cantidad de matches: "<<matches.size()<<" \n";
			return matches;
		}

		vector< DMatch > compareMatch(){

		}


		vector<Mat> matchAndTransform(vector<Mat> descriptors,vector< vector<KeyPoint> > keypoints)
		{
			Mat H;
			vector< vector< DMatch > > matches;
			for(int i = 1;i < descriptors.size() ; i++){
				matches.push_back(this->matchKp(descriptors[0],descriptors[i]));
			}
			vector< DMatch > good_matches;
			int maxMatch = 60;
			int matchSize = (matches[0].size() >=(maxMatch/matches.size()) )?(maxMatch/matches.size()) : matches[0].size();
			for(int i = 1;i < matches.size() ; i++){
				vector< DMatch > matchAux;
				for(int j = 1; j < matches[0].size();j++){
					for(int k = 1;k < matches[i].size();k++){
						if(matches[0][j].queryIdx ==  matches[i][k].queryIdx){
							matchAux.push_back(matches[0][j]);
						}
					}
				}
				sort(matchAux.begin(), matchAux.end(), sortByDist);
				good_matches.insert(good_matches.end(),matchAux.begin(),matchAux.begin() + matchSize);
				cout<< "matchaux size: " << matchAux.size() << "goodmatches" << good_matches.size() << endl;
			}
			
			sort(matches[0].begin(), matches[0].end(), sortByDist);
			good_matches.insert(good_matches.end(),matches[0].begin(),matches[0].begin() + matchSize);
			///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int i = 0; i < good_matches.size(); i++) {
				obj.push_back(keypoints[0][good_matches[i].queryIdx].pt);
				scene.push_back(keypoints[1][good_matches[i].trainIdx].pt);
			}
			// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
			Mat maskH;
			H = findHomography(scene, obj, CV_RANSAC);
			/// DEVUELVO H
			cout << "--------------------------------------------------------" << endl;
			return{ H };
		}

		Mat stitchImgs(vector<string> strImgs){
			string normal = "\033[0m";
			string process = "\033[1;32m";
			vector<Mat> H;
			vector<Mat> homoNoMultiplicated;
			clock_t begin = clock();
			//AGREGO INDENTACION

			//CALCULO HOMOGRAFIAS PARA CADA IMAGEN
			H.push_back(Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated.push_back(Mat::eye(3, 3, CV_64F));
			double xMin=0;double xMax=0;double yMin=0;double yMax=0;
			//obtengo kp
			vector<Mat> imgs = CommonFunctions::cargarImagenes(strImgs , this->tamano);
			begin = CommonFunctions::tiempo(begin, "********************Tiempo en cargar las imagenes:");
			vector<Mat> vecDesc;
			vector< vector<KeyPoint> > vecKp;
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Obteniendo keypoints y descriptores: " + normal<< endl;
			saveDetectAndCompute(imgs, vecDesc, vecKp);
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Matcheando y obteniendo Homografias: " + normal << endl;
			//usado para documentar las homografias
			FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			for (int i = 0; i < strImgs.size()-1; i++){
				cout << "Match y Homografia de "+strImgs[i]+" y " + strImgs[i+1] + " :" <<endl;
				vector<Mat> aux;
				if((vecDesc.size()-i-2) > 0 ) {
					if((vecDesc.size()-i-3) > 0 ) {
						aux = matchAndTransform(
							{vecDesc[i], vecDesc[i+1],vecDesc[i+2],vecDesc[i+3]},
							{vecKp[i], vecKp[i+1],vecKp[i+2],vecKp[i+3]});	
					}else{
						aux = matchAndTransform(
							{vecDesc[i], vecDesc[i+1],vecDesc[i+2]},
							{vecKp[i], vecKp[i+1],vecKp[i+2]});	
					}
				}else{
					aux = matchAndTransform(
						{vecDesc[i], vecDesc[i+1]},
						{vecKp[i], vecKp[i+1]});
				}
				//Una homografia es para calcular el boundbox (H) y la otra es para
				//con ese boundbox calcular las otras homografias, multiplicandolas 
				//esta es homonomultpilicates
				H.push_back(H[i] * aux[0]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				homoNoMultiplicated.push_back(aux[0]);
				//Encuentro el maximo y minimo tanto en x como en y de todas las 
				//homografias para despues poder hacer el bounding box;
				if(H[i+1].at<double>(0,2) < xMin){
					xMin = H[i+1].at<double>(0,2);
					cout << "xmin " << xMin << endl;
				}
				if(H[i+1].at<double>(0,2) > xMax){
					xMax = H[i+1].at<double>(0,2);
					cout << "xmax " << xMax << endl;
				}
				if(H[i+1].at<double>(1,2) < yMin){
					yMin = H[i+1].at<double>(1,2);
					cout << "ymin " << yMin << endl;
				}
				if(H[i+1].at<double>(1,2) > yMax){
					yMax = H[i+1].at<double>(1,2);
					cout << "ymax " << yMax << endl;
				}
				fsHomo << "homografia"+to_string(i+1) << H[i+1];
			}
			fsHomo.release();

			Mat boundBox = CommonFunctions::cargarImagen(strImgs[0], this->tamano);;
			boundBox = CommonFunctions::boundingBox(boundBox, (yMin * -1), yMax , (xMin * -1),xMax);
			Mat boundBoxMask;
			threshold( boundBox, boundBoxMask, 1, 255,THRESH_BINARY );
			cvtColor(boundBoxMask, boundBoxMask, cv::COLOR_RGB2GRAY);

			Mat descriptors;
			vector<KeyPoint> keypoints;
			saveDetectAndCompute(boundBox , boundBoxMask, descriptors, keypoints);

			Mat img1 = CommonFunctions::cargarImagen(strImgs[1] , this->tamano);

			vector<Mat> aux = matchAndTransform({descriptors,vecDesc[1]},{keypoints,vecKp[1]});
			H[1] = aux[0];
			for(int i = 2 ; i < H.size(); i++){
				H[i]=H[i-1] * homoNoMultiplicated[i];
				// H[i] = H[i] / H[i].at<double>(2,2);
			}
			
			begin = CommonFunctions::tiempo(begin, "********************Tiempo en obtener las homografias: ");
			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Generando orthomosaico: ... ("<< strImgs.size()-1<< ")" + normal<< endl;
			for (int i = 1; i < strImgs.size(); i++){
				cout << "-" << (i+1) * 100 / strImgs.size() << "%" << endl;
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				imwrite(res, boundBox);
			}

			begin = CommonFunctions::tiempo(begin, "********************Tiempo en pegar imagenes: ");

			Mat tmp,alpha;
			
			cvtColor(boundBox,tmp,CV_BGR2GRAY);
			threshold(tmp,alpha,1,255,THRESH_BINARY);
			
			Mat rgb[3];
			split(boundBox,rgb);
			
			Mat rgba[4]={rgb[0],rgb[1],rgb[2],alpha};
			merge(rgba,4,boundBox);

			return boundBox;
		}
		
};

#endif
