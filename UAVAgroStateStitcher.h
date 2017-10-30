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
		int maxMatch = 40;
		bool bound = false;
		double yMin=0;
		double yMax=0;
		double xMin=0;
		double xMax=0;
		
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

			if(obj.channels() == 4){
				//en el caso de que haya transparencia, se hace un pegado especial
				Mat objAux(scene.size(), scene.type());
				objWarped.copyTo(objAux, imgMaskWarped);
				scene = copyToTransparent(objAux, scene);
			}else{
				objWarped.copyTo(scene, imgMaskWarped);
			}

			return{ scene, imgMaskWarped };
		}

		Mat copyToTransparent(Mat obj, Mat scene){
			Mat rgbaObj[4];
			split(obj,rgbaObj);
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(rgbaObj[3].at<uchar>(i,j) == 255){
						scene.at<Vec4b>(i,j) = obj.at<Vec4b>(i,j);
					}
				}
			}
			return scene;
			
		}

		vector<Mat> stitchWarpTransp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));

			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			// Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			// erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			objWarped  = CommonFunctions::makeBackGroundTransparent(objWarped);


			objWarped.copyTo(scene, imgMaskWarped);

			return{ scene, imgMaskWarped };
		}

		void saveDetectAndCompute(Mat img, Mat mask, Mat &descriptors, vector<KeyPoint> &keypoints){
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
			///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
			//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
			Ptr<cv::BRISK> orb = cv::BRISK::create(this->kPoints);
			vecDesc = vector<Mat>(imgs.size());
			vecKp = vector< vector<KeyPoint> >(imgs.size());
			cout << vecKp.size() <<endl;
			parallel_for_(Range(0, imgs.size()), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					std::vector<KeyPoint> keypoints;
					Mat descriptors;
					orb->detectAndCompute(imgs[i] , Mat() , keypoints , descriptors);
					vecDesc[i]=descriptors;
					vecKp[i]=keypoints;
					cout << "\n KeyPoint imagen" + to_string(i) + ": " << keypoints.size()<< endl;
				}
			});
		}
		Mat readDetectAndComputeDesc(int i){
			struct timeval begin;
			gettimeofday(&begin, NULL);
			FileStorage fs("Data/DetectCompute/descriptores.yml", FileStorage::READ);
			Mat descriptor;
			fs["descriptor"+ to_string(i)] >> descriptor;
			begin = CommonFunctions::tiempo(begin, "Tiempo para cargar desc: ");

			return descriptor;
		}
		vector<Point2f> readDetectAndComputeKp(int i){
			struct timeval begin;
			gettimeofday(&begin, NULL);
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
			return matches;
		}

		vector< DMatch > compareMatch(){

		}


		vector< DMatch > match(vector<Mat> descriptors,vector< vector<KeyPoint> > keypoints){
			vector< vector< DMatch > > matches;
			for(int i = 1;i < descriptors.size() ; i++){
				matches.push_back(this->matchKp(descriptors[0],descriptors[i]));
			}
						
			vector< DMatch > matchAux;
			int maxMatch = this->maxMatch;
			for(int i = 1;i < matches.size() ; i++){
				for(int j = 0; j < matches[0].size();j++){
					for(int k = 0;k < matches[i].size();k++){
						if(matches[0][j].queryIdx ==  matches[i][k].queryIdx){
							bool agregar = true;
							for( int l = 0;l < matchAux.size(); l++){
								if(matchAux[l].queryIdx == matches[0][j].queryIdx){
									agregar = false;
									break;
								}
							}
							if(agregar){
								matchAux.push_back(matches[0][j]);
							}
						}
					}
				}
			}
			sort(matchAux.begin(), matchAux.end(), sortByDist);
			sort(matches[0].begin(),matches[0].end(),sortByDist);
			if(matchAux.size() < maxMatch) maxMatch = matchAux.size();
			vector< DMatch > good_matches(matchAux.begin(),matchAux.begin() + maxMatch);
			for(int i = 0; i < this->maxMatch + 10; i++){
				bool agregar = true;
				for(int j = 0 ; j < good_matches.size(); j++){
					if(good_matches[j].queryIdx == matches[0][i].queryIdx){
						agregar = false;
					}
				}
				if(agregar){
					good_matches.push_back(matches[0][i]);
				}
			}
			int min = 100;
			int max = 200;
			int valx = 75; //75 para las altas
			vector< DMatch > gm;
			if(this->bound){
				min+=abs(this->yMin);
				max+=abs(this->yMax);
			}
			// if(!this->bound){
				while(gm.size() < 4){
					gm= vector< DMatch >();
					for(int i = 0; i < good_matches.size(); i++){
						Point2f pt0 = keypoints[0][good_matches[i].queryIdx].pt;
						Point2f pt1 = keypoints[1][good_matches[i].trainIdx].pt;
						
						if( (pt1.y-pt0.y) > min && (pt1.y-pt0.y) < max 
							&& abs(pt1.x-pt0.x) < valx
							){
							gm.push_back(good_matches[i]);
						}
					}
					min+=100;
					max+=100;
					if(max>5000){
						break;
					}
				}
				if(gm.size() < 4){
					gm = good_matches;
				}

			return gm;
		}

		vector<Mat> matchAndTransform(vector<Mat> descriptors,vector< vector<KeyPoint> > keypoints)
		{
			Mat H;
			vector< DMatch > good_matches = match(descriptors,keypoints);
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
			return{ H };
		}

		vector<Mat> matchAndTransform(Mat img1, Mat img2, int i, vector<Mat> descriptors,vector< vector<KeyPoint> > keypoints)
		{
			Mat H;
			vector< DMatch > good_matches = match(descriptors,keypoints);
			Mat aux;
			drawMatches(img1, keypoints[0],img2,keypoints[1],good_matches,aux,
				Scalar::all(-1),Scalar::all(-1),
				std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(i)+".png",aux);
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
			return{ H };
		}

		Mat stitchImgs(vector<string> strImgs){
			string normal = "\033[0m";
			string process = "\033[1;32m";
			vector<Mat> H(strImgs.size());
			vector<Mat> homoNoMultiplicated(strImgs.size());
			struct timeval begin;
			gettimeofday(&begin, NULL);
			//AGREGO INDENTACION

			//CALCULO HOMOGRAFIAS PARA CADA IMAGEN
			H[0] = (Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));
			//obtengo kp
			vector<Mat> imgs = CommonFunctions::cargarImagenes(strImgs , this->tamano);
			begin = CommonFunctions::tiempo(begin, "cargar las imagenes:");
			vector<Mat> vecDesc;
			vector< vector<KeyPoint> > vecKp;
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Obteniendo keypoints y descriptores: " + normal<< endl;
			saveDetectAndCompute(imgs, vecDesc, vecKp);
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Matcheando y obteniendo Homografias: " + normal << endl;
			//usado para documentar las homografias
			parallel_for_(Range(0, strImgs.size()-1), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					cout << "Empezo Match y Homografia "+to_string(i)+ ": \n";
					vector<Mat> aux;
					int nMin;
					(i<3)?nMin=i:nMin=3;
					int nMax;
					int nMaxVal=3;
					((vecDesc.size()-i-1) < nMaxVal )? nMax=(vecDesc.size()-i-1) : nMax=nMaxVal;

					vector<Mat> newVecDesc;
					vector< vector<KeyPoint> > newVecKp;
					newVecDesc.push_back(vecDesc[i]);
					newVecKp.push_back(vecKp[i]);
					// for(int j=1;j <= nMin;j++){
					// 	newVecDesc.push_back(vecDesc[i-j]);
					// 	newVecKp.push_back(vecKp[i-j]);
					// }
					for(int j=1;j <= nMax;j++){
						newVecDesc.push_back(vecDesc[i+j]);
						newVecKp.push_back(vecKp[i+j]);
					}
					aux = matchAndTransform(imgs[i],imgs[i+1],i+1,newVecDesc,newVecKp);
					
					//Una homografia es para calcular el boundbox (H) y la otra es para
					//con ese boundbox calcular las otras homografias, multiplicandolas 
					//esta es homonomultpilicates
					homoNoMultiplicated[i+1] = (aux[0]);
					//Encuentro el maximo y minimo tanto en x como en y de todas las 
					//homografias para despues poder hacer el bounding box;
					cout << "Termino Match y Homografia "+to_string(i)+ ": \n";
					
				}
			});
			FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			for(int i = 0; i < strImgs.size()-1;i++){
				H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				if(H[i+1].at<double>(0,2) < this->xMin){
					this->xMin = H[i+1].at<double>(0,2);
				}
				if(H[i+1].at<double>(0,2) > this->xMax){
					this->xMax = H[i+1].at<double>(0,2);
				}
				if(H[i+1].at<double>(1,2) < this->yMin){
					this->yMin = H[i+1].at<double>(1,2);
				}
				if(H[i+1].at<double>(1,2) > this->yMax){
					this->yMax = H[i+1].at<double>(1,2);
				}
				fsHomo << "homografia"+to_string(i+1) << H[i+1];
			}
			fsHomo.release();

			Mat boundBox = imgs[0];
			this->yMin-=1000;
			boundBox = CommonFunctions::boundingBox(boundBox, abs(this->yMin) , this->yMax , abs(this->xMin),this->xMax);
			
			cout<< "ymin: "<< yMin << " ymax: "<< this->yMax<< "xmin: "<< this->xMin << " xmax: "<< this->xMax << endl;
			Mat boundBoxMask;
			threshold( boundBox, boundBoxMask, 1, 255,THRESH_BINARY );
			cvtColor(boundBoxMask, boundBoxMask, cv::COLOR_RGB2GRAY);

			Mat descriptors;
			vector<KeyPoint> keypoints;
			saveDetectAndCompute(boundBox , boundBoxMask, descriptors, keypoints);
			this->bound=true;
			vector<Mat> aux = matchAndTransform(boundBox,imgs[1],40,{descriptors,vecDesc[1]},{keypoints,vecKp[1]});
			// matchAndTransform(imgs[i],imgs[i+1],i,newVecDesc,newVecKp);
			
			H[1] = aux[0];
			for(int i = 2 ; i < H.size(); i++){
				H[i]=H[i-1] * homoNoMultiplicated[i];
				H[i] = H[i] / H[i].at<double>(2,2);
			}
			
			begin = CommonFunctions::tiempo(begin, " obtener las homografias: ");
			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Generando orthomosaico: ... ("<< strImgs.size()-1<< ")" + normal<< endl;
			for (int i = 1; i < strImgs.size(); i++){
				cout << "-" << (i+1) * 100 / strImgs.size() << "%" << endl;
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				imwrite(res, boundBox);
			}

			begin = CommonFunctions::tiempo(begin, " pegar imagenes: ");
			if(boundBox.channels() < 4){
				boundBox = CommonFunctions::addTransparence(boundBox);
			}

			return boundBox;
		}
		
};

#endif
