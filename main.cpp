#include "UAVAgroStateStitcher.h"
#include "funcionesutiles.h"
#include "opencv2/core/core.hpp"
#include "CommonFunctions.h"
#include "UAVAgroStateCalibration.h"

using namespace cv;

void readme();

int main(int argc, char** argv)
{
	int option = 0;
	if( argc > 1){
		option = atoi(argv[1]);
	}else{
		cout<<"Que deseas hacer? \n 1: Calibrar camara \n 2: Calcular orthomosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
		cin>>option;
	}

	switch(option)
	{
		case 1:{
			int optionCalibration=0;
			if(argc > 2){
				optionCalibration = atoi(argv[2]);
			}else{
			cout << "Desea obtener la matriz para calibrar (1), o realizar la undistorcion (2)?"<<endl;
			cin >> optionCalibration;
			}
			switch(optionCalibration){
				case 1:{
					cout << "Obteneniendp la matriz para calibrar"<<endl;
					// int numBoards = 0;
					int numCornersHor;
					int numCornersVer;

					printf("Enter number of corners along width: ");
					scanf("%d", &numCornersHor);
					
					printf("Enter number of corners along height: ");
					scanf("%d", &numCornersVer);
				
					cout<<"name of the camera: "<<endl;
					string cameraName;
					cin>>cameraName;

					UAVAgroStateCalibration::calibrateImg(numCornersHor,numCornersVer,cameraName);
				}break;
				case 2:{
					cout << "Realizando la undistorcion"<<endl;
					cout<<"name of the camera: "<<endl;
					string cameraName;
					cin>>cameraName;
					UAVAgroStateCalibration::undistortImgs(cameraName);
				}break;
				}	
		}
		break;
		case 2:{
			//clock_t begin = clock();
			////Cargo los string de imagenes
			int tamano = 4;
			bool undistort = false;
			float kPoints = 3	;
			bool originalSize=false;

			struct timeval begin;
			gettimeofday(&begin, NULL);
			
			if(argc > 2){
				if( atoi(argv[2]) == 0 ){
					cout<<"The order of the params is:	\n   tamano \n   undistorted \n";
					cout<<"   position (abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3) \n   kPoints \n   rowGrid \n   colGrid \n";
					return 0;
				}
				tamano = stoi(argv[2]);
			}else{
				cout<< "Resize de las imagenes 1/n, ingrese n: ";
				cin>>tamano;
			}
			if(argc > 3){
				undistort = stoi(argv[3]);
			}else{
				cout<< "Quiere cargar las imagenes desde undistort? (1): ";
				cin>>undistort;
			}
			if(argc > 4){
				kPoints = stof(argv[4]);
			}else{
				cout<< "Que threshold ingresara? (1): ";
				cin>>kPoints;
			}
			int usarHomografia = 0;
			if(argc > 5){
				usarHomografia = stoi(argv[5]);
			}
			// else{
			// 	cout<< "Desea que la imagen de salida tenga su tamaño original? (1): ";
			// 	cin>>originalSize;
			// }

			int minKeypoints = 23000;
			vector<string> strBorders = CommonFunctions::obtenerImagenes("Imagenes/Pegado/bordes/");
			vector<int> minMax(2);
			// // minMax[0] = 10;
			// // minMax[1] = 28;
			// minMax = CommonFunctions::setBorder(CommonFunctions::cargarImagen(strBorders[0], tamano,IMREAD_UNCHANGED));
			vector<string> strFolders;
			strFolders = CommonFunctions::obtenerImagenes("Imagenes/Pegado/input/");
			for(int i = 0; i < strFolders.size();i++){
				strFolders[i] = (strFolders[i]+'/');
				const char *chr = strFolders[i].c_str();
				vector<string> strImgs = CommonFunctions::obtenerImagenes(chr);
				UAVAgroStateStitcher *uav = new UAVAgroStateStitcher(
					strImgs,minMax,tamano,minKeypoints,kPoints,originalSize,usarHomografia);
				Mat img = uav->runAll();		
				imwrite("Imagenes/Pegado/output/ortomosaico/resultado"+to_string(i)+".png",img);
			}

			minKeypoints = 30000;
			vector<string> strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/output/ortomosaico/");
			// minMax = CommonFunctions::setBorder(imread(strImgs[0]));
			UAVAgroStateStitcher *uav = new UAVAgroStateStitcher(
				strImgs,minMax,1,minKeypoints,kPoints,originalSize,usarHomografia);
			Mat img = uav->runAll();
			imwrite("Imagenes/Pegado/output/resultadofinal.png",img);

			CommonFunctions::tiempo(begin, "realizar todo: ");
			
			// Mat I = imread("Imagenes/Pegado/output/resultadofinal3.png");
    		// CommonFunctions::Aindane(I, 20);
    		// imshowpair(I, outputImage, 'montage');

		}
		break;
		case 3:{
			vector<string> strNDVI = CommonFunctions::obtenerImagenes("Imagenes/NDVI/input/");
			for(int i = 0 ; i<strNDVI.size();i++){
				UAVAgroStateIndexCalcs::ndviCalcu(strNDVI[i]);
			}
		}
		break;
	}
	// cout<<"Proceso finalizado, ahora que deseas hacer? \n 1: Calibrar camara \n 2: Calcular orthomosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
}