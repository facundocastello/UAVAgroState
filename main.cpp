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
		cout<<"Que deseas hacer? \n 0: Calibrar camara \n 1: Remover distorsión  \n 2: Calcular orto-mosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
		cin>>option;
	}

	switch(option)
	{
		//calibrar o undistort
		case 0:{
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
		}
		break;
		case 1:{
			cout << "Realizando la undistorcion"<<endl;
			cout<<"name of the camera: "<<endl;
			string cameraName;
			cin>>cameraName;
			UAVAgroStateCalibration::undistortImgs(cameraName);	
		}
		break;
		//pegar
		case 2:{
			//clock_t begin = clock();
			////Cargo los string de imagenes
			int tamano = 4;
			int minKeypoints = 1000;
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
				originalSize = stoi(argv[3]);
			}else{
				cout<< "Quiere recuperar el tamaño original de las imágenes? (1): ";
				cin>>originalSize;
			}
			if(argc > 4){
				minKeypoints = stof(argv[4]);
			}else{
				cout<< "Cantidad minima de kp? (1): ";
				cin>>minKeypoints;
			}

			UAVAgroStateStitcher *uav;
			vector<string> strImgs;
			Mat img;
			while(strImgs.size() == 0){
				strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/input/");
				if(strImgs.size() == 0){
					cout << "Para comenzar ingrese las imagenes dentro de Imagenes/Pegado/input/ y presione entrer" << endl;
					getchar();
				}
			}
			uav = new UAVAgroStateStitcher(
				strImgs,tamano,minKeypoints,originalSize,false,false);
			img = uav->runAll();

			minKeypoints = 10000;
			strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/output/ortomosaico/");
			uav = new UAVAgroStateStitcher(
				strImgs,1,minKeypoints,originalSize,false,true);
			img = uav->runAll();
			imwrite("Imagenes/Pegado/output/resultadofinal.png",img);

			CommonFunctions::tiempo(begin, "realizar todo: ");
		}
		break;
		//ndvi
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