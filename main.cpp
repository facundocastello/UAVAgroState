#include "Stitcher.h"
#include "Calibration.h"
#include "Undistort.h"
#include "IndexCalculation.h"
#include "UtilInformation.h"
#include "Redaction.h"


int main(int argc, char** argv)
{
	int option = 10;
	if( argc > 1){
		option = atoi(argv[1]);
	}else{
		cout<<"Que deseas hacer? \n 0: Calibrar camara \n 1: Remover distorsión  \n 2: Calcular orto-mosaico \n 3: Calcular indices de vegetación \n 4: Calcular información útil \n 5: Redactar informe" << endl;
		cin>>option;
	}
	while(option!= 6){

		switch(option)
		{
			//calibrar o undistort
			case 0:{
				// int numBoards = 0;
				int numCornersHor;
				int numCornersVer;

				printf("Ingrese el numero de esquinas a lo largo del ancho ");
				scanf("%d", &numCornersHor);
				
				printf("Ingrese el numero de esquinas a lo largo de lo alto: ");
				scanf("%d", &numCornersVer);
			
				cout<<"Nombre de la camara: "<<endl;
				string cameraName;
				cin>>cameraName;

				cout << CommonFunctions::stringAzul("Obteneniendo la matriz para calibrar")<<endl;
				Calibration::calibrateImg(numCornersHor,numCornersVer,cameraName);
			}
			break;
			case 1:{
				cout << CommonFunctions::stringAzul("Realizando la undistorcion")<<endl;
				cout<<"Nombre de la camara: "<<endl;
				string cameraName;
				cin>>cameraName;
				Undistort::undistortImgs(cameraName);	
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
					tamano = stoi(argv[2]);
				}else{
					cout<< "Resize de las imagenes 1/n, ingrese n: ";
					cin>>tamano;
				}
				if(argc > 3){
					originalSize = stoi(argv[3]);
				}else{
					cout<< "Quiere recuperar el tamaño original de las imágenes? (0 o 1): ";
					cin>>originalSize;
				}
				if(argc > 4){
					minKeypoints = stof(argv[4]);
				}else{
					cout<< "Cantidad minima de kp?: ";
					cin>>minKeypoints;
				}

				uav::Stitcher *uav;
				uav = new uav::Stitcher(tamano,minKeypoints,originalSize);
				uav->processManager();

				CommonFunctions::tiempo(begin, "realizar todo: ");
			}
			break;
			//ndvi
			case 3:{
				bool otputSticthing;
				bool multispectral;
				bool paralell;
				bool sobreescribir;
				if(argc > 2){
					otputSticthing = stoi(argv[2]);
				}else{
					cout<< "Desea calcular los indices del resultado obtenido en el pegado? (0 o 1)";
					cin>>otputSticthing;
				}
				if(argc > 3){
					multispectral = stoi(argv[3]);
				}else{
					cout<< "Los indices a obtener son para imágenes multi-espectrales(1) o RGB(0)?";
					cin>>multispectral;
				}
				if(argc > 4){
					paralell = stoi(argv[4]);
				}else{
					cout<< "Desea implementar calculo paralelo? (0 o 1)";
					cin>>paralell;
				}
				if(argc > 5){
					sobreescribir = stoi(argv[4]);
				}else{
					cout<< "Desea sobre escribir los indices que ya hayan sido calculados? (0 o 1)";
					cin>>sobreescribir;
				}
				IndexCalculation *uavIndex;
				uavIndex = new IndexCalculation(otputSticthing,multispectral,paralell,sobreescribir);
				uavIndex->processManager();
			}
			break;
			case 4:{
				UtilInformation ui;
				ui.selectCamera();
				ui.calcularHectareas();
			}
			break;
			case 5:{

				Redaction redact;
				redact.generatePDF();

				// HPDF_Free (pdf);
			}
			break;
			case 6:{
				
			}
			break;
		}
		option = 6;

	}
	// cout<<"Proceso finalizado, ahora que deseas hacer? \n 1: Calibrar camara \n 2: Calcular orthomosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
}