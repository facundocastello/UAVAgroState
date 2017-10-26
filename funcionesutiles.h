#ifndef FUNCIONESUTILES_H
#define FUNCIONESUTILES_H
#include "pdi_functions.h"
using namespace std;
using namespace cv;
using namespace pdi;

///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS///OTROS
namespace fc {
	//recibe imagen y muestra BGR
	vector<Mat> mostrarBGR(Mat img) {
		vector<Mat> BGR(3);
		split(img, BGR);
		Mat auxBGR = mosaic(BGR, 1);
		namedWindow("BGR", WINDOW_NORMAL);
		imshow("BGR", auxBGR);
		waitKey();
		return BGR;
	}
	//Recibe vector BGR y lo muestra
	void mostrarBGR(vector<Mat> BGR) {
		Mat auxBGR = mosaic(BGR, 1);
		namedWindow("BGR", WINDOW_NORMAL);
		imshow("BGR", auxBGR);
		waitKey();
	}
	//recibe imagen y muestra BGR
	vector<Mat> mostrarHSV(Mat img) {
		cvtColor(img, img, CV_BGR2HSV);
		vector<Mat> HSV(3);
		split(img, HSV);
		if (img.depth() == CV_32F)
			normalize(HSV[0], HSV[0], 0, 1, CV_MINMAX);
		Mat auxHSV = mosaic(HSV, 1);
		if (img.depth() == CV_32F)
			normalize(HSV[0], HSV[0], 0, 240, CV_MINMAX);
		namedWindow("HSV", WINDOW_NORMAL);
		imshow("HSV", auxHSV);
		waitKey();
		cvtColor(img, img, CV_HSV2BGR);
		return HSV;
	}
	//recibe imagen y muestra BGR
	void mostrarHSV(vector<Mat> HSV) {
		if (HSV[1].depth() == CV_32F)
			normalize(HSV[0], HSV[0], 0, 1, CV_MINMAX);
		Mat auxHSV = mosaic(HSV, 1);
		if (HSV[1].depth() == CV_32F)
			normalize(HSV[0], HSV[0], 0, 240, CV_MINMAX);
		namedWindow("HSV", WINDOW_NORMAL);
		imshow("HSV", auxHSV);
		waitKey();
	}

	//Recorta una imagen en los puntos indicados
	Mat recortar(Mat img, int y0, int y1, int x0, int x1) {
		Rect croppedRect = Rect(x0, y0, x1 - x0, y1 - y0);
		Mat CroppedImage = img(croppedRect);
		return CroppedImage;
	}

	//Devuelve las posiciones del area de interes de una imagen
	vector<int> ROI(Mat img, float roni1, float roni2) {
		vector<int> a(4);
		if (img.depth() != CV_32F) {
			bool seguircontando = true;
			int contarfilasar = 0;
			int contarfilasab = 0;
			int contarcoliz = 0;
			int contarcolder = 0;
			for (int i = 0; i<img.rows; i++) {
				for (int j = 0; j<img.cols; j++) {
					if (!(img.at<uchar>(i, j) >= roni1 && img.at<uchar>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarfilasar++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int j = 0; j<img.cols; j++) {
				for (int i = 0; i<img.rows; i++) {
					if (!(img.at<uchar>(i, j) >= roni1 && img.at<uchar>(i, j) <= roni2)) {
						seguircontando = false;
						break;
					}
				}
				if (seguircontando) {
					contarcoliz++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int i = img.rows - 1; i >= 0; i--) {
				for (int j = img.cols - 1; j >= 0; j--) {
					if (!(img.at<uchar>(i, j) >= roni1 && img.at<uchar>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarfilasab++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int j = img.cols - 1; j >= 0; j--) {
				for (int i = img.rows - 1; i >= 0; i--) {
					if (!(img.at<uchar>(i, j) >= roni1 && img.at<uchar>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarcolder++;
				}
				else {
					break;
				}
			}
			cout << endl << "arriba: " << contarfilasar << "derecha: " << contarcolder << "abajo:  " << contarfilasab << "izquierda: " << contarcoliz << endl;
			a[0] = contarfilasar;
			a[1] = contarcolder;
			a[2] = contarfilasab;
			a[3] = contarcoliz;
		}
		else {
			bool seguircontando = true;
			int contarfilasar = 0;
			int contarfilasab = 0;
			int contarcoliz = 0;
			int contarcolder = 0;
			for (int i = 0; i<img.rows; i++) {
				for (int j = 0; j<img.cols; j++) {
					if (!(img.at<float>(i, j) >= roni1 && img.at<float>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarfilasar++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int j = 0; j<img.cols; j++) {
				for (int i = 0; i<img.rows; i++) {
					if (!(img.at<float>(i, j) >= roni1 && img.at<float>(i, j) <= roni2)) {
						seguircontando = false;
						break;
					}
				}
				if (seguircontando) {
					contarcoliz++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int i = img.rows - 1; i >= 0; i--) {
				for (int j = img.cols - 1; j >= 0; j--) {
					if (!(img.at<float>(i, j) >= roni1 && img.at<float>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarfilasab++;
				}
				else {
					break;
				}
			}
			seguircontando = true;
			for (int j = img.cols - 1; j >= 0; j--) {
				for (int i = img.rows - 1; i >= 0; i--) {
					if (!(img.at<float>(i, j) >= roni1 && img.at<float>(i, j) <= roni2)) {
						seguircontando = false;
					}
				}
				if (seguircontando) {
					contarcolder++;
				}
				else {
					break;
				}
			}
			cout << endl << "arriba: " << contarfilasar << "derecha: " << contarcolder << "abajo:  " << contarfilasab << "izquierda: " << contarcoliz << endl;
			a[0] = contarfilasar;
			a[1] = contarcolder;
			a[2] = contarfilasab;
			a[3] = contarcoliz;
		}
		return a;
	}
	//Devuelve un Mat con la imagen enmascarada en un area de interes, o recortada
	Mat ROI(Mat img, float roni1, float roni2, bool mascara) {
		Mat img2;
		vector<int> a;
		a = ROI(img, roni1, roni2);
		if (mascara) {
			img2 = img.clone();
			CvPoint pt1, pt2;
			pt1.x = a[3];
			pt1.y = a[0];
			pt2.x = img.cols - a[1];
			pt2.y = img.rows - a[2];
			rectangle(img2, pt1, pt2, CV_RGB(255, 255, 255), -1, 8, 0);
		}
		else {
			img2 = recortar(img, a[0], (img.rows - a[2]), a[3], (img.cols - a[1]));
		}
		return img2;
	}


	//devuelve imagen cortada
	Mat cortarROI(Mat img, vector<int> a) {
		img = recortar(img, a[0], (img.rows - a[2]), a[3], (img.cols - a[1]));
		return img;
	}
	//Devuelve la fila o columna en la que menos aparece 'Pixel'
	int mayorinfo(Mat img, float pixel, bool vertical) {
		//pixel es el color del pixel que quiero que menos aparezca

		int contador = 0;
		int contmin = 0;
		int indice = 0;
		if (img.depth() != CV_8U) {
			cout << endl << "Tiene que ser CV_8U" << endl;

		}
		else {
			if (vertical) {
				contmin = img.rows;
				for (int i = 0; i<img.cols; i++) {
					contador = 0;
					for (int j = 0; j<img.rows; j++) {
						if (img.at<uchar>(j, i) == pixel)
							contador++;
					}
					if (contador<contmin) {
						indice = i;
						contmin = contador;
					}
				}
			}
			else {
				contmin = img.cols;
				for (int i = 0; i<img.rows; i++) {
					contador = 0;
					for (int j = 0; j<img.cols; j++) {
						if (img.at<uchar>(i, j) == pixel)
							contador++;
					}
					if (contador<contmin) {
						contmin = contador;
						indice = i;
					}
				}
			}
		}
		return indice;
	}

	//Grafica el perfil de intensidad de una columa o fila
	void perfilIntensidad(Mat img, int val, bool vertical = true, bool bordes = true, float tol = 0.1) {
		if (img.depth() != CV_8U) {
			cout << endl << "Tiene que ser CV_8U" << endl;

		}
		else {
			if (vertical) {
				Mat img2(img.rows, 256, CV_8UC1, Scalar(0));
				for (int i = 0; i<img.rows; i++) {
					int aux = img.at<uchar>(i, val);
					if (bordes && i>0 && (abs(aux - img.at<uchar>(i - 1, val))>(255 * tol))) {
						for (int j = 0; j<255; j++)
							img2.at<uchar>(i, j) = 255;
					}
					img2.at<uchar>(i, 255 - aux) = 255;
				}
				imshow("Perfil Intensidad", img2);
				waitKey();
			}
			else {
				Mat img2(256, img.cols, CV_8UC1, Scalar(0));
				for (int i = 0; i<img.cols; i++) {
					int aux = img.at<uchar>(val, i);
					if (bordes && i>0 && (abs(aux - img.at<uchar>(val, i - 1))>(255 * tol))) {
						for (int j = 0; j<255; j++)
							img2.at<uchar>(j, i) = 255;
					}
					img2.at<uchar>(255 - aux, i) = 255;
				}
				imshow("Perfil Intensidad", img2);
				waitKey();
			}
		}
	}
	//Calcula el error cuadratico medio entre 2 imagene
	float ECM(Mat img1, Mat img2) {
		float error = 0;
		if (img1.depth() != CV_32F || img2.depth() != CV_32F) {
			cout << endl << "TIENE Q SER CV32F" << endl;
		}
		else {
			if (img1.rows == img2.rows && img1.cols == img2.cols) {
				for (int i = 0; i<img1.rows; i++) {
					for (int j = 0; j<img1.cols; j++) {
						error += pow(img1.at<float>(i, j) - img2.at<float>(i, j), 2);
					}
				}
				error = sqrt(error / (img1.rows*img1.cols));
			}
			else {
				cout << endl << "TIENEN QUE TENER EL MISMO TAMANO" << endl;
			}
		}
		return error;
	}

	Mat dst;
	Mat imgthres;
	string window_name3 = "imagen threshold";
	int tolth = 0;
	int max_tolth = 255;
	void thresTB(int, void*) {
		threshold(imgthres, dst, tolth, 255, 0);
		imshow(window_name3, dst);
	}
	void thresTB(Mat img) {
		if (img.depth() != CV_8U) {
			cout << endl << "TIENE QUE SER CV_8U" << endl;
		}
		else {
			imgthres = img;
			namedWindow(window_name3, CV_WINDOW_AUTOSIZE);
			/// Create Trackbar to select Morphology operation
			createTrackbar("Tol",
				window_name3, &tolth, max_tolth, thresTB);
			thresTB(0, 0);
			waitKey(0);
		}
	}

	Mat rellenoBordes(Mat imagen) {
		if (imagen.depth() != CV_8U) {
			cout << endl << "LA IMAGEN DEBE SER CV_8U" << endl;
			return imagen;
		}
		else {
			//Separo en regiones
			RNG rng(12345);
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			/// Find contours
			findContours(imagen, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			/// Draw contours
			Mat drawing = Mat::zeros(imagen.size(), CV_8UC1);
			for (int i = 0; i< contours.size(); i++)
			{
				drawContours(drawing, contours, i, Scalar(255), -1, 8, hierarchy, 0, Point());
			}

			return(drawing);
		}
	}

	float contarpixeles(Mat img, int pixel) {
		float area = 0;
		if (img.depth() != CV_8U) {
			cout << endl << "LA IMAGEN DEBE SER CV_8U" << endl;
		}
		else {
			for (int i = 0; i<img.rows; i++) {
				for (int j = 0; j<img.cols; j++) {
					if (img.at<uchar>(i, j) == pixel)
						area++;
				}
			}
		}
		return area;
	}

};


#endif
