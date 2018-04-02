#ifndef Redaction_H
#define Redaction_H

#include "hpdf.h"
#include <setjmp.h>
#include "IndexCalculation.h"


class Redaction{
    public:
        int generatePDF(){
            HPDF_Doc pdf;
			HPDF_Error_Handler error_handler;
			jmp_buf env;

			pdf = HPDF_New (error_handler, NULL);
			if (!pdf) {
				printf ("ERROR: cannot create pdf object.\n");
				return 1;
			}
			if (setjmp(env)) {
				HPDF_Free (pdf);
				return 1;
			}

			IndexCalculation ic;
			vector<string> strImgsName = ic.obtenerMSOutput();
			vector<string> strNDVI = CommonFunctions::obtenerImagenes((strImgsName[0]+"/").c_str());
			vector<string> strImgs = CommonFunctions::obtenerImagenes((strNDVI[0]+"/").c_str());
			for(int i = 0 ; i < strImgsName.size() ; i++){
				vector<string> strNDVI = CommonFunctions::obtenerImagenes((strImgsName[i]+"/").c_str());
				for(int j = 0 ; j < strNDVI.size() ; j++){
					pdf = HPDF_New (error_handler, NULL);
					HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);
					vector<string> strImgs = CommonFunctions::obtenerImagenes((strNDVI[j]+"/").c_str());
					for(int k = 0 ; k < strImgs.size(); k++){
						HPDF_Page page_1;
						page_1 = HPDF_AddPage (pdf);
						writeFullPageImg(pdf, page_1, strImgs[k].c_str());
					}
					CommonFunctions::escribirPDF(pdf,"Data/Informes/" + 
						CommonFunctions::obtenerUltimoDirectorio2(strImgsName[i]) + 
						CommonFunctions::obtenerUltimoDirectorio2(strNDVI[j]) );
				}
			}

			// HPDF_Page_SetSize (page_1, HPDF_PAGE_SIZE_B5, HPDF_PAGE_LANDSCAPE);
            return 0;

        }

        int writeFullPageImg(HPDF_Doc pdf, HPDF_Page page, const char* file){
            try {
				HPDF_Image img = HPDF_LoadPngImageFromFile(pdf,file);
				// maxw 600 maxh 840
				int maxWidth = 600;
				int maxHeight = 840;
				int xSpace = 50;
				int ySpace = 50;
				maxWidth -= 2*xSpace;
				maxHeight -= 2*ySpace;
				int width = HPDF_Image_GetWidth(img);
				int height = HPDF_Image_GetHeight(img);
				float resolucion;
				if(width > maxWidth){
					resolucion = (float)height/(float)width;
					width = maxWidth;
					height = width*resolucion;
				}
				if(height > maxHeight){
					resolucion = (float)width/(float)height;
					height = maxHeight;
					width = height*resolucion;
				}
				cout << resolucion << " " << width << " " << height; 
                xSpace += (maxWidth - width)/2;
                ySpace += (maxHeight - height)/2;
				HPDF_Page_DrawImage(page,img,xSpace,ySpace,width,height);		
			} catch (...) {
				HPDF_Free (pdf);
				return 1;
			}

            return 0;
        }


};

#endif