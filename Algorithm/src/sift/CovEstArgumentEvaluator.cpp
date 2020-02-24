#include "CovEstArgumentEvaluator.h"
void CovEstArgumentEvaluator::evaluate(int argc, char* argv[]) {
	// Information on usage
	if(argc == 1) {
		cout << "USAGE:\n"
			<< "  ./covEstimate -t SIFT|SURF -i <img> [OPTIONS]\n"
			<< endl
			<< "REQUIRED PARAMETERS:\n"
			<< "  -t <type>    Detector type, SURF or SIFT\n"
			 << "  -i <img>     Image file to process.\n"
			 << endl
			 << "OPTIONS:\n"
			 << "  -k <file>    File ending of the keypoint file to load. The name itself\n"
			 << "               is according to the name of the image.\n"
			 << "               (default is " << FE_KEYS_SIFT << "|" << FE_KEYS_SURF << ")\n"
			 << "  -d <dir>     The directory containing the image file (default ist ./)\n"
			 << "  -q           Quite prossing. Don't print status information.\n"
			 << "  -show        Display the estimated uncertainty for feature points in the\n"
			 << "               image, each as ellipse according to the covariance matrix.\n"
			 << "  -save        Draw the covariance in the image and save to file.\n";

		return;
	}

	// parse arguments
	int arg = 0;
// 	while (++arg < argc) { 
// 		if (! strcmp(argv[arg], "-t")) {
// 			string decTypeName( argv[++arg] );
// 			if(! strcmp(decTypeName.c_str(), "SIFT") )
// 				decType = DETECTOR_SIFT;
// 			else if(! strcmp(decTypeName.c_str(), "SURF") )
// 				decType = DETECTOR_SURF;
// 			else {
// 				cout << "COV Error: Unknown detector/descriptor type." << endl;
// 				exit(1);
// 			}
// 		}
// 		else if (! strcmp(argv[arg], "-d"))
// 			imgDir = argv[++arg];
// 		else if (! strcmp(argv[arg], "-i"))
// 			imgFile = argv[++arg]; 
// 		else if (! strcmp(argv[arg], "-k"))
// 			keyFileEnding = argv[++arg];
// 		else if (! strcmp(argv[arg], "-q"))
// 			verbose = false;
// 		else if (! strcmp(argv[arg], "-save"))
// 			saveCov = true;
// 		else if (! strcmp(argv[arg], "-show"))
// 			showCov = true;
// 		else
// 			cout << "Warning: Skipping unknown parameter " << argv[arg] << "." << endl;
// 	}

	if(imgFile == NULL) {
		cout << "COV Error: No image file specified." << endl;
		return;
	}
	else {
		int i = 0;
		while(imgFile[i] != '.') imgFileStart[i] = imgFile[i++];
		imgFileStart[i] = 0;
	}

	if(keyFileEnding == NULL)
		switch (decType) {
			case DETECTOR_SIFT:
				keyFileEnding = FE_KEYS_SIFT;
				break;
			case DETECTOR_SURF:
				keyFileEnding = FE_KEYS_SURF;
				break;
		}
	
	keyFile.clear();
	keyFile.append(imgFileStart).append(keyFileEnding);

}