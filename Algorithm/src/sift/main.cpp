#include "sift_cov.h"

int main(int argc, char* argv[])
{
  if( argc < 2 ) cout << "Enter filename " << endl;
  SiftCov a(argv[1], (argc>2));
  a.detectFeatures();
  
  return 0;
}

