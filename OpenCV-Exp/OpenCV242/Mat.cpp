#include <opencv2\opencv.hpp>

// calculate the cofactor of element (row,col)

int GetMinor(float **src, float **dest, int row, int col, int order)

{

    // indicate which col and row is being copied to dest

    int colCount=0,rowCount=0;

 

    for(int i = 0; i < order; i++ )

    {
        if( i != row )

        {

            colCount = 0;

            for(int j = 0; j < order; j++ )

            {

                // when j is not the element

                if( j != col )

                {

                    dest[rowCount][colCount] = src[i][j];

                    colCount++;

                }

            }

            rowCount++;

        }

    }
    return 1;
}

 

// Calculate the determinant recursively.

double CalcDeterminant( float **mat, int order)
{
    // order must be >= 0
    // stop the recursion when matrix is a single element
    if( order == 1 )
        return mat[0][0];

    // the determinant value
	float det = 0;
	
	// allocate the cofactor matrix
    float **minor;

    minor = new float*[order-1];

    for(int i=0;i<order-1;i++)

        minor[i] = new float[order-1];

 

    for(int i = 0; i < order; i++ )

    {
       // get minor of element (0,i)

        GetMinor( mat, minor, 0, i , order);

        // the recusion is here!

 

        det += (i%2==1?-1.0:1.0) * mat[0][i] * CalcDeterminant(minor,order-1);

        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );

    }

    // release memory

    for(int i=0;i<order-1;i++)

        delete [] minor[i];
    delete [] minor;
    return det;
}

// matrix inversioon
// the result is put in Y
void MatrixInversion(float **A, int order, float **Y)
{
    // get the determinant of a
    double det = 1.0/CalcDeterminant(A,order);

    // memory allocation

    float *temp = new float[(order-1)*(order-1)];

    float **minor = new float*[order-1];
    for(int i=0;i<order-1;i++)
        minor[i] = temp+(i*(order-1));

 

    for(int j=0;j<order;j++)

    {
        for(int i=0;i<order;i++)
        {
            // get the co-factor (matrix) of A(j,i)
            GetMinor(A,minor,j,i,order);
            Y[i][j] = det*CalcDeterminant(minor,order-1);
            if( (i+j)%2 == 1)
                Y[i][j] = -Y[i][j];

        }

    }

    // release memory

    //delete [] minor[0];
	delete [] temp;

    delete [] minor;

}

 



void print(cv::Mat* matriz){
	for(int i = 0 ; i < matriz->rows ; i++){
		double* m = matriz->ptr<double>(i);
		for(int j = 0 ; j < matriz->cols ; j++){
			printf("%.4f ",m[j]);
		}
		printf("\n");
	}
}

using namespace std;
using namespace cv;

void printMatrix(Mat M){
cout<<"Tipo de matriz:"<<M.type()<<endl;
 // dont print empty matrices
  if (M.empty()){
    cout << "---" << endl;
    return;
  }
  // loop through columns and rows of the matrix
  for(int i=0; i < M.rows; i++){
      for(int j=0; j < M.cols ; j++){
      //cout << M.ptr<double>(i)[j] << ", "<<endl;
		  printf("%.4f ",M.ptr<double>(i)[j]);
      }
    printf("\n");
	}
}


int main( int argc, char* argv[] ){
	Mat homogra(3,3,CV_64FC1);
	Mat coord(3,1,CV_64FC1);
	//Mat result(3,3,CV_8UC1);

	homogra.ptr<double>(0)[0] = 1;
	homogra.ptr<double>(0)[1] = 0;
	homogra.ptr<double>(0)[2] = 0;
	homogra.ptr<double>(1)[0] = 0;
	homogra.ptr<double>(1)[1] = 1./sqrt(2.);
	homogra.ptr<double>(1)[2] = -1./sqrt(2.);
	homogra.ptr<double>(2)[0] = 0;
	homogra.ptr<double>(2)[1] = 1./sqrt(2.);
	homogra.ptr<double>(2)[2] = 1./sqrt(2.);

	printMatrix(homogra);

	Mat inverse = homogra.inv(1);

	printMatrix(inverse);

	Vec3f point(0,1,1);

	Vec3f result;
	result.val[0] = inverse.row(0).at<double>(0) * point.val[0] + 
					inverse.row(0).at<double>(1) * point.val[1] +
					inverse.row(0).at<double>(2) * point.val[2];

	result.val[1] = inverse.row(1).at<double>(0) * point.val[0] + 
				inverse.row(1).at<double>(1) * point.val[1] +
				inverse.row(1).at<double>(2) * point.val[2];

	result.val[2] = inverse.row(2).at<double>(0) * point.val[0] + 
			inverse.row(2).at<double>(1) * point.val[1] +
			inverse.row(2).at<double>(2) * point.val[2];

	//cv::Vec3f _vec_1(1.,0.,0.);
	//cv::Vec3f _vec_2(0.,(1.0/sqrt(2.)),(1.0/sqrt(2.)));
	////cv::Vec3f _vec_2(0,1,0);
	//cv::Vec3f _vec_3(0.,-(1.0/sqrt(2.)),(1.0/sqrt(2.)));
	////cv::Vec3f _vec_3(0,0,1);

	//cv::Point3f _p0(0,1,1);

	////float **_mat;
	////float **_res;

	////_mat = (float**)malloc(sizeof(float*)*3);
	////_res = (float**)malloc(sizeof(float*)*3);
	////for(int i = 0 ; i < 3 ; i++){
	////	_mat[i] = (float*)malloc(sizeof(float)*3);
	////	_res[i] = (float*)malloc(sizeof(float)*3);
	////}

	////_mat[0][0] = _vec_1.val[0];
	////_mat[1][0] = _vec_1.val[1];
	////_mat[2][0] = _vec_1.val[2];
	////_mat[0][1] = _vec_2.val[0];
	////_mat[1][1] = _vec_2.val[1];
	////_mat[2][1] = _vec_2.val[2];
	////_mat[0][2] = _vec_3.val[0];
	////_mat[1][2] = _vec_3.val[1];
	////_mat[2][2] = _vec_3.val[2];

	////for(int i = 0 ; i < 3 ; i++){
	////	for(int j = 0 ; j < 3 ; j++){
	////		printf("%.4f ",_mat[i][j]);
	////	}
	////	printf("\n");
	////}

	////MatrixInversion(_mat,3,_res);

	////printf("INV\n");
	////for(int i = 0 ; i < 3 ; i++){
	////	for(int j = 0 ; j < 3 ; j++){
	////		printf("%.4f ",_res[i][j]);
	////	}
	////	printf("\n");
	////}

	//cv::Mat3f matriz(3,3,CV_64FC1);
	//matriz.ptr<double>(0)[0] = _vec_1.val[0];
	//matriz.ptr<double>(1)[0] = _vec_1.val[1];
	//matriz.ptr<double>(2)[0] = _vec_1.val[2];
	//matriz.ptr<double>(0)[1] = _vec_2.val[0];
	//matriz.ptr<double>(1)[1] = _vec_2.val[1];
	//matriz.ptr<double>(2)[1] = _vec_2.val[2];
	//matriz.ptr<double>(0)[2] = _vec_3.val[0];
	//matriz.ptr<double>(1)[2] = _vec_3.val[1];
	//matriz.ptr<double>(2)[2] = _vec_3.val[2];


	//print(&matriz);
	//
	//cv::Mat3f inv(3,3,CV_64FC1);

	//inv = matriz.inv();
	//print(&inv);


	//cv::Mat3f _aux(3,1);	
	//_aux.ptr<double>(0)[0] = _p0.x;
	//_aux.ptr<double>(1)[0] = _p0.y;
	//_aux.ptr<double>(2)[0] = _p0.z;

	//cv::Mat3f _result(3,1);
	//_result.ptr<double>(0)[0] = _p0.x;
	//_result.ptr<double>(1)[0] = _p0.y;
	//_result.ptr<double>(2)[0] = _p0.z;

	//print(&_result);

	getchar();
	
	return 0;
}