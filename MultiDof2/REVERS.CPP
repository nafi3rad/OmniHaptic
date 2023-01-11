//In the name of Allah

/******************************
 *  Reverse of Matrix         *
 *  By: Alireza Zakeri        *
 ******************************/

#include <iostream>
#include <cmath>
using namespace std;
const int MAXn = 500+10;
const double EPS = 1e-7;

struct equationSystem{
	int n;
	double mat[MAXn][MAXn], rev[MAXn][MAXn];

	inline int sgn(const double &a){
		return ( a < -EPS ? -1 : a > EPS );
	}
	inline bool EQ(const double &a, const double &b){
		return (abs(a-b) <= EPS);
	}
	inline void swp(const int &r1, const int &r2){
		for ( int i = 0; i < n; i++ ){
			swap( mat[r1][i] , mat[r2][i] );
			swap( rev[r1][i] , rev[r2][i] );
		}
	}
	bool findReverse(){
		memset(rev, 0, sizeof(rev));
		for ( int i = 0; i < n; i++)
			rev[i][i] = 1;

		int col = 0, row = 0;
		for ( ; col < n; col++ ){
			
			for ( int i = row; i < n; i++ ){
				
				if ( sgn( mat[i][col] ) != 0 ){
					if (i!=row)
						swp(row, i);
					
					// make all in that column 0 exept the one in the row "row" that should be 1
					double mod = mat[row][col];
					for ( int j = 0; j < n; j++ ){
						mat[row][j] /= mod;
						rev[row][j] /= mod;
					}
					
					for ( int j = 0; j < n; j++ ){
						if ( j == row )	continue;
						mod = mat[j][col];
						for ( int k = 0; k < n; k++ ){
							mat[j][k] = mat[j][k] - mod*mat[row][k];
							rev[j][k] = rev[j][k] - mod*rev[row][k];
						}
					}
					row++;
					break;
				}
			}
		}
		for ( int i = 0; i < n; i++ )
			if ( !EQ(mat[i][i] , 1.))	return false;
		return true;
	}
	void Print(){
		for ( int i = 0; i < n; i++ ){
			for ( int j = 0; j < n; j++ )
				printf("%.4lf " ,rev[i][j]+EPS);
			printf("\n");
		}		
	}
}eq;


int main(){
	int n;
	scanf("%d", &n);// n*n matrix
	eq.n = n;
	
	for ( int i = 0; i < n; i++ ){
		for ( int j = 0; j < n; j++)
			scanf("%lf", &eq.mat[i][j]);
	}
	cout<<eq.findReverse()<<endl;
	eq.Print();
	return 0;
}

