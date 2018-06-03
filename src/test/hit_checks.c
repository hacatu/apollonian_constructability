#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"

static Point random_point(double ux, double uy, double dx, double dy){
	double u, v, s;
	do{
		u = (double)rand()/RAND_MAX*2 - 1;
		v = (double)rand()/RAND_MAX*2 - 1;
		s = u*u + v*v;
	}while(s >= 1 || s == 0);
	double q = sqrt(-2*log(s)/s);
	return (Point){u*q*dx + ux, v*q*dy + uy};
}

int test_eqL_pair(){
	Point A, B, F;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		F = random_point(0, 0, EPSILON*EPSILON, EPSILON*EPSILON);
	}while(eq_points(&A, &B));
	Line a;
	construct_line(&a, &A, &B);
	Line b = {B.x, B.y, a.dx + F.x, a.dy + F.y};
	double h = hypot(b.dx, b.dy);
	b.dx /= h;
	b.dy /= h;
	int ret = 0;
	ret += eq_lines(&a, &b);
	a.dx = -a.dx;
	a.dy = -a.dy;
	ret += eq_lines(&a, &b);
	return ret;
}

//Not EQual Lines in different Directions
int test_neqL_d_pair(){
	Point D1, D2;
	do{
		D1 = random_point(0, 0, 100, 100);
		D2 = random_point(0, 0, 100, 100);
		double h = hypot(D1.x, D1.y);
		if(h < EPSILON){
			continue;
		}
		D1.x /= h;
		D1.y /= h;
		h = hypot(D2.x, D2.y);
		if(h < EPSILON){
			continue;
		}
		D2.x /= h;
		D2.y /= h;
	}while(fabs(D1.x*D2.x + D1.y*D2.y) > 1 - EPSILON);
	Line a = {0, 0, D1.x, D1.y};
	Line b = {0, 0, D2.x, D2.y};
	int ret = 0;
	ret += !eq_lines(&a, &b);
	a.dx = -a.dx;
	a.dy = -a.dy;
	ret += !eq_lines(&a, &b);
	return ret;
}

//Not EQual Lines that are Parallel
int test_neqL_p_pair(){
	Point A, B, F, S;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		F = random_point(0, 0, EPSILON/6.6, EPSILON/6.6);
		S = random_point(0, 0, 100, 100);
	}while(eq_points(&A, &B) || S.x < EPSILON);
	Line a;
	construct_line(&a, &A, &B);
	Line b = {B.x + S.x*-a.dy, B.y + S.x*a.dx, a.dx + F.x, a.dy + F.y};
	double h = hypot(b.dx, b.dy);
	b.dx /= h;
	b.dy /= h;
	int ret = 0;
	ret += !eq_lines(&a, &b);
	a.dx = -a.dx;
	a.dy = -a.dy;
	ret += !eq_lines(&a, &b);
	return ret;
}

int test_eqC_single(){
	Point A, F, R;
	do{
		A = random_point(0, 0, 100, 100);
		F = random_point(0, 0, EPSILON/6.6, EPSILON/6.6);
		R = random_point(0, 0, 100, EPSILON/6.6);
		R.x = fabs(R.x);
	}while(R.x < EPSILON);
	Circle a = {A.x, A.y, R.x};
	Circle b = {A.x + F.x, A.y + F.y, R.x + R.y};
	int ret = eq_circles(&a, &b);
	return ret;
}

//Not EQual Circles with different Centers
int test_neqC_c_single(){
	Point A, B, R;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		R = random_point(0, 0, 100, EPSILON/6.6);
		R.x = fabs(R.x);
	}while(eq_points(&A, &B) || R.x < EPSILON);
	Circle a = {A.x, A.y, R.x};
	Circle b = {B.x, B.y, R.x + R.y};
	int ret = !eq_circles(&a, &b);
	return ret;
}

//Not EQual Circles with different Radii
int test_neqC_r_single(){
	Point A, F, R;
	do{
		A = random_point(0, 0, 100, 100);
		F = random_point(0, 0, EPSILON/6.6, EPSILON/6.6);
		R = random_point(0, 0, 100, 100);
		R.x = fabs(R.x);
		R.y = fabs(R.y);
	}while(R.x < EPSILON || R.y < EPSILON || fabs(R.x - R.y) < EPSILON);
	Circle a = {A.x, A.y, R.x};
	Circle b = {A.x + F.x, A.y + F.x, R.y};
	int ret = !eq_circles(&a, &b);
	return ret;
}

int test_eqP_single(){
	Point A = random_point(0, 0, 100, 100);
	Point F = random_point(0, 0, EPSILON/6.6, EPSILON/6.6);
	Point B = {A.x + F.x, A.y + F.y};
	int ret = eq_points(&A, &B);
	return ret;
}

int test_neqP_single(){
	Point A, B;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
	}while(eq_points(&A, &B));
	int ret = !eq_points(&A, &B);
	return ret;
}

int test_onL_single(){
	Point A, B, S;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 100, EPSILON/6.6);
	}while(eq_points(&A, &B));
	Line l;
	construct_line(&l, &A, &B);
	Point p = {l.x + S.x*l.dx + S.y*-l.dy, l.y + S.x*l.dy + S.y*l.dx};
	int ret = is_on_line(&l, &p);
	return ret;
}

int test_offL_single(){
	Point A, B, S;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 100, 100);
	}while(eq_points(&A, &B) || fabs(S.y) < EPSILON);
	Line l;
	construct_line(&l, &A, &B);
	Point p = {l.x + S.x*l.dx + S.y*-l.dy, l.y + S.x*l.dy + S.y*l.dx};
	int ret = !is_on_line(&l, &p);
	return ret;
}

int test_onC_single(){
	Point A, D, R;
	do{
		A = random_point(0, 0, 100, 100);
		D = random_point(0, 0, 100, 100);
		R = random_point(0, 0, 100, EPSILON/6.6);
		double h = hypot(D.x, D.y);
		if(h < EPSILON){
			continue;
		}
		D.x /= h;
		D.y /= h;
		R.x = fabs(R.x);
	}while(R.x < EPSILON);
	Circle c = {A.x, A.y, R.x};
	double a = R.x + R.y;
	Point p = {A.x + a*D.x, A.y + a*D.y};
	int ret = is_on_circle(&c, &p);
	return ret;
}

int test_offC_single(){
	Point A, D, R;
	do{
		A = random_point(0, 0, 100, 100);
		D = random_point(0, 0, 100, 100);
		R = random_point(0, 0, 100, 100);
		double h = hypot(D.x, D.y);
		if(h < EPSILON){
			continue;
		}
		D.x /= h;
		D.y /= h;
		R.x = fabs(R.x);
	}while(R.x < EPSILON || fabs(R.x - fabs(R.y)) < EPSILON);
	Circle c = {A.x, A.y, R.x};
	double a = R.x + R.y;
	Point p = {a*D.x, a*D.y};
	int ret = !is_on_circle(&c, &p);
	return ret;
}

int test_n_times(int (*test_fn)(void), const char *name, int n, int m){
	int total = n*m, passed = 0;
	for(int i = 0; i < n; ++i){
		passed += test_fn();
	}
	if(passed == total){
		printf("\e[1;32mPassed all %d/%d %s tests\e[0m\n", passed, total, name);
		return 1;
	}
	printf("\e[1;31mFailed %d/%d %s tests\e[0m\n", total - passed, total, name);
	return 0;
}

int main(){
	int total = 12, passed = 0
	+ test_n_times(test_eqL_pair, "eqL", 10000, 2)
	+ test_n_times(test_neqL_d_pair, "neqL_d", 10000, 2)
	+ test_n_times(test_neqL_p_pair, "neqL_p", 10000, 2)
	+ test_n_times(test_eqC_single, "eqC", 10000, 1)
	+ test_n_times(test_neqC_c_single, "neqC_c", 10000, 1)
	+ test_n_times(test_neqC_r_single, "neqC_r", 10000, 1)
	+ test_n_times(test_eqP_single, "eqP", 10000, 1)
	+ test_n_times(test_neqP_single, "neqP", 10000, 1)
	+ test_n_times(test_onL_single, "onL", 10000, 1)
	+ test_n_times(test_offL_single, "offL", 10000, 1)
	+ test_n_times(test_onC_single, "onC", 10000, 1)
	+ test_n_times(test_offC_single, "offC", 10000, 1);
	if(passed == total){
		printf("\e[1;32mPassed %d/%d tests\e[0m\n", passed, total);
	}else{
		printf("\e[1;31mFailed %d/%d tests\e[0m\n", total - passed, total);
	}
}

