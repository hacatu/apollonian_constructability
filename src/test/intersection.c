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

int test_LL1_quartet(){
	Point X, A, B;
	do{
		X = random_point(0, 0, 100, 100);
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
	}while(eq_points(&X, &A) || eq_points(&X, &B) || eq_points(&A, &B));
	Line a, b;
	ApproxCons base = {};
	int ret = 0;
	construct_line(&a, &A, &X);
	construct_line(&b, &B, &X);
	ret += intersect_LL(&base, &a, &b) && eq_points(&X, base.points);
	a.dx = -a.dx;
	a.dy = -a.dy;
	base.points_len = 0;
	ret += intersect_LL(&base, &a, &b) && eq_points(&X, base.points);
	b.dx = -b.dx;
	b.dy = -b.dy;
	base.points_len = 0;
	ret += intersect_LL(&base, &a, &b) && eq_points(&X, base.points);
	a.dx = -a.dx;
	a.dy = -a.dy;
	base.points_len = 0;
	intersect_LL(&base, &a, &b);
	return ret + (intersect_LL(&base, &a, &b) && eq_points(&X, base.points));
}

int test_LL0_pair(){
	Point X, A, B;
	do{
		X = random_point(0, 0, 100, 100);
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
	}while(eq_points(&X, &A) || eq_points(&X, &B) || eq_points(&A, &B));
	Line a, b;
	ApproxCons base = {};
	int ret = 0;
	construct_line(&a, &A, &X);
	X.x += B.x - A.x;
	X.y += B.y - A.y;
	construct_line(&b, &B, &X);
	ret += !intersect_LL(&base, &a, &b);
	a.dx = -a.dx;
	a.dy = -a.dy;
	base.points_len = 0;
	ret += !intersect_LL(&base, &a, &b);
	return ret;
}

int test_CL2_pair(){
	Point X1, X2, S;
	do{
		X1 = random_point(0, 0, 100, 100);
		X2 = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 1, 1);
	}while(eq_points(&X1, &X2) || fabs(S.x) < EPSILON);
	Point A = {X1.x + S.x*(X2.x - X1.x), X1.y + S.x*(X2.y - X1.y)};
	Point B = {(X1.x + X2.x)/2 + S.y*(X1.y - X2.y), (X1.y + X2.y)/2 + S.y*(X2.x - X1.x)};
	Line l;
	Circle c;
	ApproxCons base = {};
	int ret = 0;
	construct_line(&l, &A, &X1);
	construct_circle(&c, &B, &X1);
	if(intersect_CL(&base, &c, &l) == 2){
		if(eq_points(&X1, base.points)){
			ret += eq_points(&X2, base.points + 1);
		}else if(eq_points(&X1, base.points + 1)){
			ret += eq_points(&X2, base.points);
		}
	}
	l.dx = -l.dx;
	l.dy = -l.dy;
	base.points_len = 0;
	if(intersect_CL(&base, &c, &l) == 2){
		if(eq_points(&X1, base.points)){
			ret += eq_points(&X2, base.points + 1);
		}else if(eq_points(&X1, base.points + 1)){
			ret += eq_points(&X2, base.points);
		}
	}
	return ret;
}

int test_CL1_pair(){
	Point X, B, S;
	do{
		X = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 1, 1);
	}while(eq_points(&X, &B) || fabs(S.x) < EPSILON);
	Point A = {X.x + S.x*(B.y - X.y), X.y + S.x*(X.x - B.x)};
	Line l;
	Circle c;
	ApproxCons base = {};
	int ret = 0;
	construct_line(&l, &A, &X);
	construct_circle(&c, &B, &X);
	if(intersect_CL(&base, &c, &l) == 1){
		ret += eq_points(&X, base.points);
	}
	l.dx = -l.dx;
	l.dy = -l.dy;
	base.points_len = 0;
	if(intersect_CL(&base, &c, &l) == 1){
		ret += eq_points(&X, base.points);
	}
	return ret;
}

int test_CL0_single(){
	Point X, B, S;
	do{
		X = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 1, 0.5);
	}while(eq_points(&X, &B) || fabs(S.x) < EPSILON || fabs(S.y) < EPSILON || fabs(S.y) > 1 - EPSILON);
	Point A = {X.x + S.x*(B.y - X.y), X.y + S.x*(X.x - B.x)};
	Point R = {B.x + S.y*(X.x - B.x), B.y + S.y*(X.y - B.y)};
	Line l;
	Circle c;
	ApproxCons base = {};
	construct_line(&l, &A, &X);
	construct_circle(&c, &B, &R);
	if(S.y < 0){
		l.dx = -l.dx;
		l.dy = -l.dy;
	}
	return !intersect_CL(&base, &c, &l);
}

int test_CC2_single(){
	Point X1, X2, S;
	do{
		X1 = random_point(0, 0, 100, 100);
		X2 = random_point(0, 0, 100, 100);
		S = random_point(0, 0, 1, 1);
	}while(eq_points(&X1, &X2) || fabs(S.x) < EPSILON || fabs(S.y) < EPSILON || fabs(S.x - S.y) < EPSILON);
	Point M = {(X1.x + X2.x)/2, (X1.y + X2.y)/2};
	double dx = X1.y - X2.y;
	double dy = X2.x - X1.x;
	Point A = {M.x + S.x*dx, M.y + S.x*dy};
	Point B = {M.x + S.y*dx, M.y + S.y*dy};
	Circle a, b;
	ApproxCons base = {};
	construct_circle(&a, &A, &X1);
	construct_circle(&b, &B, &X1);
	if(intersect_CC(&base, &a, &b) == 2){
		if(eq_points(&X1, base.points)){
			return eq_points(&X2, base.points + 1);
		}else if(eq_points(&X1, base.points + 1)){
			return eq_points(&X2, base.points);
		}
	}
	return 0;
}

int test_CC1_single(){
	Point A, X, R;
	double dx, dy, h;
	do{
		A = random_point(0, 0, 100, 100);
		X = random_point(0, 0, 100, 100);
		R = random_point(0, 0, 100, 100);
		dx = A.x - X.x;
		dy = A.y - X.y;
		h = hypot(dx, dy);
	}while(eq_points(&A, &X) || fabs(R.x) < EPSILON || fabs(fabs(R.x) - h) < EPSILON);
	Circle a = {A.x, A.y, h};
	h = R.x/h;
	Circle b = {X.x + h*dx, X.y + h*dy, fabs(R.x)};
	ApproxCons base = {};
	int ret = 0;
	if(intersect_CC(&base, &a, &b) == 1){
		ret += eq_points(&X, base.points);
	}
	return ret;
}

int test_CC0_single(){
	Point A, B, R;
	do{
		A = random_point(0, 0, 100, 100);
		B = random_point(0, 0, 100, 100);
		R = random_point(0, 0, 100, 100);
	}while(eq_points(&A, &B) || R.x + R.y > hypot(B.x - A.x, B.y - A.y) - EPSILON);
	Circle a = {A.x, A.y, R.x};
	Circle b = {B.x, B.y, R.y};
	ApproxCons base = {};
	return !intersect_CC(&base, &a, &b);
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
	int total = 8, passed = 0
	+ test_n_times(test_LL1_quartet, "LL1", 10000, 4)
	+ test_n_times(test_LL0_pair, "LL0", 10000, 2)
	+ test_n_times(test_CL2_pair, "CL2", 10000, 2)
	+ test_n_times(test_CL1_pair, "CL1", 10000, 2)
	+ test_n_times(test_CL0_single, "CL0", 10000, 1)
	+ test_n_times(test_CC2_single, "CC2", 10000, 1)
	+ test_n_times(test_CC1_single, "CC1", 10000, 1)
	+ test_n_times(test_CC0_single, "CC0", 10000, 1);
	if(passed == total){
		printf("\e[1;32mPassed %d/%d tests\e[0m\n", passed, total);
	}else{
		printf("\e[1;31mFailed %d/%d tests\e[0m\n", total - passed, total);
	}
}

