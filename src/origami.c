typedef struct{
	double x, y;
} Point;

typedef struct{
	double x, y, dx, dy;
} Line;

typedef enum{
	ORIGAMI_HH1;
	ORIGAMI_HH2;
	ORIGAMI_HH3;
	ORIGAMI_HH4;
	ORIGAMI_HH5;
	ORIGAMI_HH6;
	ORIGAMI_HH7;
} OrigamiAxiom;

typedef struct{
	OrigamiAxiom type;
	int i1, i2, i3, i4;
	int which;
} OrigamiStep;

typedef struct{
	Point points[MAX_ORIGAMI_POINTS];
	Line lines[MAX_ORIGAMI_LINES];
	OrigamiStep steps[MAX_ORIGAMI_STEPS];
	int points_len, lines_len, steps_len;
} Origami;

int add_line(Origami *self, const Line *l){
	return 0;
}

int dfs_fold(const Origami *base, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data);

int fold_HH1(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//1 solution
	Origami next = *base;
	next.steps[next.steps_len++];
	const Point *restrict p1 = next.points[step->i1];
	const Point *restrict p2 = next.points[step->i2];
	double x = p1->x, y = p1->y, dx = p2->x - x, dy = p2->y - y;
	double d = hypot(dx, dy);
	Line fold = {x, y, dx/d, dy/d};
	return add_line(&next, &fold) && dfs_fold(&next, depth, callback_fn, cb_data);
}

int fold_HH2(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//1 solution
	Origami next = *base;
	next.steps[next.steps_len++];
	const Point *restrict p1 = next.points[step->i1];
	const Point *restrict p2 = next.points[step->i2];
	double x = (p1->x + p2->x)/2, y = (p1->y + p2->y)/2, dx = p2->x - p1->x, dy = p2->y - p1->y;
	double d = hypot(dx, dy);
	Line fold = {x, y, -dy/d, dx/d};
	return add_line(&next, &fold) && dfs_fold(&next, depth, callback_fn, cb_data);
}

int fold_HH3(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//1 or 2 solutions
}

int fold_HH4(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//1 solution
}

int fold_HH5(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//0 - 2 solutions
}

int fold_HH6(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//0 - 3 solutions
}

int fold_HH7(const Origami *base, const OrigamiStep *step, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	//0 - 1 solutions
}

int dfs_fold(const Origami *base, int depth, int (*callback_fn)(const Origami *base, void *data), void *cb_data){
	return 0;
}

