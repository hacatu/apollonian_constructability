#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "construction.h"
#include "init_geometry.h"

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

SDL_Window *window;
SDL_Renderer *renderer;

void quit(void){
	if(renderer){
		SDL_DestroyRenderer(renderer);
	}
	if(window){
		SDL_DestroyWindow(window);
	}
	SDL_Quit();
}

void init(void){
	if(SDL_Init(SDL_INIT_VIDEO)){
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	atexit(quit);
	if(!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1")){
		printf("Warning: Linear texture filtering not enabled!");
	}
	window = SDL_CreateWindow("Construction Visualization", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
	if(!window){
		printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if(renderer == NULL){
		printf("Renderer could not be created! SDL Error: %s\n", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
}

typedef struct{
	double l, r, t, b;
} Bounds;

double getBounds(const ApproxCons *self, Bounds *bounds){
	bounds->l = bounds->r = self->points[0].x;
	bounds->t = bounds->b = self->points[0].y;
	for(int i = 1; i < self->points_len; ++i){
		double x = self->points[i].x;
		double y = self->points[i].y;
		if(x < bounds->l){
			bounds->l = x;
		}else if(x > bounds->r){
			bounds->r = x;
		}
		if(y < bounds->b){
			bounds->b = y;
		}else if(y > bounds->t){
			bounds->t = y;
		}
	}
	for(int i = 0; i < self->circles_len; ++i){
		double x = self->circles[i].x;
		double y = self->circles[i].y;
		double r = self->circles[i].r;
		if(x - r < bounds->l){
			bounds->l = x - r;
		}
		if(x + r > bounds->r){
			bounds->r = x + r;
		}
		if(y - r < bounds->b){
			bounds->b = y - r;
		}
		if(y + r > bounds->t){
			bounds->t = y + r;
		}
	}
	double h_scale = SCREEN_WIDTH/(bounds->r - bounds->l);
	double v_scale = SCREEN_HEIGHT/(bounds->t - bounds->b);
	if(h_scale < v_scale){
		bounds->t = bounds->b + SCREEN_HEIGHT/h_scale;
		return h_scale;
	}
	bounds->r = bounds->l + SCREEN_WIDTH/v_scale;
	return v_scale;
}

void drawLine(const Line *line, const Bounds *bounds, double scale_factor, Uint8 r, Uint8 g, Uint8 b, Uint8 a){
	if(!line->dx){
		int x = scale_factor*(line->x - bounds->l);
		aalineRGBA(renderer, x, 0, x, SCREEN_HEIGHT, r, g, b, a);
	}else if(!line->dy){
		int y = scale_factor*(bounds->t - line->y);
		aalineRGBA(renderer, 0, y, SCREEN_WIDTH, y, r, g, b, a);
	}else{
		double x = line->x, y = line->y;
		double dx = line->dx, dy = line->dy;
		if(dx < 0){
			dx = -dx;
			dy = -dy;
		}
		double l_time = (line->x - bounds->l)/dx;
		double r_time = (bounds->r - line->x)/dx;
		double t_time = (bounds->t - line->y)/fabs(dy);
		double b_time = (line->y - bounds->b)/fabs(dy);
		int x1, y1, x2, y2;
		if(dy > 0){
			if(t_time > r_time){
				x1 = SCREEN_WIDTH;
				y1 = scale_factor*(bounds->t - y - r_time*dy);
			}else{
				x1 = scale_factor*(x + t_time*dx - bounds->l);
				y1 = 0;
			}
			if(b_time > l_time){
				x2 = 0;
				y2 = scale_factor*(bounds->t - y + l_time*dy);
			}else{
				x2= scale_factor*(x - b_time*dx - bounds->l);
				y2 = SCREEN_HEIGHT;
			}
		}else{
			if(b_time > r_time){
				x1 = SCREEN_WIDTH;
				y1 = scale_factor*(bounds->t - y - r_time*dy);
			}else{
				x1 = scale_factor*(x + b_time*dx - bounds->l);
				y1 = SCREEN_HEIGHT;
			}
			if(t_time > l_time){
				x2 = 0;
				y2 = scale_factor*(bounds->t - y + l_time*dy);
			}else{
				x2 = scale_factor*(x - t_time*dx - bounds->l);
				y2 = 0;
			}
		}
		aalineRGBA(renderer, x1, y1, x2, y2, r, g, b, a);
	}
}

void drawCircle(const Circle *circle, const Bounds *bounds, double scale_factor, Uint8 r, Uint8 g, Uint8 b, Uint8 a){
	int x = scale_factor*(circle->x - bounds->l);
	int y = scale_factor*(bounds->t - circle->y);
	aacircleRGBA(renderer, x, y, scale_factor*circle->r, r, g, b, a);
}

void drawPoint(const Point *point, const Bounds *bounds, double scale_factor, Uint8 r, Uint8 g, Uint8 b, Uint8 a){
	int x = scale_factor*(point->x - bounds->l);
	int y = scale_factor*(bounds->t - point->y);
	pixelRGBA(renderer, x, y, r, g, b, a);
}

void drawGeometry(const ApproxCons *base, const Bounds *bounds, double scale_factor){
	for(int i = 0; i < base->lines_len; ++i){
		drawLine(base->lines + i, bounds, scale_factor, 255, 255, 0, 255);
	}
	for(int i = 0; i < base->circles_len; ++i){
		drawCircle(base->circles + i, bounds, scale_factor, 255, 255, 255, 255);
	}
	for(int i = 0; i < base->points_len; ++i){
		drawPoint(base->points + i, bounds, scale_factor, 255, 0, 255, 255);
	}
	drawCircle(&base->goal, bounds, scale_factor, 255, 0, 255, 255);
}

int main(void){
	init();
	size_t len;
	int fd = open("scalene.dat", O_RDONLY);
	if(fd == -1){
		printf("Could not open file \"%s\"\n", "scalene.dat");
		exit(1);
	}
	XCons *candidates = read_file(fd, &len);
	close(fd);
	if(!candidates){
		printf("Could not read file \"%s\"\n", "scalene.dat");
		exit(1);
	}
	printf("Loaded %zu candidates\n", len);
	ApproxCons base = {};
	init_geometry_345(&base);
	//init_geometry_scalene_lt120(&base, 40, 13, 37);
	if(!apply_xcons(&base, candidates)){
		printf("Could not apply construction.\n");
		base = (ApproxCons){};
	}
	Bounds bounds;
	getBounds(&base, &bounds);
	double scale_factor = SCREEN_HEIGHT/(bounds.t - bounds.b) > SCREEN_WIDTH/(bounds.r - bounds.l) ?
		SCREEN_WIDTH/(bounds.r - bounds.l) : SCREEN_HEIGHT/(bounds.t - bounds.b);
	drawGeometry(&base, &bounds, scale_factor);
	SDL_RenderPresent(renderer);
	int i = 0;
	int curr_steps = candidates->len;
	SDL_Event e;
	while(1){
		while(SDL_PollEvent(&e)){
			if(e.type == SDL_QUIT){
				exit(0);
			}else if(e.type == SDL_KEYDOWN){
				switch(e.key.keysym.sym){
				case SDLK_LEFT:
					candidates[i].len = curr_steps;
					i = (i + len - 1)%len;
					break;
				case SDLK_RIGHT:
					candidates[i].len = curr_steps;
					i = (i + 1)%len;
					break;
				case SDLK_DOWN:
					for(int j = candidates[i].len; j < curr_steps; ++j){
						switch(candidates[i].construction[j].type){
						case GEOM_DEP_PPL:
						case GEOM_DEP_PPC:
							candidates[i].len = j + 1;
							j = curr_steps;
						default:
							break;
						}
					}
					break;
				case SDLK_UP:
				{
					int j = candidates[i].len;
					if(j){
						for(--j; j-- > 0;){
							switch(candidates[i].construction[j].type){
							case GEOM_DEP_PPL:
							case GEOM_DEP_PPC:
								candidates[i].len = j + 1;
								j = -1;
							default:
								break;
							}
						}
						if(j == -1){
							candidates[i].len = 0;
						}
					}
				}
				}
				base = (ApproxCons){};
				init_geometry_345(&base);
				//init_geometry_scalene_lt120(&base, 40, 13, 37);
				if(!apply_xcons(&base, candidates + i)){
					printf("Could not apply construction.\n");
					base = (ApproxCons){};
				}
				SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
				SDL_RenderFillRect(renderer, &(SDL_Rect){0, 0, SCREEN_WIDTH, SCREEN_HEIGHT});
				drawGeometry(&base, &bounds, scale_factor);
				SDL_RenderPresent(renderer);
			}
		}
	}
}

