#!/usr/bin/env python3

import struct, sys
from fractions import Fraction

sys.path.insert(0, "/usr/lib/python3.6/site-packages/")

from sympy import sqrt, Symbol, simplify, sin, cos, acos, exp, I, re, im, Number

GEOM_DEP_PPL = 0
GEOM_DEP_PPC = 1
GEOM_DEP_LLP = 2
GEOM_DEP_CLP = 3
GEOM_DEP_CCP = 4

def hypot(*args):
	return sqrt(sum(x**2 for x in args))

class ExactCons:
	def __init__(self):
		self.points = []
		self.lines = []
		self.circles = []
		self.goal = None
	
	@staticmethod
	def intersect_LL(a, b):
		daxdb = a[2]*b[3] - a[3]*b[2]
		if daxdb == 0:
			return []
		ambx = a[0] - b[0]
		amby = a[1] - b[1]
		t = (amby*b[2] - ambx*b[3])/daxdb;
		return [(a[0] + t*a[2], a[1] + t*a[3])]
	
	@staticmethod
	def intersect_CC(a, b):
		d = hypot(b[0] - a[0], b[1] - a[1])
		if d > a[2] + b[2] or d < abs(b[2] - a[2]):
			return []
		elif a[2] + b[2] == d or abs(b[2] - a[2]) == d:
			if a[2] <= b[2] and d < b[2] + a[2]:
				d = -d
			d = a[2]/d
			return [(a[0] + d*(b[0] - a[0]), a[1] + d*(b[1] - a[1]))]
		al = (a[2]**2 - b[2]**2 + d**2)/(2*d)
		h = sqrt(a[2]**2 - al**2)
		al /= d
		h /= d
		dx = b[1] - a[1]
		dy = b[0] - a[0]
		x = a[0] + al*dy
		y = a[1] + al*dx
		dx *= h
		dy *= h
		return [
			(x + dx, y - dy),
			(x - dx, y + dy)]
	
	@staticmethod
	def intersect_CL(C, L):
		d = (L[0] - C[0])*(-L[3]) + (L[1] - C[1])*L[2]
		ox = d*-L[3]
		oy = d*L[2]
		a = hypot(ox, oy)
		if a > C[2]:
			return []
		elif a == C[2]:
			return [(C[0] + ox, C[1] + oy)]
		h = C[2] if a == 0 else sqrt(C[2]**2 - a**2)/a
		dx = h*oy
		dy = h*ox
		ox += C[0]
		oy += C[1]
		return [
			(ox + dx, oy - dy),
			(ox - dx, oy + dy)]
	
	@staticmethod
	def construct_line(a, b):
		dx = b[0] - a[0]
		dy = b[1] - a[1]
		h = hypot(dx, dy)
		dx /= h
		dy /= h
		return a[0], a[1], simplify(dx), simplify(dy)
	
	@staticmethod
	def construct_circle(a, b):
		return a[0], a[1], simplify(hypot(b[0] - a[0], b[1] - a[1]))
	
	def apply_steps(self, steps):
		for i, (i1, i2, o1, o2, geom_dep_type) in enumerate(steps):
			#print("applying step {}/{}".format(i, len(steps)), ["PPL", "PPC", "LLP", "CLP", "CCP"][geom_dep_type])
			[self.apply_PPL, self.apply_PPC, self.apply_LLP, self.apply_CLP, self.apply_CCP][geom_dep_type](i1, i2, o1, o2)
	
	def apply_PPL(self, i1, i2, o1, o2):
		l = self.construct_line(self.points[i1], self.points[i2])
		if len(self.lines) > o1:
			self.lines[o1] = l
		else:
			self.lines += [None]*(o1 - len(self.lines)) + [l]
	
	def apply_PPC(self, i1, i2, o1, o2):
		c = self.construct_circle(self.points[i1], self.points[i2])
		if len(self.circles) > o1:
			self.circles[o1] = c
		else:
			self.circles += [None]*(o1 - len(self.circles)) + [c]
	
	def apply_LLP(self, i1, i2, o1, o2):
		ps = [simplify(p) for p in self.intersect_LL(self.lines[i1], self.lines[i2])]
		if not ps:
			raise ValueError("Lines don't intersect")
		if len(self.points) > o1:
			self.points[o1] = ps[0]
		else:
			self.points += [None]*(o1 - len(self.points)) + ps
	
	def apply_CLP(self, i1, i2, o1, o2):
		ps = [simplify(p) for p in self.intersect_CL(self.circles[i1], self.lines[i2])]
		if len(ps) == 0 or (len(ps) == 1 and o2 != -1):
			raise ValueError("Circle and line don't have enough intersections")
		if max(o1, o2) >= len(self.points):
			self.points += [None]*(max(o1, o2) - len(self.points) + 1)
		if o1 != -1:
			self.points[o1] = simplify(ps[0])
		if o2 != -1:
			self.points[o2] = simplify(ps[1])
	
	def apply_CCP(self, i1, i2, o1, o2):
		ps = [simplify(p) for p in self.intersect_CC(self.circles[i1], self.circles[i2])]
		if len(ps) == 0 or (len(ps) == 1 and o2 != -1):
			raise ValueError("Circles don't have enough intersections")
		if max(o1, o2) >= len(self.points):
			self.points += [None]*(max(o1, o2) - len(self.points) + 1)
		if o1 != -1:
			self.points[o1] = ps[0]
		if o2 != -1:
			self.points[o2] = ps[1]

def init_isosceles(base, a, b):
	base.points = [
		(-b*a/(a + b), sqrt((a + b)**2 - b**2)*(1 + a/(a + b))),
		(b*a/(a + b), sqrt((a + b)**2 - b**2)*(1 + a/(a + b))),
		(0, sqrt((a + b)**2 - b**2)), (0, 0),
		(-b*a/(a + b), sqrt((a + b)**2 - b**2)*(1 - a/(a + b))),
		(b*a/(a + b), sqrt((a + b)**2 - b**2)*(1 - a/(a + b))),
		(-b, 0), (b, 0),
		(-b*(1 + a/(a + b)), -sqrt((a + b)**2 - b**2)*a/(a + b)),
		(b*(1 + a/(a + b)), -sqrt((a + b)**2 - b**2)*a/(a + b))]
	base.lines = [
		(0, sqrt((a + b)**2 - b**2), sqrt((a + b)**2 - b**2)/(a + b), b/(a + b)),
		(0, sqrt((a + b)**2 - b**2), -sqrt((a + b)**2 - b**2)/(a + b), b/(a + b))]
	base.circles = [
		(0, sqrt((a + b)**2 - b**2), a), (-b, 0, b), (b, 0, b)]
	r = 1/(1/a + 2/b + 2*sqrt(1/b**2 + 2/(a*b)))
	base.goal = 0, sqrt((a + b)**2 - b**2) - a - r, r

def init_scalene_normal(base, a, b, c):
	C = acos((a**2 + b**2 - c**2)/(2*a*b))
	rx = (a - b + c)/2
	ry = (c - a + b)/2
	r0 = abs((c - a - b)/2)
	if a*sin(C) <= rx or b*sin(C) <= ry:
		raise ValueError("The side lengths a, b, and c produce extra points")
	base.points = [
		(cos(C)*(b + ry), sin(C)*(b + ry)),
		(cos(C)*b, sin(C)*b),
		(cos(C)*r0, sin(C)*r0), (0, 0), (-cos(C)*r0, -sin(C)*r0),
		(-r0, 0), (r0, 0), (a, 0), (a + rx, 0),
		(a - (a + r0)*rx/c, sin(C)*b*rx/c)]
	base.lines = [(0, 0, cos(C), sin(C)), (0, 0, 1, 0)]
	base.circles = [(0, 0, r0), (a, 0, rx), (cos(C)*b, sin(C)*b, ry)]
	for i, (x, y) in enumerate(base.points):
		base.points[i] = simplify(x), simplify(y)
	for i, (x, y, dx, dy) in enumerate(base.lines):
		base.lines[i] = simplify(x), simplify(y), simplify(dx), simplify(dy)
	for i, (x, y, r) in enumerate(base.circles):
		base.circles[i] = simplify(x), simplify(y), simplify(r)
	r = 1/(1/r0 + 1/rx + 1/ry + 2*sqrt(1/(r0*rx) + 1/(rx*ry) + 1/(ry*r0)))
	z = r*(a/rx + b*exp(I*C)/ry + 2*sqrt(a*b*exp(I*C)/(rx*ry)))
	base.goal = simplify(re(z)), simplify(im(z)), simplify(r)

def read_fd(fd):
	constructions = []
	try:
		while True:
			l, c_i, r_i = struct.unpack("iii", fd.read(12))
			steps = []
			for _ in range(l):
				steps.append(struct.unpack("iiiii", fd.read(20)))
			constructions.append((steps, c_i, r_i))
	except struct.error as e:
		return constructions

if __name__ == "__main__":
	if len(sys.argv) == 1:
		constructions = read_fd(sys.stdin.buffer)
	else:
		with open(sys.argv[1], "rb") as f:
			constructions = read_fd(f)
	for n, (steps, c_i, r_i) in enumerate(constructions[:], 0):
		base = ExactCons()
		#a = Symbol("a", positive=True)
		#b = Symbol("b", positive=True)
		#init_isosceles(base, a, b)
		init_scalene_normal(base, Number(3), Number(4), Number(5))
		#print(base.points)
		#print(base.circles)
		#print(base.goal)
		#break
		try:
			base.apply_steps(steps)
		except TypeError as e:
			print("Couldn't resolve comparison")
			continue
		except ValueError as e:
			print("Wrong number of intersections at some step")
			continue
		x = simplify(base.goal[0] - base.points[c_i][0])
		y = simplify(base.goal[1] - base.points[c_i][1])
		r = simplify(base.goal[2] - hypot(base.points[r_i][0] - base.goal[0], base.points[r_i][1] - base.goal[1]))
		if x != 0 or y != 0 or r != 0:
			print("Construction is not exact")
			continue
		base = ExactCons()
		init_scalene_normal(base, Number(5), Number(12), Number(13))
		try:
			base.apply_steps(steps)
		except TypeError as e:
			print("Couldn't resolve comparison")
			continue
		except ValueError as e:
			print("Wrong number of intersections at some step")
			continue
		x = simplify(base.goal[0] - base.points[c_i][0])
		y = simplify(base.goal[1] - base.points[c_i][1])
		r = simplify(base.goal[2] - hypot(base.points[r_i][0] - base.goal[0], base.points[r_i][1] - base.goal[1]))
		if x != 0 or y != 0 or r != 0:
			print("Construction is not exact")
			continue
		print("Valid construction", n)

