#!/usr/bin/env python3
import operator, functools

def constructions_ub(n):
	return functools.reduce(operator.mul, (((((3*k + 54)*k + 300)*k + 511)*k + 260)//2 for k in range(n)))

print(constructions_ub(4))

