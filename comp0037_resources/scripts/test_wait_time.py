#!/usr/bin/env python
from numpy import array
from numpy.random import exponential

waitLambda=2
samples=10000

samples = []

for i in range(1000):
    visibleTime = 0.5 * waitLambda + exponential(0.5 * waitLambda)
    samples.append(visibleTime)

print sum(samples) / len(samples)
    
    
