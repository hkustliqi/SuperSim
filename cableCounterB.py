#!/usr/bin/env python


import argparse
import os
import math

def calDistance (x, y, z, i, j, k):
  return math.fabs(x-i) + math.fabs(y-j) + z + k

def main(args):
  row = 10
  column = 20
  chassis = 8
  width = 0.663575
  depth = 1.778
  height = 0.22225
  total = 0

  for i in range(0, row):
    for j in range (0, column):
      for k in range (0, chassis):
        for y in range (0, column):
          for z in range (k/2 * 2, k/2 * 2 +2):
            total += calDistance (i*depth, j*width, k*height, i*depth, y*width, z*height)
        for x in range (0, row):
          for z in range (k/2 * 2, k/2 * 2 +2):
	    total += calDistance (i*depth, j*width, k*height, x*depth, j*width, z*height)
  total /= 2
  print("total distance is " + str(total))

if __name__ == '__main__':
  ap = argparse.ArgumentParser()

  args = ap.parse_args()
  main(args)

