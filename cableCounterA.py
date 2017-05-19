#!/usr/bin/env python


import argparse
import os
import math

def calDistance (x, y, i, j):
  return math.fabs(x-i) + math.fabs(y-j)

def main(args):
  row = 9
  column = 29
  width = 0.4826
  depth = 1.778
  total = 0
  for i in range(0, row):
    for j in range (0, column):
      for x in range (0, row):
        for y in range (0, column):
          total += calDistance (x*depth, y*width, i*depth, j*width)
  total /= 2
  print("total distance is " + str(total))

if __name__ == '__main__':
  ap = argparse.ArgumentParser()

  args = ap.parse_args()
  main(args)

