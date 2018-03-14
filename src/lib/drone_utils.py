#!/usr/bin/env python
from scipy import signal
from math import pi
import math
import numpy as np
import logging

class L(list):
    def __init__(self,maxlen):
        super(L,self).__init__()
        self.maxlen = maxlen

    def __repr__(self):
        return '%s:%s' % (self.__class__.__name__,list.__repr__(self),)

    def append(self, item):
         list.append(self, item)
         if len(self) > self.maxlen:
             self[:-self.maxlen]= []

def degtorad(x):
    return x*pi/180

def deriv(x,xp,h):
    return (x-xp)/h

def filter_avg(x,x_p):
    """
    Apply a simple average lowpass filter to a signal
    """
    return 0.5*(x+x_p)

def avg_error(arrs):
    """ Function that computes the average error between a group of arrays
        arrs can be a tuple or a list"""
    set1, set2 = arrs[0]
    err_total = np.sqrt(np.power((set1-set2),2))
    return np.sum(err_total)/len(err_total)

def filter_FIR(cutoff, x, new_val):
    """
    Apply a lowpass FIR filter to a signal

    The x list must be in the next order:
        x = [val[n],val[n-1],val[n-2]...,val[n-N]]
    """

    for i in reversed(range(len(x))):
        if i == 0:
            x[i] = new_val
        else:
            x[i] = x[i - 1]

    order = len(x)
    coefs = signal.firwin(order, cutoff, window='hamming')
    filtered_signal = 0
    for i in range(order):
        filtered_signal += coefs[i]*x[i]
    return filtered_signal

def RK4(f,x,v,tp,h):
    k1 = h*v
    l1 = h*f(x,v,tp)
    k2 = h*(v+(l1/2.0))
    l2 = h*f(x+(k1/2.0),v+(l1/2.0),tp+(h/2.0))
    k3 = h*(v+(l2/2.0))
    l3 = h*f(x+(k2/2.0),v+(l2/2.0),tp+(h/2.0))
    k4 = h*(v+l3)
    l4 = h*f(x+k3,v+l3,tp+h)
    x = x+(1.0/6.0)*(k1+(2.0*k2)+(2.0*k3)+k4)
    v = v+(1.0/6.0)*(l1+(2.0*l2)+(2.0*l3)+l4)
    return x,v

def RK4_2(f,x,v,tp,h):
    k1 = h*f(x,v,tp)
    k2 = h*f(x,v+(k1/2.0),tp+(h/2.0))
    k3 = h*f(x,v+(k2/2.0),tp+(h/2.0))
    k4 = h*f(x,v+k3,tp+h)
    v = v+(1.0/6.0)*(k1+(2.0*k2)+(2.0*k3)+k4)
    return v