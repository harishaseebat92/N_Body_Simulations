import numpy as np
from numpy import array
from numpy.linalg import norm
import math
from IPython.display import HTML

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation


import pandas as pd
import random

from copy import deepcopy

import itertools

class Vector:
  def __init__(self,x=0.,y=0.,z=0.):
    self.x = x
    self.y = y
    self.z = z
  def __repr__(self):
      return f"Vector({self.x}, {self.y}, {self.z})"
  def __str__(self):
      return f"{self.x}i + {self.y}j + {self.z}k"
  def __getitem__(self,index):
    if index == 0:
      return self.x
    elif index == 1:
      return self.y
    elif index == 2:
      return self.z
    else: 
      raise IndexError('out of bounds')
  def __add__(self,other):
    return Vector(self.x+other.x,
                  self.y+other.y,
                  self.z+other.z
                  )
  def __sub__(self,other):
      return Vector(self.x-other.x,
                    self.y-other.y,
                    self.z-other.z
                    )
  def __mul__(self, other):
        if isinstance(other, Vector):  # Vector dot product
            return (
                self.x * other.x
                + self.y * other.y
                + self.z * other.z
            )
        elif isinstance(other, (int, float)):  # Scalar multiplication
            return Vector(
                self.x * other,
                self.y * other,
                self.z * other,
            )
        else:
            raise TypeError("operand must be Vector, int, or float")
  def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Vector(
                self.x / other,
                self.y / other,
                self.z / other,
            )
        else:
            raise TypeError("operand must be int or float")


  def mag(self):
    return np.linalg.norm(self)
  def norm(self):
    mag = self.mag()
    return Vector(self.x/mag,self.y/mag,self.z/mag)

class SolarSystem:
    def __init__(self, size):
        self.size = size
        self.bodies = []

        self.fig, self.ax = plt.subplots(
            1,
            1,
            subplot_kw={"projection": "3d"},
            figsize=(self.size / 50, self.size / 50),
        )
        self.fig.tight_layout()

    def add_body(self, body):
        self.bodies.append(body)

    def update_all(self):
      for body in self.bodies:
        body.move()
        body.draw()
    def draw_all(self):
        self.ax.set_xlim((-self.size / 2, self.size / 2))
        self.ax.set_ylim((-self.size / 2, self.size / 2))
        self.ax.set_zlim((-self.size / 2, self.size / 2))
        plt.pause(0.001)
        self.ax.clear()
    def calc_all_force(self):
        bodies_copy = self.bodies.copy()
        for i in range(len(bodies_copy)):
            for j in range(len(bodies_copy)):
                if i != j:
                    bodies_copy[i].accel(bodies_copy[j])




# class SolarSystem:
# ... 

class SolarSystemBody:
    min_display_size = 10
    display_log_base = 1.3

    def __init__(
        self,
        solar_system,
        mass,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        self.solar_system = solar_system
        self.mass = mass
        self.position = position
        self.velocity = Vector(*velocity)
        self.display_size = max(
            math.log(self.mass, self.display_log_base),
            self.min_display_size,
        )
        self.colour = "black"

        self.solar_system.add_body(self)

    def move(self):
        self.position = (
            self.position[0] + self.velocity[0],
            self.position[1] + self.velocity[1],
            self.position[2] + self.velocity[2],
        )

    def draw(self):
        self.solar_system.ax.plot(
            *self.position,
            marker="o",
            markersize=self.display_size + self.position[0]/30,
            color=self.colour
        )
    def accel(self, other):
        distance = Vector(*other.position) - Vector(*self.position)
        distance_mag = distance.get_magnitude()
        force_mag = self.mass * other.mass / (distance_mag ** 2)
        force = distance.normalize() * force_mag
        reverse = 1
        for body in self, other:
            acceleration = force / body.mass
            body.velocity += acceleration * reverse
            reverse = -1
# simple_solar_system.py
solar_system = SolarSystem(400)

body = SolarSystemBody(solar_system, 100, velocity=(1, 1, 1))

class Sun(Sol)