import numpy as np

class Target:

	def __init__(self, pos0, vel0):
		self.tof = 0.0
		self.dt = None
		self.pos = pos0
		self.vel = vel0

		print("TARGET CONSTRUCTED")

	def update(self, DT):
		self.dt = DT
		deltaPos = self.dt * self.vel
		self.pos += deltaPos
		self.tof += self.dt