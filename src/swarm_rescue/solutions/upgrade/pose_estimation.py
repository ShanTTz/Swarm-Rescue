"""Short sensor-only dead reckoning when GPS or compass is unavailable."""
from __future__ import annotations
import math
import numpy as np


class PoseEstimate:
    def __init__(self):
        self.step = -1
        self.position = None
        self.heading = None
        self.position_sigma = float("inf")
        self.heading_sigma = float("inf")
        self.velocity = None

    def update(self, step, gps, compass, odometer):
        if step == self.step:
            return
        delta = max(1, step-self.step)
        old_heading = self.heading
        self.step = step
        self.velocity = None
        if odometer is not None and len(odometer) >= 3 and all(math.isfinite(float(v)) for v in odometer[:3]):
            distance, direction, turn = map(float, odometer[:3])
            if old_heading is not None and delta == 1:
                travel = np.array([distance*math.cos(old_heading+direction), distance*math.sin(old_heading+direction)])
                self.velocity = travel
                if self.position is not None:
                    self.position = self.position+travel
                self.heading = math.atan2(math.sin(old_heading+turn), math.cos(old_heading+turn))
                # Odometer direction noise is about 8 degrees; distance noise
                # alone would substantially understate accumulated drift.
                self.position_sigma += .25+abs(distance)*.15
                self.heading_sigma += .02+abs(turn)*.02
            else:
                self.position_sigma += 5*delta
                self.heading_sigma += .1*delta
        else:
            self.position_sigma += 5*delta
            self.heading_sigma += .1*delta
        if gps is not None and len(gps) >= 2 and all(math.isfinite(float(v)) for v in gps[:2]):
            self.position = np.asarray(gps[:2], dtype=float)
            self.position_sigma = 5.0
        if compass is not None and math.isfinite(float(compass)):
            self.heading = float(compass)
            self.heading_sigma = .04

    @property
    def can_map(self):
        return self.position is not None and self.heading is not None and self.position_sigma <= 9 and self.heading_sigma <= .12

    def measured_position(self):
        return self.position if self.position_sigma <= 18 and self.heading_sigma <= .4 else None

    def measured_heading(self):
        return self.heading if self.heading_sigma <= .4 else None
