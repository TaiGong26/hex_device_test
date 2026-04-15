import time
import numpy as np

from enum import Enum

class TrajectoryStatus(Enum):
    IDLE            = 0
    Trajectory      = 1
    Return_Home     = 2 
    Finished        = 3

class TrajectoryPlanner:
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, segment_duration=3.0):
        """
        Initialize trajectory planner
        waypoints: List of waypoints
        segment_duration: Duration of each trajectory segment (seconds)
        """
        self.waypoints = waypoints
        self.segment_duration = segment_duration
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        self.last_target_position = None  # Store last commanded position
        
    def start_trajectory(self):
        """Start trajectory execution"""
        if not self.waypoints:
            return False
        
        self.trajectory_started = True
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        
    def get_current_target(self):
        """Get the target position at the current moment"""
        if not self.trajectory_started or not self.waypoints:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        total_segments = len(self.waypoints)
        segment_index = int(elapsed_time / self.segment_duration) % total_segments
        
        segment_elapsed = elapsed_time % self.segment_duration
        normalized_time = segment_elapsed / self.segment_duration
        
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % total_segments]
        
        # Use S-curve interpolation to calculate current position
        s = self._smooth_step(normalized_time)
        
        start_pos = np.array(start_waypoint)
        end_pos = np.array(end_waypoint)
        target_position = start_pos + s * (end_pos - start_pos)
        
        self.current_waypoint_index = segment_index
        self.last_target_position = target_position  # Store for potential return home
        
        return target_position
    
    def get_last_position(self):
        """Get the last commanded position"""
        return self.last_target_position
        
    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
        
    def get_current_segment_info(self):
        """Get information about the current segment"""
        if not self.trajectory_started:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        segment_index = int(elapsed_time / self.segment_duration) % len(self.waypoints)
        segment_elapsed = elapsed_time % self.segment_duration
        segment_progress = segment_elapsed / self.segment_duration
        
        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'total_elapsed': elapsed_time
        }

class ReturnHomeController:
    """Controller for smooth return to home position"""
    
    def __init__(self, start_position, home_position, duration):
        """
        Initialize return home controller
        start_position: Starting position (current position when Ctrl+C is pressed)
        home_position: Target home position
        duration: Duration to reach home position (seconds)
        """
        self.start_position = np.array(start_position)
        self.home_position = np.array(home_position)
        self.duration = duration
        self.start_time = time.time()
        self.done = False
        
    def get_target_position(self):
        """Get the current target position during return home"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if np.allclose(self.start_position,self.home_position):
            self.done = True
        
        if elapsed_time >= self.duration or self.done:
            return self.home_position, True  # Reached home
        
        # Calculate normalized time [0, 1]
        t = elapsed_time / self.duration
        
        # Use S-curve interpolation for smooth motion
        s = self._smooth_step(t)
        
        # Interpolate between start and home position
        target_position = self.start_position + s * (self.home_position - self.start_position)
        
        return target_position, False  # Not yet reached home
    
    def _smooth_step(self, t):
        """S-curve interpolation function"""
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3
