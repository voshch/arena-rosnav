#!/usr/bin/env python3
"""
Simple SocNav Trajectory Loader - Minimal CSV reader
"""
import os
import csv
from typing import Dict, List, Tuple, Optional


class SimpleTrajectoryLoader:
    """Minimal loader for SocNav CSV files"""
    
    def __init__(self):
        self.data: Dict[int, Dict[int, Tuple[float, float]]] = {}  # {frame_id: {ped_id: (x, y)}}
        self.episode_name = ""
        self.total_frames = 0
        self.total_pedestrians = 0
        
    def load_csv(self, episode_name: str) -> bool:
        """Load CSV file and parse to simple format"""
        
        csv_file = os.path.join(
            os.path.dirname(__file__), 
            "trajectory_data", 
            f"{episode_name}.csv"
        )
        
        if not os.path.exists(csv_file):
            print(f"Error: {csv_file} not found")
            return False
            
        print(f"Loading {episode_name}...")
        
        try:
            # Load and transpose CSV (like SocNavBench does)
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                raw_data = list(reader)
            
            # Transpose data
            data_transposed = list(zip(*raw_data))
            
            # Parse: frame, ped_id, y, x (SocNavBench format)
            self.data = {}
            pedestrian_ids = set()
            
            for row in data_transposed:
                frame = int(row[0])
                ped_id = int(row[1]) 
                x = float(row[2])
                y = float(row[3])
                
                if frame not in self.data:
                    self.data[frame] = {}
                self.data[frame][ped_id] = (x, y)
                pedestrian_ids.add(ped_id)
                
            self.episode_name = episode_name
            self.total_frames = len(self.data)
            self.total_pedestrians = len(pedestrian_ids)
            
            print(f"Loaded {self.total_frames} frames with {self.total_pedestrians} pedestrians")
            return True
            
        except Exception as e:
            print(f"Error loading {episode_name}: {e}")
            return False
        
    def get_pedestrians_at_frame(self, frame_id: int) -> Dict[int, Tuple[float, float]]:
        """Get all pedestrians at specific frame"""
        return self.data.get(frame_id, {})
        
    def get_frame_range(self) -> Tuple[int, int]:
        """Get min/max frame IDs"""
        if not self.data:
            return (0, 0)
        frames = list(self.data.keys())
        return (min(frames), max(frames))
        
    def get_available_frames(self) -> List[int]:
        """Get sorted list of all available frame IDs"""
        return sorted(self.data.keys())
        
    def get_pedestrian_trajectory(self, ped_id: int) -> List[Tuple[int, float, float]]:
        """Get complete trajectory for one pedestrian: [(frame, x, y), ...]"""
        trajectory = []
        for frame_id in sorted(self.data.keys()):
            if ped_id in self.data[frame_id]:
                x, y = self.data[frame_id][ped_id]
                trajectory.append((frame_id, x, y))
        return trajectory
        
    def get_stats(self) -> Dict[str, any]:
        """Get dataset statistics"""
        if not self.data:
            return {}
            
        frame_range = self.get_frame_range()
        return {
            "episode": self.episode_name,
            "total_frames": self.total_frames,
            "total_pedestrians": self.total_pedestrians,
            "frame_range": frame_range,
            "duration_seconds": (frame_range[1] - frame_range[0]) * 0.04,  # 25 fps
        }


def test_loader():
    """Test function to verify the loader works"""
    loader = SimpleTrajectoryLoader()
    
    # Test with hotel dataset
    if loader.load_csv("hotel"):
        stats = loader.get_stats()
        print(f"Dataset stats: {stats}")
        
        # Show first few frames
        frames = loader.get_available_frames()[:5]
        for frame in frames:
            peds = loader.get_pedestrians_at_frame(frame)
            print(f"Frame {frame}: {len(peds)} pedestrians")
            
        # Show trajectory for pedestrian 1
        traj = loader.get_pedestrian_trajectory(1)[:5]
        print(f"Pedestrian 1 trajectory (first 5): {traj}")


if __name__ == "__main__":
    test_loader()