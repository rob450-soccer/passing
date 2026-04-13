# --- schema.py ---
# The only thing shared between collection and testing.

from dataclasses import dataclass, field
import json, os, datetime

@dataclass
class TrialData:
    run_id:          str
    trial_number:    int
    seed:            int
    timestamp:       str

    start_position:  tuple[float, float]
    ball_position:   tuple[float, float]
    obstacle_config: list[tuple[float, float]]

    # Raw logs — parsed at test time
    log_lines: list[str] = field(default_factory=list)

    # Per-timestep measurements
    velocities:          list[float] = field(default_factory=list)
    com_heights:         list[float] = field(default_factory=list)
    com_z_velocities:    list[float] = field(default_factory=list)
    com_x_velocities:    list[float] = field(default_factory=list)
    joint_angles:        list[dict]  = field(default_factory=list)
    joint_torques:       list[dict]  = field(default_factory=list)
    collision_times:     list[float] = field(default_factory=list)
    latencies_ms:        list[float] = field(default_factory=list)
    out_of_bounds_steps: int         = 0
    total_steps:         int         = 0

    # End-of-trial
    ball_final_pos:        tuple | None = None
    ball_target_pos:       tuple | None = None
    time_to_score_seconds: float | None = None
    messages_sent:         int          = 0
    messages_received:     int          = 0
    player_crashed:        bool         = False
    timed_out:             bool         = False

    def save(self, data_dir="data"):
        os.makedirs(data_dir, exist_ok=True)
        path = os.path.join(data_dir, f"run_{self.run_id}_trial_{self.trial_number:03d}.json")
        with open(path, "w") as f:
            json.dump(self.__dict__, f, indent=2)

    @classmethod
    def load_run(cls, run_id: str, data_dir="data") -> list["TrialData"]:
        trials = []
        for fname in sorted(os.listdir(data_dir)):
            if fname.startswith(f"run_{run_id}_") and fname.endswith(".json"):
                with open(os.path.join(data_dir, fname)) as f:
                    trials.append(cls(**json.load(f)))
        assert trials, f"No data for run '{run_id}'. Run: python collect.py {run_id}"
        return trials