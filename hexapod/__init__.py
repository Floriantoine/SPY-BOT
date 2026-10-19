"""SPY-BOT : contrôle d'un hexapode 6 pattes par cinématique inverse."""

from .config import HexapodConfig, LegConfig, default_config
from .leg_ik import LegAngles, UnreachableError, leg_fk, leg_ik
from .body_ik import BodyPose, body_to_leg_frame
from .gait import GaitCommand, TripodGait
from .robot import Hexapod

__all__ = [
    "HexapodConfig",
    "LegConfig",
    "default_config",
    "LegAngles",
    "UnreachableError",
    "leg_fk",
    "leg_ik",
    "BodyPose",
    "body_to_leg_frame",
    "GaitCommand",
    "TripodGait",
    "Hexapod",
]
