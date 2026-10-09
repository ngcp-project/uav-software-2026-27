"""Code frame for an unknown-location lawnmower search pattern.

This module intentionally contains declarations only.  Search-area discovery,
coordinate conversion, path generation, vehicle guidance, and detection
handling are left for a later implementation.


A complete pipeline for this search pattern include the following:

 1. Aquire search area (defined by GCS)
 2. Fly the plane autonomously in lawnmower pattern
 3. Report location, altitude, heading, and detections to GCS while in pattern
 4. Detect RF signals and report detections to GCS while in pattern
 5. Once location is partially determined, the pattern of flight is autonomously adjusted to partially known
    location search pattern (Expanding Square)
NOTE: the transition between flight patterns is not controlled by GCS.

"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class SearchPhase(str, Enum):
    """Lifecycle phases for an unknown-location search mission."""

    IDLE = "idle"
    LOCATING = "locating"
    AREA_DEFINED = "area_defined"
    PATTERN_READY = "pattern_ready"
    EXECUTING = "executing"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class GeoPoint:
    """WGS84 geographic position."""

    latitude_deg: float
    longitude_deg: float


@dataclass(frozen=True)
class LocalPoint:
    """Point in a search-local Cartesian frame, in metres."""

    east_m: float
    north_m: float


@dataclass(frozen=True)
class SearchArea:
    """Search boundary and coordinate-frame metadata."""

    boundary: Tuple[GeoPoint, ...] = field(default_factory=tuple)
    origin: Optional[GeoPoint] = None


@dataclass(frozen=True)
class VehicleState:
    """Vehicle state sent to GCS while executing a lawnmower search pattern."""

    position: Optional[GeoPoint] = None
    altitude_m: Optional[float] = None
    heading_deg: Optional[float] = None
    armed: bool = False
    mode: Optional[str] = None


@dataclass(frozen=True)
class SearchDetection:
    """RF Detection report produced while searching."""

    position: Optional[GeoPoint] = None
    confidence: Optional[float] = None
    timestamp_s: Optional[float] = None
    payload: object = None


@dataclass(frozen=True)
class LawnmowerConfig:
    """Configuration inputs for a lawnmower search pattern."""

    lane_spacing_m: Optional[float] = None
    search_altitude_m: Optional[float] = None
    boundary_clearance_m: Optional[float] = None
    waypoint_radius_m: Optional[float] = None
    max_search_duration_s: Optional[float] = None


@dataclass
class LawnmowerSearchPlan:
    """Combines search area, configuration, and waypoints for a lawnmower search pattern."""

    phase: SearchPhase = SearchPhase.IDLE
    search_area: Optional[SearchArea] = None
    config: LawnmowerConfig = field(default_factory=LawnmowerConfig)
    waypoints: list[GeoPoint] = field(default_factory=list)
    detections: list[SearchDetection] = field(default_factory=list)


class LawnmowerSearchFrame:
    """Interface boundary for an unknown-location lawnmower search."""

    def __init__(self, config: Optional[LawnmowerConfig] = None) -> None:
        pass

    def acquire_search_area(self, vehicle: VehicleState) -> Optional[SearchArea]:
        """Receive the search boundary from GCS."""
        pass

    def define_search_area(self, area: SearchArea) -> None:
        """Register a discovered or externally supplied search boundary."""
        pass

    def build_plan(self, area: SearchArea) -> LawnmowerSearchPlan:
        """Create a lawnmower plan from a search boundary."""
        pass

    def next_waypoint(self, vehicle: VehicleState) -> Optional[GeoPoint]:
        """Return the next waypoint for the future guidance layer."""
        pass

    def record_detection(self, detection: SearchDetection) -> None:
        """Accept a detection from the RF layer."""
        pass

    def update(self, vehicle: VehicleState) -> SearchPhase:
        """Advance the future search lifecycle from current vehicle state."""
        pass

    def abort(self, reason: Optional[str] = None) -> None:
        """Terminate the future search operation."""
        #TODO: implement abort logic
        # HOWEVER, should this method turn the MRA into loiter mode or transition 
        # into another flight pattern once the location is partially determined, or both?
        pass


__all__ = [
    "GeoPoint",
    "LawnmowerConfig",
    "LawnmowerSearchFrame",
    "LawnmowerSearchPlan",
    "LocalPoint",
    "SearchArea",
    "SearchDetection",
    "SearchPhase",
    "VehicleState",
]
