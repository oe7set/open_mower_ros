"""Python twin of open_mower::events::EventPublisher.

Producers (currently mower_scheduler) call ``init(source)`` once at node
startup, then ``emit(severity, type, summary, details=None)`` at each
lifecycle transition. Events are published on the global ``/events`` ROS
topic; ``xbot_monitoring`` is the single subscriber that persists them and
mirrors them onto MQTT.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from typing import Any, Optional

import rospy
from xbot_msgs.msg import Event

SEVERITY_INFO = Event.SEVERITY_INFO
SEVERITY_WARNING = Event.SEVERITY_WARNING
SEVERITY_ERROR = Event.SEVERITY_ERROR
SEVERITY_CRITICAL = Event.SEVERITY_CRITICAL

_lock = threading.Lock()
_publisher: Optional[rospy.Publisher] = None
_source: str = ""
_warned_uninitialized = False


def init(source: str) -> None:
    """Initialise the singleton publisher. Call once after ``rospy.init_node``."""
    global _publisher, _source
    with _lock:
        _publisher = rospy.Publisher("/events", Event, queue_size=20, latch=False)
        _source = source


def emit(severity: int, event_type: str, summary: str, details: Optional[dict[str, Any]] = None) -> None:
    """Publish a single event. Safe to call before init() — silently dropped."""
    global _warned_uninitialized
    with _lock:
        if _publisher is None:
            if not _warned_uninitialized:
                rospy.logwarn("event_publisher.emit called before init(); event '%s' dropped", event_type)
                _warned_uninitialized = True
            return
        publisher = _publisher
        source = _source

    msg = Event()
    msg.header.stamp = rospy.Time.now()
    msg.id = str(uuid.uuid4())
    msg.ts_ms = int(time.time() * 1000)
    msg.severity = int(severity)
    msg.type = event_type
    msg.source = source
    msg.summary = summary
    msg.details_json = json.dumps(details, separators=(",", ":")) if details else ""
    publisher.publish(msg)


def info(event_type: str, summary: str, details: Optional[dict[str, Any]] = None) -> None:
    emit(SEVERITY_INFO, event_type, summary, details)


def warning(event_type: str, summary: str, details: Optional[dict[str, Any]] = None) -> None:
    emit(SEVERITY_WARNING, event_type, summary, details)


def error(event_type: str, summary: str, details: Optional[dict[str, Any]] = None) -> None:
    emit(SEVERITY_ERROR, event_type, summary, details)


def critical(event_type: str, summary: str, details: Optional[dict[str, Any]] = None) -> None:
    emit(SEVERITY_CRITICAL, event_type, summary, details)
