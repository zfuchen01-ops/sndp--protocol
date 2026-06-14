#!/usr/bin/env python3
"""
Discrete-Event Simulation Engine
=================================
轻量级离散事件仿真器。事件按时间戳排序，顺序执行。

用法:
  sim = EventSim()
  sim.schedule(10.0, my_callback, arg1, arg2)
  sim.schedule_recurring(5.0, periodic_task)
  sim.run(until=3600.0)
"""

import heapq
import time as _time
from typing import Callable, Any, List, Tuple


class Event:
    """一个仿真事件"""
    __slots__ = ('time', 'priority', 'callback', 'args')

    def __init__(self, time: float, callback: Callable, args: tuple = (),
                 priority: int = 0):
        self.time = time
        self.priority = priority  # 同时间戳时，priority 小的先执行
        self.callback = callback
        self.args = args

    def __lt__(self, other: 'Event') -> bool:
        if self.time != other.time:
            return self.time < other.time
        return self.priority < other.priority


class RecurringEvent:
    """周期性事件配置"""
    def __init__(self, interval: float, callback: Callable, args: tuple = (),
                 offset: float = 0.0):
        self.interval = interval
        self.callback = callback
        self.args = args
        self.offset = offset  # 首次触发的偏移


class EventSim:
    """离散事件仿真引擎"""

    def __init__(self):
        self._queue: List[Event] = []
        self._now: float = 0.0
        self._recurring: List[RecurringEvent] = []
        self._running = False
        self._event_count: int = 0

    @property
    def now(self) -> float:
        return self._now

    def schedule(self, time: float, callback: Callable, *args,
                 priority: int = 0):
        """在 time 时刻调度一个事件"""
        ev = Event(time, callback, args, priority)
        heapq.heappush(self._queue, ev)
        self._event_count += 1

    def schedule_recurring(self, interval: float, callback: Callable,
                           *args, offset: float = 0.0):
        """调度周期性事件。首次触发在 offset，之后每 interval 触发一次。"""
        rec = RecurringEvent(interval, callback, args, offset)
        self._recurring.append(rec)
        # 首次触发
        self.schedule(self._now + offset, self._recurring_wrapper, rec)

    def _recurring_wrapper(self, rec: RecurringEvent):
        """周期性事件包装器：执行回调，然后重新调度"""
        rec.callback(*rec.args)
        self.schedule(self._now + rec.interval, self._recurring_wrapper, rec)

    def run(self, until: float, realtime_factor: float = 0.0):
        """
        运行仿真直到 until 时刻。

        realtime_factor: 如果 > 0，限制实时速度 (如 1.0 = 实时)
        """
        self._running = True
        last_real = _time.time()

        while self._running and self._queue:
            ev = heapq.heappop(self._queue)
            if ev.time > until:
                heapq.heappush(self._queue, ev)  # 放回去
                break

            self._now = ev.time

            # 实时速率限制
            if realtime_factor > 0:
                expected = (ev.time - (self._now if self._now == ev.time else 0)) / realtime_factor
                elapsed = _time.time() - last_real
                if expected > elapsed:
                    _time.sleep(expected - elapsed)
                last_real = _time.time()

            try:
                ev.callback(*ev.args)
            except Exception as e:
                print(f"  [EVENT ERROR @ t={self._now:.1f}] {e}")

        self._now = until

    def stop(self):
        self._running = False

    @property
    def queue_size(self) -> int:
        return len(self._queue)
