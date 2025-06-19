from __future__ import division

class Encoder:
    """Tracks ticks for a single wheel encoder and reports the change
    since the last query."""

    def __init__(self):
        # Configure expected raw counter range and wrap thresholds
        self.setRange(-32768, 32767)
        # Start delta accumulation from an initial count
        self.initCount(0)
        # Whether to invert the sign of the reported delta
        self.isReversed = False

    def setRange(self, low, high):
        """Define the raw count limits delivered by the encoder hardware."""
        self.range = high - low + 1  # total distinct count values
        # Thresholds used to detect wrap events (±30 % band)
        self.lowThresh = low + self.range * 30 // 100
        self.highThresh = low + self.range * 70 // 100

    def initCount(self, startCount):
        """Reset the delta accumulator and remember the current count."""

class Encoder:
    """Monitors a single wheel encoder and accumulates delta ticks
    since the last time they were requested.
    """

    def __init__(self):
        self.setRange(-32768, 32767)
        self.initCount(0)
        self.isReversed = False

    def setRange(self, low, high):
        self.range = high - low + 1
        self.lowThresh = low + self.range*30//100
        self.highThresh = low + self.range*70//100

    def initCount(self, startCount):
        self.delta = 0
        self.last = startCount

    def update(self, newCount):
        """Process a fresh raw count from the encoder and accumulate ticks."""
        # Detect wrap‑around and compute signed increment
        if self.last > self.highThresh and newCount < self.lowThresh:
            # Counter rolled over its maximum
            increment = newCount + self.range - self.last
        elif self.last < self.lowThresh and newCount > self.highThresh:
            # Counter rolled under its minimum
        if self.last > self.highThresh and newCount < self.lowThresh:
            # Wrapped around the upper limit
            increment = newCount + self.range - self.last
        elif self.last < self.lowThresh and newCount > self.highThresh:
            # Wrapped around the lower limit
            increment = newCount - self.range - self.last
        else:
            increment = newCount - self.last

        self.delta += increment
        self.last = newCount

    def setReversed(self, isReversed):
        """Invert delta sign if the encoder is mounted in reverse."""
        self.isReversed = isReversed

    def getDelta(self):
        """Return accumulated ticks and reset accumulator."""
        delta = self.delta
        self.delta = 0
        return -delta if self.isReversed else delta

    def getLimits(self):
        """Expose current wrap detection thresholds for diagnostics."""
        return {
            'range': self.range,
            'lowThresh': self.lowThresh,
            'highThresh': self.highThresh,
        
    def setReversed(self, isReversed):
        self.isReversed = isReversed

    def getDelta(self):
        delta = self.delta
        self.delta = 0
        if self.isReversed:
            return -delta
        else:
            return delta

    def getLimits(self):
        return {
            'range': self.range,
            'lowThresh': self.lowThresh,
            'highThresh': self.highThresh
        }
