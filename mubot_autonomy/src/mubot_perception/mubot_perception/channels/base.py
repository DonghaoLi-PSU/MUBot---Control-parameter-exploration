"""Common interface for sensor channels. New sensors add one class."""


class Channel:
    def update(self, t: float, measurement):
        """Consume one measurement; return this channel's latest estimate."""
        raise NotImplementedError
